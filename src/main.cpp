// Sterownik pompy obiegowej - ESP32 Relay X2 + 10x DS18B20 + SUPLA
// - czujniki 1-5 -> termostat 1 -> K1, czujniki 6-10 -> termostat 2 -> K2
// - temperatura grupy = srednia z czujnikow, ktore odpowiadaja i NIE sa wylaczone przelacznikiem w SUPLA
// - brak zadnego czujnika w grupie -> pompa stoi
// - strona serwisowa http://<IP>:81 (podglad, reczne K1/K2, wgrywanie .bin na /update)
// - strona konfiguracyjna SUPLA (WiFi, serwer, e-mail) zawsze dostepna pod http://<IP>
// - po zmianie WiFi: jesli nowa siec nie dziala przez 5 min -> powrot do ostatniej dzialajacej
//
// Kanaly SUPLA (kolejnosc stala - zmiana wymaga ponownej rejestracji urzadzenia):
//   0-9   termometry pokoi 1-10
//   10-19 przelaczniki "wylacz czujnik 1-10" (ZAL = czujnik nie jest brany pod uwage)
//   20-21 termometr grupy 1 / grupy 2 (srednia)
//   22-23 termostat K1 / K2
//   24    "Aktualizuj z GitHub" - pobiera firmware.bin z GITHUB_FW_URL i restartuje sterownik

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <SuplaDevice.h>
#include <supla/actions.h>
#include <supla/network/esp_wifi.h>
#include <supla/network/esp_web_server.h>
#include <supla/network/html/device_info.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/wifi_parameters.h>
#include <supla/storage/eeprom.h>
#include <supla/storage/littlefs_config.h>
#include <supla/clock/clock.h>
#include <supla/control/button.h>
#include <supla/control/hvac_base.h>
#include <supla/control/output_interface.h>
#include <supla/control/virtual_relay.h>
#include <supla/sensor/thermometer.h>

#include "config.h"

constexpr int N_SENSORS = 10;
constexpr int GROUP_SIZE = 5;
// Czujnik:                                 1   2   3   4   5   6   7   8   9  10
static const uint8_t SENSOR_PINS[N_SENSORS] = {4, 13, 14, 18, 19, 25, 26, 27, 32, 33};

constexpr int N_RELAYS = 2;
static const uint8_t RELAY_PINS[N_RELAYS] = {16, 17};   // K1, K2

constexpr uint8_t CFG_BUTTON_GPIO = 0;                  // przycisk IO0

constexpr unsigned long READ_INTERVAL_MS = 5000;
constexpr unsigned long CONVERSION_MS    = 800;         // 12 bit = 750 ms
constexpr unsigned long SERVICE_TIMEOUT_MS = 30UL * 60 * 1000;  // tryb reczny wylacza sie sam po 30 min
constexpr unsigned long WIFI_GOOD_MS   = 60UL * 1000;       // po tylu ms polaczenia siec uznana za dzialajaca
constexpr unsigned long WIFI_REVERT_MS = 5UL * 60 * 1000;   // po tylu ms bez polaczenia powrot do dzialajacej sieci

OneWire ow[N_SENSORS];
DallasTemperature dt[N_SENSORS];
float temps[N_SENSORS];

unsigned long lastRequest = 0;
bool conversionPending = false;

Supla::ESPWifi wifi;
Supla::Eeprom eeprom;
Supla::LittleFsConfig configSupla;
Supla::EspWebServer suplaServer;   // strona konfiguracyjna SUPLA (port 80)

WebServer server(SERVICE_PORT);    // strona serwisowa
HTTPUpdateServer httpUpdater;
bool serverStarted = false;
bool suplaWebStarted = false;

Preferences wifiBackup;            // ostatnia dzialajaca siec WiFi
bool wifiWasConnected = false;
bool wifiBackupChecked = false;
unsigned long wifiStateSince = 0;

Supla::Control::VirtualRelay *excluded[N_SENSORS];
Supla::Control::HvacBase *hvac[N_RELAYS];
Supla::Control::VirtualRelay *githubUpdate;
String lastUpdateResult = "-";

// Tryb reczny ze strony serwisowej: przekazniki ustawiane recznie, termostaty ignorowane
bool serviceMode = false;
bool serviceOn[N_RELAYS] = {false, false};
unsigned long serviceStart = 0;

// ------------------------------------------------------------ czujniki
void initSensor(int i) {
  dt[i].begin();
  dt[i].setResolution(12);
  dt[i].setWaitForConversion(false);
}

void requestAll() {
  for (int i = 0; i < N_SENSORS; i++) dt[i].requestTemperatures();
}

void readAll() {
  for (int i = 0; i < N_SENSORS; i++) {
    float t = dt[i].getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) {
      temps[i] = NAN;
      initSensor(i);              // ponowne szukanie - czujnik mogl zostac podlaczony pozniej
    } else {
      temps[i] = t;
    }
  }
}

bool isIncluded(int i) {
  return !excluded[i]->isOn();
}

// Srednia z czujnikow grupy, ktore odpowiadaja i nie sa wylaczone; NAN gdy brak
float groupAverage(int g) {
  float sum = 0;
  int n = 0;
  for (int i = g * GROUP_SIZE; i < (g + 1) * GROUP_SIZE; i++) {
    if (isIncluded(i) && !isnan(temps[i])) {
      sum += temps[i];
      n++;
    }
  }
  return n > 0 ? sum / n : NAN;
}

class RoomThermometer : public Supla::Sensor::Thermometer {
 public:
  explicit RoomThermometer(int idx) : idx(idx) {}
  double getValue() override {
    return isnan(temps[idx]) ? TEMPERATURE_NOT_AVAILABLE : temps[idx];
  }

 private:
  int idx;
};

class GroupThermometer : public Supla::Sensor::Thermometer {
 public:
  explicit GroupThermometer(int group) : group(group) {}
  double getValue() override {
    float t = groupAverage(group);
    return isnan(t) ? TEMPERATURE_NOT_AVAILABLE : t;
  }

 private:
  int group;
};

// ------------------------------------------------------------ przekazniki
// Wyjscie termostatu. W trybie recznym stan z termostatu jest zapamietywany, ale nie trafia na przekaznik.
class PumpOutput : public Supla::Control::OutputInterface {
 public:
  explicit PumpOutput(int n) : n(n) {}
  int getOutputValue() const override { return value; }
  void setOutputValue(int v) override {
    value = v;
    apply();
  }
  bool isOnOffOnly() const override { return true; }
  bool isOn() const { return serviceMode ? serviceOn[n] : value > 0; }
  void apply() {
    digitalWrite(RELAY_PINS[n], isOn() ? HIGH : LOW);
  }

 private:
  int n;
  int value = 0;
};

PumpOutput *pump[N_RELAYS];

void applyRelays() {
  for (int n = 0; n < N_RELAYS; n++) pump[n]->apply();
}

// ------------------------------------------------------------ aktualizacja z GitHub
// Wywolywane z loop() po zalaczeniu przelacznika w SUPLA. Przy sukcesie sterownik sam sie restartuje.
void updateFromGithub() {
  githubUpdate->turnOff();
  if (strlen(GITHUB_FW_URL) == 0) {
    lastUpdateResult = "brak adresu GITHUB_FW_URL w config.h";
    Serial.println("GitHub: " + lastUpdateResult);
    return;
  }
  Serial.printf("GitHub: pobieranie %s\n", GITHUB_FW_URL);
  WiFiClientSecure client;
  client.setInsecure();   // bez sprawdzania certyfikatu - plik i tak musi byc poprawnym programem ESP32
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // GitHub przekierowuje na serwer z plikami
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = httpUpdate.update(client, GITHUB_FW_URL);
  if (ret == HTTP_UPDATE_FAILED) {
    lastUpdateResult = "blad " + String(httpUpdate.getLastError()) + ": " + httpUpdate.getLastErrorString();
  } else {
    lastUpdateResult = "brak nowego programu";
  }
  Serial.println("GitHub: " + lastUpdateResult);
}

// ------------------------------------------------------------ powrot do dzialajacej sieci WiFi
// Siec dzialajaca >= 1 min jest zapamietywana. Gdy przez 5 min nie ma polaczenia, a w konfiguracji SUPLA
// jest inna siec niz zapamietana, przywracana jest zapamietana i sterownik sie restartuje.
void wifiFallback() {
  unsigned long now = millis();
  if (SuplaDevice.getDeviceMode() == Supla::DEVICE_MODE_CONFIG) {
    wifiStateSince = now;   // nie przeszkadzaj w trakcie konfiguracji przez przycisk IO0
    return;
  }
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected != wifiWasConnected) {
    wifiWasConnected = connected;
    wifiStateSince = now;
    wifiBackupChecked = false;
  }
  auto cfg = Supla::Storage::ConfigInstance();
  if (!cfg) return;

  char ssid[128] = {};
  char pass[128] = {};
  cfg->getWiFiSSID(ssid);
  cfg->getWiFiPassword(pass);
  if (connected) {
    if (!wifiBackupChecked && now - wifiStateSince >= WIFI_GOOD_MS) {
      wifiBackupChecked = true;
      if (wifiBackup.getString("ssid") != ssid || wifiBackup.getString("pass") != pass) {
        wifiBackup.putString("ssid", ssid);
        wifiBackup.putString("pass", pass);
        Serial.printf("WiFi: zapamietano dzialajaca siec \"%s\"\n", ssid);
      }
    }
  } else if (now - wifiStateSince >= WIFI_REVERT_MS) {
    wifiStateSince = now;
    String bSsid = wifiBackup.getString("ssid");
    String bPass = wifiBackup.getString("pass");
    if (bSsid.length() > 0 && (bSsid != ssid || bPass != pass)) {
      Serial.printf("WiFi: brak polaczenia z \"%s\" - powrot do \"%s\"\n", ssid, bSsid.c_str());
      cfg->setWiFiSSID(bSsid.c_str());
      cfg->setWiFiPassword(bPass.c_str());
      cfg->commit();
      SuplaDevice.scheduleSoftRestart(1000);
    }
  }
}

// ------------------------------------------------------------ strona serwisowa
String fmtTemp(float t) {
  return isnan(t) ? String("brak") : String(t, 2) + " &deg;C";
}

void handleRoot() {
  String h;
  h.reserve(4500);
  h += F("<!doctype html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta http-equiv='refresh' content='5'>"
         "<title>Sterownik pompy</title>"
         "<style>body{font-family:sans-serif;margin:16px;max-width:520px}"
         "table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:6px;text-align:left}"
         ".off{color:#999}"
         "a.b{display:inline-block;padding:8px 14px;margin:4px;border:1px solid #0f6e56;border-radius:6px;"
         "color:#0f6e56;text-decoration:none}.on{background:#0f6e56;color:#fff!important}"
         ".warn{background:#fff3cd;border:1px solid #e0b100;padding:8px;border-radius:6px}</style>"
         "</head><body><h2>Sterownik pompy</h2>"
         "<table><tr><th>Czujnik</th><th>GPIO</th><th>Temperatura</th><th>W SUPLA</th></tr>");
  for (int i = 0; i < N_SENSORS; i++) {
    bool inc = isIncluded(i);
    h += "<tr" + String(inc ? "" : " class='off'") + "><td>" + String(i + 1) + "</td><td>" +
         String(SENSOR_PINS[i]) + "</td><td>" + fmtTemp(temps[i]) + "</td><td>" +
         (inc ? "liczony" : "wylaczony") + "</td></tr>";
  }
  h += F("</table><h3>Termostaty</h3><table><tr><th></th><th>Srednia</th><th>Zadana</th><th>Przekaznik</th></tr>");
  for (int n = 0; n < N_RELAYS; n++) {
    int16_t sp = hvac[n]->getTemperatureSetpointHeat();
    h += "<tr><td>K" + String(n + 1) + " (czujniki " + String(n * GROUP_SIZE + 1) + "-" +
         String((n + 1) * GROUP_SIZE) + ")</td><td>" + fmtTemp(groupAverage(n)) + "</td><td>" +
         (sp > INT16_MIN ? String(sp / 100.0, 1) + " &deg;C" : String("-")) + "</td><td>" +
         (pump[n]->isOn() ? "ZAL" : "WYL") + "</td></tr>";
  }
  h += F("</table><h3>Reczne sterowanie (do testow)</h3>");
  if (serviceMode) {
    h += F("<p class='warn'>Tryb reczny - termostaty nie steruja przekaznikami. "
           "Wylaczy sie sam po 30 min lub po restarcie.</p>");
  }
  for (int n = 0; n < N_RELAYS; n++) {
    bool on = serviceMode && serviceOn[n];
    h += "K" + String(n + 1) + " (GPIO" + String(RELAY_PINS[n]) + "): ";
    h += "<a class='b" + String(on ? " on" : "") + "' href='/relay?n=" + String(n + 1) + "&s=1'>ZAL</a>";
    h += "<a class='b" + String(serviceMode && !on ? " on" : "") + "' href='/relay?n=" + String(n + 1) +
         "&s=0'>WYL</a><br>";
  }
  if (serviceMode) h += F("<a class='b' href='/auto'>Wroc do automatyki (termostaty)</a>");
  h += "<p>WiFi: " + String(WiFi.RSSI()) + " dBm &middot; IP: " + WiFi.localIP().toString() + "</p>";
  h += "<p>Ostatnia aktualizacja z GitHub: " + lastUpdateResult + "</p>";
  h += "<p><a href='http://" + WiFi.localIP().toString() + "/'>Konfiguracja WiFi / SUPLA</a></p>";
  h += F("<p><a href='/update'>Wgraj nowy program (.bin)</a></p></body></html>");
  server.send(200, "text/html; charset=utf-8", h);
}

void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRelay() {
  int n = server.arg("n").toInt();
  int s = server.arg("s").toInt();
  if (n >= 1 && n <= N_RELAYS) {
    if (!serviceMode) {
      // wejscie w tryb reczny - przekazniki startuja ze stanu, ktory ustawil termostat
      for (int k = 0; k < N_RELAYS; k++) serviceOn[k] = pump[k]->isOn();
      serviceMode = true;
    }
    serviceStart = millis();
    serviceOn[n - 1] = (s == 1);
    applyRelays();
    Serial.printf("Tryb reczny: K%d %s\n", n, s == 1 ? "ZAL" : "WYL");
  }
  redirectHome();
}

void handleAuto() {
  serviceMode = false;
  applyRelays();
  Serial.println("Tryb reczny wylaczony - steruja termostaty");
  redirectHome();
}

void handleJson() {
  String j = "{\"temps\":[";
  for (int i = 0; i < N_SENSORS; i++) {
    j += isnan(temps[i]) ? String("null") : String(temps[i], 2);
    if (i < N_SENSORS - 1) j += ",";
  }
  j += "],\"included\":[";
  for (int i = 0; i < N_SENSORS; i++) {
    j += isIncluded(i) ? "1" : "0";
    if (i < N_SENSORS - 1) j += ",";
  }
  j += "],\"relays\":[" + String(pump[0]->isOn() ? 1 : 0) + "," + String(pump[1]->isOn() ? 1 : 0) +
       "],\"service\":" + String(serviceMode ? 1 : 0) + "}";
  server.send(200, "application/json", j);
}

void startServiceServer() {
  httpUpdater.setup(&server, "/update", UPDATE_USER, UPDATE_PASS);
  server.on("/", handleRoot);
  server.on("/relay", handleRelay);
  server.on("/auto", handleAuto);
  server.on("/json", handleJson);
  server.begin();
  serverStarted = true;
  Serial.printf("Strona serwisowa: http://%s:%d\n", WiFi.localIP().toString().c_str(), SERVICE_PORT);
}

// ------------------------------------------------------------ SUPLA
// Przy pierwszym starcie (pusta konfiguracja) wpisuje WiFi i dane SUPLA z config.h
void presetConfig() {
  auto cfg = Supla::Storage::ConfigInstance();
  if (!cfg) return;
  char buf[128] = {};
  if (strlen(WIFI_SSID) == 0) return;
  if (cfg->getWiFiSSID(buf) && strlen(buf) > 0) return;
  Serial.println("Pierwszy start - zapis domyslnej konfiguracji WiFi/SUPLA z config.h");
  cfg->setWiFiSSID(WIFI_SSID);
  cfg->setWiFiPassword(WIFI_PASS);
  cfg->setSuplaServer(SUPLA_SERVER);
  cfg->setEmail(SUPLA_EMAIL);
  cfg->setSuplaCommProtocolEnabled(true);
  cfg->setDeviceName(DEVICE_NAME);
  cfg->commit();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Sterownik pompy - SUPLA ===");

  for (int n = 0; n < N_RELAYS; n++) {
    pinMode(RELAY_PINS[n], OUTPUT);
    digitalWrite(RELAY_PINS[n], LOW);
  }

  for (int i = 0; i < N_SENSORS; i++) {
    temps[i] = NAN;
    ow[i].begin(SENSOR_PINS[i]);
    dt[i].setOneWire(&ow[i]);
    initSensor(i);
    Serial.printf("Czujnik %2d (GPIO%2d): %s\n", i + 1, SENSOR_PINS[i],
                  dt[i].getDeviceCount() > 0 ? "znaleziony" : "BRAK");
  }

  presetConfig();
  wifiBackup.begin("wifibak", false);

  // Termostat potrzebuje zegara (harmonogram) - czas pobierany z serwera SUPLA
  new Supla::Clock;

  // Strona konfiguracyjna SUPLA
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;

  // Kanaly 0-9: termometry pokoi
  for (int i = 0; i < N_SENSORS; i++) new RoomThermometer(i);

  // Kanaly 10-19: wylaczenie czujnika z liczenia sredniej (stan pamietany po restarcie)
  for (int i = 0; i < N_SENSORS; i++) {
    excluded[i] = new Supla::Control::VirtualRelay();
    excluded[i]->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    excluded[i]->setDefaultStateRestore();
  }

  // Kanaly 20-21: srednia temperatura grupy
  for (int g = 0; g < N_RELAYS; g++) new GroupThermometer(g);

  // Kanaly 22-23: termostaty K1, K2
  for (int n = 0; n < N_RELAYS; n++) {
    pump[n] = new PumpOutput(n);
    hvac[n] = new Supla::Control::HvacBase(pump[n]);
    hvac[n]->getChannel()->setDefault(SUPLA_CHANNELFNC_HVAC_THERMOSTAT);
    hvac[n]->setDefaultSubfunction(SUPLA_HVAC_SUBFUNCTION_HEAT);
    hvac[n]->setMainThermometerChannelNo(20 + n);
    // zalacz ponizej (zadana - histereza), wylacz po przekroczeniu zadanej
    hvac[n]->setUsedAlgorithm(SUPLA_HVAC_ALGORITHM_ON_OFF_SETPOINT_AT_MOST);
    hvac[n]->setTemperatureHisteresisMin(10);    // 0,1 st.C
    hvac[n]->setTemperatureHisteresisMax(500);   // 5 st.C
    hvac[n]->setTemperatureHisteresis(50);       // domyslnie 0,5 st.C
    hvac[n]->setOutputValueOnError(0);           // brak temperatury -> pompa stoi
  }

  // Kanal 24: aktualizacja z GitHub (po restarcie zawsze wylaczony)
  githubUpdate = new Supla::Control::VirtualRelay();
  githubUpdate->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
  githubUpdate->setDefaultStateOff();

  auto cfgButton = new Supla::Control::Button(CFG_BUTTON_GPIO, true, true);
  cfgButton->configureAsConfigButton(&SuplaDevice);

  SuplaDevice.setName(DEVICE_NAME);
  SuplaDevice.begin();
}

void loop() {
  SuplaDevice.iterate();

  if (!serverStarted && WiFi.status() == WL_CONNECTED) startServiceServer();
  if (serverStarted) server.handleClient();
  if (!suplaWebStarted && WiFi.status() == WL_CONNECTED) {
    // strona konfiguracyjna SUPLA normalnie dziala tylko w trybie konfiguracji - wlaczamy ja na stale
    SuplaDevice.handleAction(0, Supla::START_LOCAL_WEB_SERVER);
    suplaWebStarted = true;
  }
  wifiFallback();
  if (githubUpdate->isOn() && WiFi.status() == WL_CONNECTED) updateFromGithub();

  unsigned long now = millis();
  if (serviceMode && now - serviceStart >= SERVICE_TIMEOUT_MS) {
    serviceMode = false;
    applyRelays();
    Serial.println("Tryb reczny wylaczony automatycznie po 30 min");
  }

  if (!conversionPending && now - lastRequest >= READ_INTERVAL_MS) {
    requestAll();
    lastRequest = now;
    conversionPending = true;
  }
  if (conversionPending && now - lastRequest >= CONVERSION_MS) {
    readAll();
    conversionPending = false;
  }
}
