// Sterownik ogrzewania strefowego - ESP32 Relay X2 + 12x DS18B20 + MCP23017 + SUPLA
// - kazdy pokoj: czujnik N -> termostat N -> przekaznik N -> silownik termoelektryczny NC 230 V strefy
// - strefy 1-6 (mieszkanie 1): modul przekaznikow 1 na porcie A ekspandera MCP23017 (I2C, IN1-IN6),
//   strefy 7-12 (mieszkanie 2): modul przekaznikow 2 na porcie B (IN1-IN6)
// - pompa mieszkania 1 / 2 (przekaznik K1 / K2 na plytce -> stycznik 12 V): pracuje, gdy grzeje choc jeden
//   termostat pokoi 1-6 / 7-12 (start z krotkim opoznieniem, stop od razu)
// - brak temperatury -> strefa zamknieta; restart / brak zasilania -> wszystkie strefy zamkniete, pompy stoja
// - po starcie sterownik sprawdza GitHub i sam pobiera nowsza wersje programu
// - strona serwisowa http://<IP>:81 (podglad, reczne sterowanie strefami i pompami, wgrywanie .bin na /update)
// - strona konfiguracyjna SUPLA (WiFi, serwer, e-mail) zawsze dostepna pod http://<IP>
//
// Kanaly SUPLA (kolejnosc stala - zmiana wymaga ponownej rejestracji urzadzenia):
//   0-11  termometry pokoi 1-12
//   12-23 termostaty pokoi 1-12
//   24    "Restart" - restart sterownika (po starcie sprawdza i pobiera nowa wersje z GitHub)
//   25-26 praca pompy mieszkania 1 / 2 (pomiar 0/1 bez jednostki - wykres do nakladania na temperatury)

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
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
#include <supla/sensor/general_purpose_measurement.h>

#include "config.h"

// Wersja publikowana na GitHub (env:github) - plik .bin jest publiczny, wiec bez danych prywatnych.
// Sterownik ma je juz zapisane w swojej pamieci z pierwszego wgrania.
#ifdef PUBLIC_BUILD
#undef WIFI_SSID
#undef WIFI_PASS
#undef SUPLA_SERVER
#undef SUPLA_EMAIL
#undef UPDATE_USER
#undef UPDATE_PASS
#define WIFI_SSID    ""
#define WIFI_PASS    ""
#define SUPLA_SERVER ""
#define SUPLA_EMAIL  ""
#define UPDATE_USER  ""
#define UPDATE_PASS  ""
#endif

#ifndef FW_VERSION
#define FW_VERSION "lokalna"
#endif

constexpr int N_ZONES = 12;
// Pokoj/strefa (= numer czujnika na listwach): 1   2   3   4   5   6   7   8   9  10  11  12
static const uint8_t SENSOR_PINS[N_ZONES] =    {4, 13, 14, 18, 19, 25, 26, 27, 32, 33, 5, 15};
// Strefy 1-6: mieszkanie 1, strefy 7-12: mieszkanie 2 (rozroznienie przez lokalizacje w SUPLA).
// Nazwy kanalow SUPLA bierze tylko przy pierwszej rejestracji - pozniej zmiana w aplikacji.
static const char *const ZONE_NAMES[N_ZONES] = {"Salon", "Sypialnia", "Pokój", "Korytarz", "Łazienka", "Rezerwa",
                                                "Salon", "Sypialnia", "Pokój", "Korytarz", "Łazienka", "Rezerwa"};

// Pompy mieszkan: przekazniki na plytce (stan wysoki = zalaczony, potwierdzone) -> cewki stycznikow 12 V.
// Strefy 1-6 -> pompa M1 (K1), strefy 7-12 -> pompa M2 (K2)
constexpr int N_PUMPS = 2;
constexpr int ZONES_PER_PUMP = N_ZONES / N_PUMPS;
static const uint8_t PUMP_PINS[N_PUMPS] = {16, 17};   // K1, K2
static const char *const PUMP_NAMES[N_PUMPS] = {"Pompa M1", "Pompa M2"};
constexpr unsigned long PUMP_START_DELAY_MS = 10UL * 1000;  // pompa rusza 10 s po starcie grzania pierwszej strefy

// MCP23017: port A 0-5 -> modul przekaznikow 1 IN1-IN6 (strefy 1-6, mieszkanie 1), A6-A7 wolne,
//           port B 0-5 -> modul przekaznikow 2 IN1-IN6 (strefy 7-12, mieszkanie 2), B6-B7 wolne.
// Moduly wyzwalane stanem niskim, wejscia modulow zasilane z 5 V.
// Pin ekspandera: przekaznik WL = wyjscie ze stanem 0, WYL = wejscie (odlaczony, modul podciaga do 5 V).
// Po wlaczeniu zasilania MCP23017 ma wszystkie piny jako wejscia -> wszystko wylaczone, zanim ruszy program.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t MCP_ADDR = 0x20;                      // A0-A2 do GND
constexpr uint8_t MCP_IODIRA = 0x00;                    // IODIRA, IODIRB (kolejne adresy)
constexpr uint8_t MCP_OLATA = 0x14;                     // OLATA, OLATB
constexpr unsigned long MCP_REFRESH_MS = 1000;          // ponowny zapis wyjsc / szukanie ekspandera

constexpr uint8_t CFG_BUTTON_GPIO = 0;                  // przycisk IO0
constexpr uint8_t LED_GPIO = 23;                        // dioda na plytce
constexpr bool LED_ACTIVE_HIGH = true;                  // true: stan wysoki zapala diode

constexpr unsigned long READ_INTERVAL_MS = 5000;
constexpr unsigned long CONVERSION_MS    = 800;         // 12 bit = 750 ms
constexpr unsigned long SERVICE_TIMEOUT_MS = 30UL * 60 * 1000;  // tryb reczny wylacza sie sam po 30 min
constexpr unsigned long OTA_CONFIRM_MS = 60UL * 1000;           // nowa wersja zatwierdzana po 1 min pracy z WiFi
constexpr unsigned long OTA_ROLLBACK_MS = 10UL * 60 * 1000;     // bez WiFi przez 10 min -> powrot do poprzedniej

OneWire ow[N_ZONES];
DallasTemperature dt[N_ZONES];
float temps[N_ZONES];

unsigned long lastRequest = 0;
bool conversionPending = false;

Supla::ESPWifi wifi;
Supla::Eeprom eeprom;
// Bufor konfiguracji: domyslne 1024 B to za malo (nadmiar jest po cichu pomijany przy zapisie,
// po restarcie brakuje WiFi). 12 termostatow z harmonogramem -> 16 kB.
Supla::LittleFsConfig configSupla(16384);
Supla::EspWebServer suplaServer;   // strona konfiguracyjna SUPLA (port 80)

WebServer server(SERVICE_PORT);    // strona serwisowa
HTTPUpdateServer httpUpdater;
bool serverStarted = false;
bool suplaWebStarted = false;

Preferences updateLogin;           // login/haslo do /update - zapamietane, bo wersja z GitHub ich nie zawiera
String updateUser;
String updatePass;

Preferences githubPrefs;           // wersja, ktorej instalacja sie nie powiodla (powrot do poprzedniej)

Supla::Control::HvacBase *hvac[N_ZONES];
Supla::Control::VirtualRelay *restartButton;
String lastUpdateResult = "-";
bool githubCheckDone = false;
bool forceGithubUpdate = false;
bool otaPendingVerify = false;
unsigned long wifiSince = 0;

// Tryb reczny ze strony serwisowej: przekazniki ustawiane recznie, termostaty i logika pomp ignorowane.
// serviceOn: strefy 0-11, potem pompy.
bool serviceMode = false;
bool serviceOn[N_ZONES + N_PUMPS] = {};
unsigned long serviceStart = 0;

bool mcpOk = false;
unsigned long lastMcpRefresh = 0;

// ------------------------------------------------------------ dziennik zdarzen (http://<IP>:81/log)
// Pamiec RTC przetrwa restart programowy (nie przetrwa odlaczenia zasilania).
constexpr int LOG_LINES = 40;
constexpr int LOG_LINE_LEN = 72;
constexpr uint32_t LOG_MAGIC = 0x50554D50;
struct EventLog {
  uint32_t magic;
  uint32_t boot;
  uint16_t next;
  char lines[LOG_LINES][LOG_LINE_LEN];
};
RTC_NOINIT_ATTR EventLog eventLog;

void logEvent(const char *fmt, ...) {
  char msg[LOG_LINE_LEN - 16];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  snprintf(eventLog.lines[eventLog.next], LOG_LINE_LEN, "#%lu %6lus %s",
           (unsigned long)eventLog.boot, millis() / 1000, msg);
  eventLog.next = (eventLog.next + 1) % LOG_LINES;
  Serial.printf("LOG: %s\n", msg);
}

void initEventLog() {
  if (eventLog.magic != LOG_MAGIC || eventLog.next >= LOG_LINES) {
    memset(&eventLog, 0, sizeof(eventLog));
    eventLog.magic = LOG_MAGIC;
  }
  eventLog.boot++;
  static const char *const reasons[] = {"nieznany", "wlaczenie zasilania", "zewnetrzny", "programowy",
                                        "blad programu (panic)", "watchdog przerwan", "watchdog zadan",
                                        "watchdog", "wybudzenie", "spadek napiecia (brownout)", "SDIO"};
  int r = esp_reset_reason();
  logEvent("START %s, przyczyna: %s", FW_VERSION, r < 11 ? reasons[r] : "inny");
}

// Zapisuje zmiany trybu SUPLA, statusu polaczenia i WiFi
void watchState() {
  static int lastMode = -1;
  static int lastStatus = -1;
  static int lastWifi = -1;
  int mode = SuplaDevice.getDeviceMode();
  if (mode != lastMode) {
    lastMode = mode;
    logEvent("tryb SUPLA: %d%s", mode, mode == Supla::DEVICE_MODE_CONFIG ? " (KONFIGURACJA)" : "");
  }
  int status = SuplaDevice.getCurrentStatus();
  if (status != lastStatus) {
    lastStatus = status;
    logEvent("status SUPLA: %d", status);
  }
  int wifi = WiFi.status() == WL_CONNECTED ? 1 : 0;
  if (wifi != lastWifi) {
    lastWifi = wifi;
    wifiSince = millis();
    if (wifi) logEvent("WiFi polaczone, IP %s", WiFi.localIP().toString().c_str());
    else logEvent("WiFi rozlaczone");
  }
}

// ------------------------------------------------------------ czujniki
void initSensor(int i) {
  dt[i].begin();
  dt[i].setResolution(12);
  dt[i].setWaitForConversion(false);
}

void requestAll() {
  for (int i = 0; i < N_ZONES; i++) dt[i].requestTemperatures();
}

void readAll() {
  for (int i = 0; i < N_ZONES; i++) {
    float t = dt[i].getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) {
      temps[i] = NAN;
      initSensor(i);              // ponowne szukanie - czujnik mogl zostac podlaczony pozniej
    } else {
      temps[i] = t;
    }
  }
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

// ------------------------------------------------------------ przekazniki / silowniki
class ZoneOutput;
ZoneOutput *zone[N_ZONES];
void writeMcp();

// Wyjscie termostatu. W trybie recznym stan z termostatu jest zapamietywany, ale nie trafia na przekaznik.
class ZoneOutput : public Supla::Control::OutputInterface {
 public:
  explicit ZoneOutput(int n) : n(n) {}
  int getOutputValue() const override { return value; }
  void setOutputValue(int v) override {
    value = v;
    writeMcp();
  }
  bool isOnOffOnly() const override { return true; }
  bool isOn() const { return serviceMode ? serviceOn[n] : value > 0; }
  bool thermostatOn() const { return value > 0; }   // stan z termostatu (takze w trybie recznym)

 private:
  int n;
  int value = 0;
};

// ------------------------------------------------------------ pompy
// Pompa mieszkania pracuje, gdy grzeje choc jeden termostat jego pokoi - po PUMP_START_DELAY_MS od startu grzania
// pierwszego. Koniec grzania ostatniego zatrzymuje pompe od razu.
bool pumpAuto[N_PUMPS] = {};
unsigned long pumpDemandSince[N_PUMPS] = {};

bool pumpIsOn(int p) { return serviceMode ? serviceOn[N_ZONES + p] : pumpAuto[p]; }

bool anyZoneOpen(int p) {
  for (int n = p * ZONES_PER_PUMP; n < (p + 1) * ZONES_PER_PUMP; n++) {
    if (zone[n] && zone[n]->thermostatOn()) return true;
  }
  return false;
}

// Wywolywane w loop(); zwraca true, gdy stan ktorejs pompy (w trybie automatycznym) sie zmienil
bool updatePumps() {
  bool changed = false;
  unsigned long now = millis();
  for (int p = 0; p < N_PUMPS; p++) {
    bool demand = anyZoneOpen(p);
    if (!demand) pumpDemandSince[p] = 0;
    else if (pumpDemandSince[p] == 0) pumpDemandSince[p] = now | 1;   // 0 = brak zapotrzebowania
    bool on = demand && now - pumpDemandSince[p] >= PUMP_START_DELAY_MS;
    if (on != pumpAuto[p]) {
      pumpAuto[p] = on;
      changed = true;
      logEvent("%s %s", PUMP_NAMES[p], on ? "ZAL" : "WYL");
    }
  }
  return changed;
}

// Stan pompy jako liczba 0/1 - historia w SUPLA
class PumpState : public Supla::Sensor::GeneralPurposeMeasurement {
 public:
  explicit PumpState(int p) : p(p) {}
  double getValue() override { return pumpIsOn(p) ? 1 : 0; }

 private:
  int p;
};

void applyPumps() {
  for (int p = 0; p < N_PUMPS; p++) digitalWrite(PUMP_PINS[p], pumpIsOn(p) ? HIGH : LOW);
}

// Kierunek pinow portu: bit 1 = wejscie (przekaznik WYL), 0 = wyjscie ze stanem 0 (ZAL)
// Port A 0-5: strefy 1-6 (first = 0), port B 0-5: strefy 7-12 (first = ZONES_PER_PUMP)
uint8_t mcpDir(int first) {
  uint8_t bits = 0xFF;
  for (int k = 0; k < ZONES_PER_PUMP && first + k < N_ZONES; k++) {
    if (zone[first + k] && zone[first + k]->isOn()) bits &= ~(1 << k);
  }
  return bits;
}

// Zapis rejestru portu A i B (kolejne adresy)
bool mcpWrite(uint8_t reg, uint8_t a, uint8_t b) {
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(reg);
  Wire.write(a);
  Wire.write(b);
  return Wire.endTransmission() == 0;
}

// Najpierw stan wyjsc = 0 (gdyby ekspander sie zresetowal), potem kierunek pinow
bool mcpWriteAll() {
  return mcpWrite(MCP_OLATA, 0x00, 0x00) && mcpWrite(MCP_IODIRA, mcpDir(0), mcpDir(ZONES_PER_PUMP));
}

void writeMcp() {
  if (mcpOk && !mcpWriteAll()) {
    mcpOk = false;
    logEvent("ekspander MCP23017: brak odpowiedzi");
  }
}

void detectMcp() {
  if (mcpWriteAll()) {
    mcpOk = true;
    logEvent("ekspander MCP23017: znaleziony");
  }
}

void refreshMcp() {
  if (mcpOk) writeMcp();     // zapis co sekunde - przywraca stan, gdyby ekspander sie zresetowal
  else detectMcp();
}

void applyRelays() {
  writeMcp();
  applyPumps();
}

// ------------------------------------------------------------ aktualizacja z GitHub
// Adres strony najnowszego wydania - z GITHUB_FW_URL ".../releases/latest/download/firmware.bin"
String latestReleaseUrl() {
  String url = GITHUB_FW_URL;
  int p = url.indexOf("/releases/latest");
  return p < 0 ? String() : url.substring(0, p) + "/releases/latest";
}

// Numer najnowszego wydania (np. "v1.1.7") - GitHub przekierowuje .../releases/latest na .../releases/tag/<numer>
String latestReleaseTag() {
  String url = latestReleaseUrl();
  if (url.length() == 0) return String();
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  const char *headers[] = {"Location"};
  http.collectHeaders(headers, 1);
  String tag;
  if (http.begin(client, url)) {
    int code = http.GET();
    String location = http.header("Location");
    int p = location.lastIndexOf("/tag/");
    if ((code == 301 || code == 302) && p >= 0) tag = location.substring(p + 5);
    http.end();
  }
  return tag;
}

void downloadFromGithub() {
  logEvent("GitHub: pobieranie, RAM %u kB (min %u kB)", ESP.getFreeHeap() / 1024, ESP.getMinFreeHeap() / 1024);
  WiFiClientSecure client;
  client.setInsecure();   // bez sprawdzania certyfikatu - plik i tak musi byc poprawnym programem ESP32
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // GitHub przekierowuje na serwer z plikami
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = httpUpdate.update(client, GITHUB_FW_URL);
  logEvent("GitHub: RAM min %u kB", ESP.getMinFreeHeap() / 1024);
  if (ret == HTTP_UPDATE_FAILED) {
    lastUpdateResult = "blad " + String(httpUpdate.getLastError()) + ": " + httpUpdate.getLastErrorString();
  } else {
    lastUpdateResult = "brak nowego programu";
  }
  githubPrefs.putString("failed", "");   // nieudane pobranie to nie wina wersji - mozna probowac ponownie
  logEvent("GitHub: %s", lastUpdateResult.c_str());
}

// Wywolywane raz po polaczeniu z WiFi (oraz na zadanie ze strony serwisowej).
// Wersja lokalna (wgrana z laptopa) nie jest automatycznie nadpisywana.
void checkGithub(bool force) {
  if (strlen(GITHUB_FW_URL) == 0) {
    lastUpdateResult = "brak adresu GITHUB_FW_URL w config.h";
    return;
  }
  if (force) {
    githubPrefs.putString("failed", "");
    downloadFromGithub();
    return;
  }
  if (strcmp(FW_VERSION, "lokalna") == 0) {
    lastUpdateResult = "wersja lokalna - bez automatycznej aktualizacji";
    logEvent("GitHub: %s", lastUpdateResult.c_str());
    return;
  }
  String tag = latestReleaseTag();
  if (tag.length() == 0) {
    lastUpdateResult = "nie udalo sie sprawdzic wersji na GitHub";
  } else if (tag == "v" FW_VERSION) {
    lastUpdateResult = "aktualna (" + tag + ")";
  } else if (tag == githubPrefs.getString("failed")) {
    lastUpdateResult = tag + " pominieta - wczesniej nie dzialala (powrot do poprzedniej)";
  } else {
    logEvent("GitHub: nowa wersja %s", tag.c_str());
    githubPrefs.putString("failed", tag);   // skasowane przez nowa wersje po zatwierdzeniu
    downloadFromGithub();
    return;
  }
  logEvent("GitHub: %s", lastUpdateResult.c_str());
}

// Nowa wersja programu startuje jako "do sprawdzenia". Jesli sie zawiesi / zrestartuje przed zatwierdzeniem,
// bootloader wraca do poprzedniej. Zatwierdzenie: 1 min pracy z WiFi.
extern "C" bool verifyRollbackLater() { return true; }   // nadpisuje funkcje z rdzenia Arduino (plik .c)

void checkOtaPending() {
  esp_ota_img_states_t state;
  otaPendingVerify = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
                     state == ESP_OTA_IMG_PENDING_VERIFY;
  if (otaPendingVerify) logEvent("nowa wersja - czeka na zatwierdzenie");
}

void confirmOta() {
  if (!otaPendingVerify) return;
  if (WiFi.status() == WL_CONNECTED && millis() - wifiSince >= OTA_CONFIRM_MS) {
    esp_ota_mark_app_valid_cancel_rollback();
    otaPendingVerify = false;
    githubPrefs.putString("failed", "");
    logEvent("nowa wersja zatwierdzona");
  } else if (millis() >= OTA_ROLLBACK_MS) {
    logEvent("nowa wersja bez WiFi przez 10 min - powrot do poprzedniej");
    delay(100);
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
}

// ------------------------------------------------------------ strona serwisowa
String fmtTemp(float t) {
  return isnan(t) ? String("brak") : String(t, 2) + " &deg;C";
}

String fmtSetpoint(int n) {
  int16_t sp = hvac[n]->getTemperatureSetpointHeat();
  return sp > INT16_MIN ? String(sp / 100.0, 1) + " &deg;C" : String("-");
}

String outputName(int n) {
  int m = n / ZONES_PER_PUMP;          // 0 = port A / modul 1, 1 = port B / modul 2
  int k = n % ZONES_PER_PUMP;
  return String("MCP ") + (m == 0 ? "A" : "B") + String(k) + " / modul " + String(m + 1) + " IN" + String(k + 1);
}

// Przyciski ZAL/WYL trybu recznego; n = numer na stronie (strefy 1-12, pompy 13-14)
String manualButtons(int n) {
  bool on = serviceMode && serviceOn[n - 1];
  String nr = String(n);
  return "<a class='b" + String(on ? " on" : "") + "' href='/relay?n=" + nr + "&s=1'>ZAL</a>" +
         "<a class='b" + String(serviceMode && !on ? " on" : "") + "' href='/relay?n=" + nr + "&s=0'>WYL</a>";
}

void handleRoot() {
  String h;
  h.reserve(8000);
  h += F("<!doctype html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta http-equiv='refresh' content='5'>"
         "<title>Sterownik ogrzewania</title>"
         "<style>body{font-family:sans-serif;margin:16px;max-width:640px}"
         "table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:5px;text-align:left}"
         "a.b{display:inline-block;padding:5px 10px;margin:2px;border:1px solid #0f6e56;border-radius:6px;"
         "color:#0f6e56;text-decoration:none}.on{background:#0f6e56;color:#fff!important}"
         ".warn{background:#fff3cd;border:1px solid #e0b100;padding:8px;border-radius:6px}</style>"
         "</head><body><h2>Sterownik ogrzewania</h2>");
  if (serviceMode) {
    h += F("<p class='warn'>Tryb reczny - termostaty i logika pomp nie steruja przekaznikami. "
           "Wylaczy sie sam po 30 min lub po restarcie. <a href='/auto'>Wroc do automatyki</a></p>");
  }
  h += F("<table><tr><th>Strefa</th><th>Temperatura</th><th>Zadana</th><th>Silownik</th><th>Wyjscie</th>"
         "<th>Recznie</th></tr>");
  for (int n = 0; n < N_ZONES; n++) {
    h += "<tr><td>" + String(n + 1) + " " + ZONE_NAMES[n] + " (M" + String(n / ZONES_PER_PUMP + 1) + ", GPIO" +
         String(SENSOR_PINS[n]) + ")</td><td>" + fmtTemp(temps[n]) + "</td><td>" + fmtSetpoint(n) + "</td><td>" +
         (zone[n]->isOn() ? "OTWARTY" : "zamkniety") + "</td><td>" + outputName(n) + "</td><td>" +
         manualButtons(n + 1) + "</td></tr>";
  }
  h += F("</table><h3>Pompy</h3><table><tr><th>Pompa</th><th>Stan</th><th>Wyjscie</th><th>Recznie</th></tr>");
  for (int p = 0; p < N_PUMPS; p++) {
    String state = pumpIsOn(p) ? String("PRACUJE") : String("stoi");
    if (!serviceMode && !pumpAuto[p] && pumpDemandSince[p]) {
      state += " (start za " + String((PUMP_START_DELAY_MS - (millis() - pumpDemandSince[p])) / 1000) + " s)";
    }
    h += "<tr><td>" + String(PUMP_NAMES[p]) + " (strefy " + String(p * ZONES_PER_PUMP + 1) + "-" +
         String((p + 1) * ZONES_PER_PUMP) + ")</td><td>" + state + "</td><td>K" + String(p + 1) + " (GPIO" +
         String(PUMP_PINS[p]) + ")</td><td>" + manualButtons(N_ZONES + p + 1) + "</td></tr>";
  }
  h += F("</table>");
  h += "<p>Ekspander MCP23017 (strefy 1-12): " +
       String(mcpOk ? "OK" : "BRAK - strefy nie przelaczaja") + "</p>";
  h += "<p>WiFi: " + String(WiFi.RSSI()) + " dBm &middot; IP: " + WiFi.localIP().toString() + "</p>";
  h += "<p>Pamiec RAM: wolna " + String(ESP.getFreeHeap() / 1024) + " kB &middot; najmniej od startu " +
       String(ESP.getMinFreeHeap() / 1024) + " kB &middot; najwiekszy blok " +
       String(ESP.getMaxAllocHeap() / 1024) + " kB</p>";
  h += "<p>Wersja programu: " FW_VERSION " &middot; GitHub: " + lastUpdateResult + "</p>";
  h += "<p><a href='http://" + WiFi.localIP().toString() + "/'>Konfiguracja WiFi / SUPLA</a></p>";
  h += F("<p><a href='/update'>Wgraj nowy program (.bin)</a> &middot; <a href='/log'>Dziennik zdarzen</a> &middot; "
         "<a href='/github' onclick=\"return confirm('Pobrac program z GitHub i zrestartowac?')\">"
         "Wymus pobranie z GitHub</a></p></body></html>");
  server.send(200, "text/html; charset=utf-8", h);
}

void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRelay() {
  int n = server.arg("n").toInt();
  int s = server.arg("s").toInt();
  if (n >= 1 && n <= N_ZONES + N_PUMPS) {
    if (!serviceMode) {
      // wejscie w tryb reczny - przekazniki startuja ze stanu, ktory ustawily termostaty i logika pomp
      for (int k = 0; k < N_ZONES; k++) serviceOn[k] = zone[k]->isOn();
      for (int p = 0; p < N_PUMPS; p++) serviceOn[N_ZONES + p] = pumpAuto[p];
      serviceMode = true;
    }
    serviceStart = millis();
    serviceOn[n - 1] = (s == 1);
    applyRelays();
    Serial.printf("Tryb reczny: wyjscie %d %s\n", n, s == 1 ? "ZAL" : "WYL");
  }
  redirectHome();
}

void handleAuto() {
  serviceMode = false;
  applyRelays();
  Serial.println("Tryb reczny wylaczony - steruja termostaty");
  redirectHome();
}

void handleGithub() {
  forceGithubUpdate = true;
  server.send(200, "text/html; charset=utf-8",
              "<meta http-equiv='refresh' content='60;url=/'>Pobieranie programu z GitHub - "
              "sterownik zrestartuje sie po wgraniu (ok. 1 min).");
}

String jsonList(String (*item)(int)) {
  String j = "[";
  for (int n = 0; n < N_ZONES; n++) {
    if (n) j += ",";
    j += item(n);
  }
  return j + "]";
}

void handleJson() {
  String j = "{\"temps\":" + jsonList([](int n) {
    return isnan(temps[n]) ? String("null") : String(temps[n], 2);
  });
  j += ",\"setpoints\":" + jsonList([](int n) {
    int16_t sp = hvac[n]->getTemperatureSetpointHeat();
    return sp > INT16_MIN ? String(sp / 100.0, 1) : String("null");
  });
  j += ",\"zones\":" + jsonList([](int n) { return String(zone[n]->isOn() ? 1 : 0); });
  j += ",\"pumps\":[";
  for (int p = 0; p < N_PUMPS; p++) j += String(p ? "," : "") + (pumpIsOn(p) ? "1" : "0");
  j += "]";
  j += ",\"mcp\":" + String(mcpOk ? 1 : 0) + ",\"service\":" + String(serviceMode ? 1 : 0) +
       ",\"version\":\"" FW_VERSION "\"";
  // pamiec RAM w bajtach: wolna teraz, najmniej wolnej od startu, najwiekszy wolny blok (HTTPS potrzebuje ok. 40-50 kB)
  j += ",\"heap_free\":" + String(ESP.getFreeHeap()) + ",\"heap_min\":" + String(ESP.getMinFreeHeap()) +
       ",\"heap_max_block\":" + String(ESP.getMaxAllocHeap()) + ",\"uptime_s\":" + String(millis() / 1000) + "}";
  server.send(200, "application/json", j);
}

void handleLog() {
  String t;
  t.reserve(LOG_LINES * LOG_LINE_LEN);
  for (int k = 0; k < LOG_LINES; k++) {
    const char *line = eventLog.lines[(eventLog.next + k) % LOG_LINES];
    if (line[0]) {
      t += line;
      t += '\n';
    }
  }
  server.send(200, "text/plain; charset=utf-8", t);
}

void startServiceServer() {
  server.on("/log", handleLog);
  if (updatePass.length() == 0) Serial.println("UWAGA: brak hasla do /update - strona bez logowania");
  httpUpdater.setup(&server, "/update", updateUser.c_str(), updatePass.c_str());
  server.on("/", handleRoot);
  server.on("/relay", handleRelay);
  server.on("/auto", handleAuto);
  server.on("/github", handleGithub);
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

// Dioda: miga gdy brak WiFi, swieci na stale gdy WiFi polaczone
void updateLed() {
  bool on = WiFi.status() == WL_CONNECTED || (millis() / 500) % 2;
  digitalWrite(LED_GPIO, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Sterownik ogrzewania - SUPLA ===");
  initEventLog();
  checkOtaPending();

  for (int p = 0; p < N_PUMPS; p++) {
    pinMode(PUMP_PINS[p], OUTPUT);
    digitalWrite(PUMP_PINS[p], LOW);
  }
  pinMode(LED_GPIO, OUTPUT);

  for (int i = 0; i < N_ZONES; i++) {
    temps[i] = NAN;
    ow[i].begin(SENSOR_PINS[i]);
    dt[i].setOneWire(&ow[i]);
    initSensor(i);
    Serial.printf("Czujnik %2d (GPIO%2d): %s\n", i + 1, SENSOR_PINS[i],
                  dt[i].getDeviceCount() > 0 ? "znaleziony" : "BRAK");
  }

  presetConfig();

  updateLogin.begin("update", false);
  if (strlen(UPDATE_PASS) > 0 &&
      (updateLogin.getString("user") != UPDATE_USER || updateLogin.getString("pass") != UPDATE_PASS)) {
    updateLogin.putString("user", UPDATE_USER);
    updateLogin.putString("pass", UPDATE_PASS);
  }
  updateUser = updateLogin.getString("user", UPDATE_USER);
  updatePass = updateLogin.getString("pass", UPDATE_PASS);
  githubPrefs.begin("github", false);

  eeprom.setStateSavePeriod(5000);

  // Termostat potrzebuje zegara (harmonogram) - czas pobierany z serwera SUPLA
  new Supla::Clock;

  // Strona konfiguracyjna SUPLA
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;

  char caption[40];

  // Kanaly 0-11: termometry pokoi
  for (int i = 0; i < N_ZONES; i++) {
    auto t = new RoomThermometer(i);
    snprintf(caption, sizeof(caption), "%s temperatura", ZONE_NAMES[i]);
    t->setInitialCaption(caption);
  }

  // Kanaly 12-23: termostaty pokoi. Minimalny czas wl./wyl. ustawia sie w SUPLA (domyslnie brak).
  for (int n = 0; n < N_ZONES; n++) {
    zone[n] = new ZoneOutput(n);
    hvac[n] = new Supla::Control::HvacBase(zone[n]);
    hvac[n]->getChannel()->setDefault(SUPLA_CHANNELFNC_HVAC_THERMOSTAT);
    hvac[n]->setDefaultSubfunction(SUPLA_HVAC_SUBFUNCTION_HEAT);
    hvac[n]->setMainThermometerChannelNo(n);
    // otworz strefe ponizej (zadana - histereza), zamknij po przekroczeniu zadanej
    hvac[n]->setUsedAlgorithm(SUPLA_HVAC_ALGORITHM_ON_OFF_SETPOINT_AT_MOST);
    hvac[n]->setTemperatureHisteresisMin(10);    // 0,1 st.C
    hvac[n]->setTemperatureHisteresisMax(500);   // 5 st.C
    hvac[n]->setTemperatureHisteresis(30);       // domyslnie 0,3 st.C
    hvac[n]->setOutputValueOnError(0);           // brak temperatury -> strefa zamknieta
    hvac[n]->setInitialCaption(ZONE_NAMES[n]);
  }

  // Kanal 24: restart (po restarcie zawsze wylaczony)
  restartButton = new Supla::Control::VirtualRelay();
  restartButton->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
  restartButton->setDefaultStateOff();
  restartButton->setInitialCaption("Restart (+ aktualizacja z GitHub)");

  // Kanaly 25-26: praca pomp
  for (int p = 0; p < N_PUMPS; p++) {
    auto state = new PumpState(p);
    state->setInitialCaption(PUMP_NAMES[p]);
    state->setDefaultValuePrecision(0);
    state->setDefaultRefreshIntervalMs(1000);
    state->setKeepHistory(1, false);   // historia od razu wlaczona (wysylane przy pierwszej rejestracji kanalu)
  }

  // Ekspander - po utworzeniu stref (stan wyjsc liczony z zone[])
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(20);
  detectMcp();
  if (!mcpOk) logEvent("ekspander MCP23017: brak");

  auto cfgButton = new Supla::Control::Button(CFG_BUTTON_GPIO, true, true);
  cfgButton->configureAsConfigButton(&SuplaDevice);

  SuplaDevice.setName(DEVICE_NAME);
  SuplaDevice.setSwVersion(FW_VERSION);
  SuplaDevice.begin();
}

void loop() {
  SuplaDevice.iterate();
  watchState();

  bool wifiOk = WiFi.status() == WL_CONNECTED;
  if (!serverStarted && wifiOk) startServiceServer();
  if (serverStarted) server.handleClient();
  if (!suplaWebStarted && wifiOk) {
    // strona konfiguracyjna SUPLA normalnie dziala tylko w trybie konfiguracji - wlaczamy ja na stale
    SuplaDevice.handleAction(0, Supla::START_LOCAL_WEB_SERVER);
    suplaWebStarted = true;
  }
  if (wifiOk && (!githubCheckDone || forceGithubUpdate)) {
    checkGithub(forceGithubUpdate);
    githubCheckDone = true;
    forceGithubUpdate = false;
  }
  if (restartButton->isOn()) {
    restartButton->turnOff();
    logEvent("restart z SUPLA");
    SuplaDevice.scheduleSoftRestart(2000);   // chwila na wyslanie stanu przycisku do SUPLA
  }
  confirmOta();
  updateLed();

  if (updatePumps()) applyPumps();

  unsigned long now = millis();
  if (now - lastMcpRefresh >= MCP_REFRESH_MS) {
    lastMcpRefresh = now;
    refreshMcp();
  }
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
