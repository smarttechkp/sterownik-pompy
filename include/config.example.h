#pragma once
// Wzor - skopiuj jako config.h i uzupelnij (config.h nie trafia do repozytorium)

// ---- Wartosci domyslne - zapisywane tylko przy PIERWSZYM starcie (pusta konfiguracja) ----
// Pozniej WiFi i dane SUPLA zmienia sie przez strone konfiguracyjna SUPLA:
// przytrzymaj przycisk IO0 kilka sekund -> siec "SUPLA-..." -> http://192.168.4.1
// albo w sieci lokalnej: http://<IP sterownika>
// Wersje publikowane na GitHub budowac z pustym WIFI_SSID/WIFI_PASS (plik .bin jest publiczny)!
#define WIFI_SSID     ""
#define WIFI_PASS     ""
#define SUPLA_SERVER  "svrXXX.supla.org"
#define SUPLA_EMAIL   "twoj@email.pl"

// Nazwa urzadzenia w SUPLA Cloud
#define DEVICE_NAME   "Sterownik pompy"

// Aktualizacja z GitHub (przelacznik "Aktualizuj z GitHub" w SUPLA).
// Plik firmware.bin dolaczony do najnowszego wydania (Release) w publicznym repozytorium.
// Pusty adres = funkcja wylaczona.
#define GITHUB_FW_URL "https://github.com/smarttechkp/sterownik-pompy/releases/latest/download/firmware.bin"

// Strona serwisowa (podglad czujnikow, K1/K2, wgrywanie .bin): http://<IP>:81
#define SERVICE_PORT  81

// Login i haslo do strony http://<IP>:81/update (wgrywanie pliku .bin przez przegladarke)
#define UPDATE_USER   "admin"
#define UPDATE_PASS   "zmien-to-haslo"
