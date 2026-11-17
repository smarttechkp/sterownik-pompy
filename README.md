# Sterownik ogrzewania strefowego – ESP32 + 10× DS18B20 + SUPLA

Program dla płytki „ESP32_Relay X2” (ESP32, 2 przekaźniki) z ekspanderem MCP23017 i modułem 8 przekaźników.
Każdy pokój ma własny termostat w aplikacji SUPLA, który otwiera lub zamyka siłownik termoelektryczny
(NC 230 V) swojej strefy.

- czujnik N → termostat N → przekaźnik N → siłownik strefy N,
- temperatura zadana i histereza osobno dla każdego pokoju (krok 0,1 °C), minimalny czas pracy – w ustawieniach termostatu w SUPLA,
- pompa mieszkania 1 / 2 pracuje, gdy otwarty jest choć jeden siłownik pokoi 1–5 / 6–10 (start 2 min po otwarciu
  pierwszego siłownika, stop od razu po zamknięciu ostatniego),
- brak temperatury → strefa zamknięta; restart / brak zasilania → wszystkie strefy zamknięte, pompy stoją.

## Wyjścia

| Strefa | Wyjście |
|---|---|
| 1–2 | przekaźniki K1 (GPIO16) / K2 (GPIO17) na płytce |
| 3–10 | MCP23017 (I2C: SDA GPIO21, SCL GPIO22, adres 0x20), port A0–A7 → moduł przekaźników 1 IN1–IN8 |
| pompy M1, M2 | MCP23017 port B0, B1 → moduł przekaźników 2 IN1, IN2 (B2–B7 wolne) |

Bez ekspandera program działa normalnie – strefy 3–10 i pompy są liczone i widoczne, ale nic nie przełączają.
Moduły przekaźników wyzwalane stanem niskim, wejścia (VCC) zasilane z 5 V, zworki JD-VCC zdjęte. Pin ekspandera: ZAŁ = wyjście 0, WYŁ = wejście (odłączony).

## Kanały SUPLA

| Nr | Kanał |
|---|---|
| 0–9 | termometry „<pokój> temperatura” |
| 10–19 | termostaty „<pokój>” |
| 20 | „Restart” (po restarcie sterownik sprawdza GitHub i pobiera nową wersję) |
| 21–30 | „<pokój> siłownik” – praca 0/1 z historią (do wykresów) |
| 31–32 | „Pompa M1”, „Pompa M2” – praca 0/1 z historią |

Pokoje (strefy 1–5 = mieszkanie 1, 6–10 = mieszkanie 2): Salon, Sypialnia, Pokój, Korytarz, Łazienka – `ZONE_NAMES` w `src/main.cpp`.
Nazwy trafiają do SUPLA tylko przy pierwszej rejestracji urządzenia.

## Strony WWW sterownika

- `http://<IP>` – konfiguracja WiFi i SUPLA,
- `http://<IP>:81` – podgląd stref, ręczne sterowanie, pamięć RAM, `/update` – wgrywanie pliku `.bin`,
  `/json` – dane, `/log` – dziennik zdarzeń, `/github` – wymuszone pobranie programu z GitHub.

## Budowanie

1. Skopiuj `include/config.example.h` jako `include/config.h` i uzupełnij.
2. Skopiuj `secrets.example.ini` jako `secrets.ini` i uzupełnij (adres sterownika, hasło do `/update`).
3. PlatformIO: `env:usb` (przez programator) albo `env:ota` (przez WiFi).

Pierwsze wgranie (zmiana tablicy partycji na `min_spiffs`) tylko przez USB: plik scalony
z bootloaderem, partycjami i programem pod adres `0x0`.

## Aktualizacja z GitHub

1. Zmień kod (np. w przeglądarce: plik → ikona ołówka → **Commit changes**).
2. GitHub Actions sam zbuduje program i opublikuje wydanie `v1.1.N` z plikiem `firmware.bin`
   (zakładka **Actions**, ok. 3–8 min; zielony znaczek = gotowe, czerwony = błąd kompilacji, wydania nie ma).
3. W SUPLA włącz „Restart”. Po starcie sterownik porównuje swoją wersję z najnowszym wydaniem
   i jeśli jest inna – pobiera `releases/latest/download/firmware.bin` i restartuje się.
   Wersja wgrana z komputera („lokalna”) nie jest nadpisywana automatycznie.
4. Nowa wersja jest zatwierdzana po 1 min pracy z WiFi. Jeśli wcześniej się zawiesi lub zrestartuje
   (albo przez 10 min nie połączy się z WiFi), sterownik wraca do poprzedniej wersji i pomija tę wersję
   przy kolejnych restartach.

Budowanie można też uruchomić ręcznie: **Actions → Budowanie programu → Run workflow**.
