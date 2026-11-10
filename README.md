# Sterownik pompy obiegowej – ESP32 + 10× DS18B20 + SUPLA

Program dla płytki „ESP32_Relay X2” (ESP32, 2 przekaźniki). Steruje dwiema pompami obiegowymi
na podstawie temperatur w pokojach, z obsługą przez aplikację SUPLA.

- czujniki 1–5 → termostat 1 → przekaźnik K1, czujniki 6–10 → termostat 2 → przekaźnik K2,
- temperatura grupy = średnia z czujników, które odpowiadają i nie są wyłączone w SUPLA,
- temperatura zadana i histereza osobno dla każdego termostatu (krok 0,1 °C),
- brak działającego czujnika w grupie → pompa stoi.

## Kanały SUPLA

| Nr | Kanał |
|---|---|
| 0–9 | termometry pokoi 1–10 |
| 10–19 | „wyłącz czujnik N” (ZAŁ = czujnik pomijany w średniej) |
| 20–21 | średnia temperatura grupy 1 / 2 |
| 22–23 | termostat K1 / K2 |
| 24 | „Aktualizuj z GitHub” |

## Strony WWW sterownika

- `http://<IP>` – konfiguracja WiFi i SUPLA,
- `http://<IP>:81` – podgląd czujników, ręczne sterowanie K1/K2, `/update` – wgrywanie pliku `.bin`.

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
3. W SUPLA włącz przełącznik „Aktualizuj z GitHub”. Sterownik pobiera `releases/latest/download/firmware.bin`
   i restartuje się. Wersja programu jest widoczna na `http://<IP>:81`.

Budowanie można też uruchomić ręcznie: **Actions → Budowanie programu → Run workflow**.
