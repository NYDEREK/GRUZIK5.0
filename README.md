# GRUZIK 5.0

Firmware i aplikacja pomocnicza dla robota line follower opartego o
**ESP32-S3-MINI-1-N8**. Projekt zachowuje tekstowy protokół komend GRUZIK4.0,
ale łączy się z nową płytką przez **Bluetooth Low Energy** jako `gruzik 5.0`.

Stara aplikacja GRUZIK4.0 i wszystkie dotychczasowe roboty pozostają
nienaruszone. Katalog `android-app` buduje oddzielną aplikację BLE do GRUZIK5.

## Co potrafi firmware

- Prowadzenie po linii z tymi samymi parametrami PID i komendami co GRUZIK4.0.
- Obsługa 16 czujników przez 74HC4067 oraz odczyt napięcia akumulatora.
- Sterowanie dwoma sterownikami DRV, enkoderami i LED-em statusu.
- Automatyczne wykrywanie 0, 1 lub 2 układów ICM-42688-P.
- Fuzja yaw z dwóch IMU: dolny U10 jest odwrócony mechanicznie, dlatego jego
  oś yaw jest odwracana przed uśrednieniem.
- Telemetria BLE: odometria, żyroskop, IMU, bateria i wszystkie 16 kanałów
  czujnika linii.
- Mapowanie i odtwarzanie tras zapisanych w LittleFS (`/GRUZIK.txt`,
  `/map.txt`).

Robot **nie zatrzymuje się sam** ani po utracie linii, ani po rozłączeniu BLE.
Przy zatrzymanym robocie LED miga, a podczas aktywnej jazdy świeci ciągle.

## Struktura projektu

```text
firmware/                 PlatformIO / ESP32-S3
  src/main.cpp            logika robota, BLE i telemetria
  include/board_pins.h    przypisanie pinów PCB
  include/line_follower.h regulator line follower
  include/icm42688p_dual.h sterownik jednego lub dwóch ICM-42688-P

android-app/              aplikacja Android BLE
  apk/GRUZIK5-BLE-companion-debug.apk  gotowy APK
```

## Połączenia sprzętowe

| Funkcja | Piny ESP32-S3 |
| --- | --- |
| ICM U7 (górny) | MOSI 11, SCLK 12, MISO 13, CS 7, INT 6 |
| ICM U10 (dolny, opcjonalny) | wspólne SPI, CS 9, INT 8 |
| Multiplekser Sensorx16 | Z/ADC 10, S0 14, S1 15, S2 16, S3 17, E 18 |
| Dzielnik napięcia akumulatora | ADC 1 |
| Lewy silnik | PWM 41, DIR 42 |
| Prawy silnik | PWM 21, DIR 48 |
| DRV wspólne | nSLEEP 39, DRVOFF 40 |
| Enkoder lewy | A 37, B 36, I 38 |
| Enkoder prawy | A 33, B 34, I 35 |
| LED statusu | 47 |

Sensorx16 jest zasilany z 3,3 V. Kanały 74HC4067 są mapowane do logicznej
numeracji `SENSOR0..SENSOR15`, mimo że fizyczne banki multipleksera są
odwrócone na płytce czujników.

## Wgranie firmware

Wymagane jest PlatformIO Core oraz przewód USB z transmisją danych.

```bash
cd firmware
pio run
pio run -t upload --upload-port /dev/cu.usbmodemXXXX
pio device monitor -b 115200
```

Jeżeli port nie pojawia się automatycznie, przytrzymaj `BOOT`, naciśnij i puść
`RESET`, a następnie puść `BOOT`. Na macOS port zwykle ma nazwę
`/dev/cu.usbmodemXXXX`.

W logu startowym sprawdź:

```text
ICM-42688-P: mode=1 U7(status=1 who=0x47) U10(status=0 who=0xFF)
gruzik 5.0 ready (BLE only, no Wi-Fi)
```

`mode=1` oznacza jeden wykryty IMU, a `mode=2` dwa. `who=0xFF` oznacza brak
odpowiedzi z danego układu po SPI i wymaga kontroli zasilania, lutowania oraz
linii MISO/SCLK/MOSI/CS.

## Aplikacja Android

Można od razu zainstalować
[`GRUZIK5-BLE-companion-debug.apk`](android-app/apk/GRUZIK5-BLE-companion-debug.apk).
W Androidzie zaakceptuj uprawnienie **Urządzenia w pobliżu**, uruchom
odświeżanie, wybierz `gruzik 5.0 [BLE]` i naciśnij `Connect`.

Po podłączeniu aplikacja automatycznie włącza telemetrykę Debug, aby pokazać
napięcie, odometrię, IMU i surowe kanały czujników. Aplikacja przesyła te same
linie poleceń co GRUZIK4.0, na przykład:

```text
Kp=0.015
Kd=0.55
Base_speed=125
Max_speed=200
Treshold=3300
Mode=Y
Telemetry=debug
```

Do zbudowania APK ze źródeł:

```bash
cd android-app
./gradlew assembleDebug
```

## Ważne założenia

- ESP32-S3 w tej konfiguracji używa BLE, nie Bluetooth Classic SPP.
- Firmware nie uruchamia Wi-Fi ani nie łączy się z żadną siecią.
- Wejścia enkoderów są gotowe programowo, nawet jeśli enkodery nie są jeszcze
  zamontowane.
- Przed pierwszą jazdą z silnikami sprawdź kierunki kół i wartości PID na
  podniesionym robocie.
