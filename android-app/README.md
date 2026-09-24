# GRUZIK5.0 Android

Aplikacja GRUZIK4.0 zachowująca dotychczasowy interfejs i komendy, ale
korzystająca z Bluetooth Low Energy wymaganego przez ESP32-S3.

## Połączenie

1. Włącz Bluetooth i nadaj aplikacji uprawnienie „Urządzenia w pobliżu”.
2. Włącz robota z firmware GRUZIK5.0.
3. Naciśnij `Refresh`.
4. Wybierz `gruzik 5.0` i naciśnij `Connect`.

Nie trzeba wcześniej parować robota w ustawieniach Androida.

## BLE

- Service: `8f1d0001-8a6b-4dd2-9b52-8ea36a45f001`
- RX commands: `8f1d0003-8a6b-4dd2-9b52-8ea36a45f001`
- TX notifications: `8f1d0004-8a6b-4dd2-9b52-8ea36a45f001`

Komendy i odpowiedzi tekstowe pozostają zgodne z GRUZIK4.0, w tym PID,
joystick, czyszczenie opon, telemetria, debug, map dump i map upload.

## Budowanie

```bash
./gradlew assembleDebug
```

Gotowy plik instalacyjny znajduje się w `apk/GRUZIK5.0-debug.apk`.
