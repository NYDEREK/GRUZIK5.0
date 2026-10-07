# GRUZIK5.0 firmware

ESP-IDF v6.1 firmware for the GRUZIK5.0 line follower (ESP32-S3-MINI-1-N8).

## Build and flash

```bash
source ~/.espressif/tools/activate_idf_v6.1.sh
cd firmware
idf.py build flash monitor
```

Robot-specific options (Wi-Fi SSID/password, TCP port, gyro on/off) are in
`idf.py menuconfig` → **GRUZIK5**.

Connect the battery before the robot boots: the motor drivers are woken up
once at start-up and only accept the wake-up acknowledge while their motor
supply is present.

## Modules (`main/`)

| File | Responsibility |
| --- | --- |
| `board.h` | Pin map from the schematic |
| `config.h` | Physical constants and loop timing |
| `analog.c` | ADC1 one-shot driver, calibrated battery channel |
| `line_sensors.c` | 74HC4067 scan of the 16 line sensors |
| `battery.c` | Battery voltage and PWM speed level |
| `motors.c` | DRV8245H wake-up handshake, LEDC PWM, direction |
| `encoders.c` | Quadrature decoding in the PCNT peripheral |
| `imu.c` | ICM-42688-P yaw rate over SPI |
| `odometry.c` | Pose from encoders and gyro |
| `line_follower.c` | PD line controller, gap bridging, edge search |
| `route_map.c` | Route recording, playback, map file transfer |
| `storage.c` | FAT filesystem on flash (`/fs`) |
| `link.c` | Wi-Fi access point, TCP server, transmit queue |
| `commands.c` | Text command interpreter |
| `robot.c` | Robot modes and the 1 kHz control task |
| `telemetry.c` | ODOM / DBG stream for the app |
| `console.c` | USB commands and status report |

## Tasks

| Task | Core | Priority | Period / trigger |
| --- | --- | --- | --- |
| `control` | 1 | highest application priority | every 1 ms |
| `link_rx` | 0 | 5 | incoming TCP data |
| `link_tx` | 0 | 5 | queued outgoing text |
| `telemetry` | 0 | 4 | every 100 ms |
| `map_writer` | 0 | 3 | recorded route points |
| `console` | 0 | 2 | USB input, status every 2 s |

## Why FreeRTOS tasks

ESP-IDF itself runs on FreeRTOS (Wi-Fi, TCP/IP and the drivers are FreeRTOS
tasks), so the question is how the application uses it. A single super-loop
would mix work with very different timing needs:

* **Deterministic control timing.** The control loop must run every 1 ms no
  matter what else happens. As a high-priority task woken by
  `xTaskDelayUntil`, it preempts everything else and runs on a fixed grid
  instead of whenever a loop iteration gets around to it.
* **A core of its own.** Wi-Fi and the TCP/IP stack run on core 0. Pinning the
  control task to core 1 keeps radio bursts away from the motors and sensors.
* **Slow work cannot stall the motors.** A flash write can take several
  milliseconds and a socket send can block for much longer. These live in
  their own low-priority tasks (`map_writer`, `link_tx`); the control loop
  only drops data into a queue or stream buffer and returns immediately.
* **Blocking code stays simple.** Accepting a client, waiting for bytes or
  reading the USB console are written as straightforward blocking calls in
  their own tasks instead of polled state machines.
* **Safe shared state.** One mutex guards the robot state, so a command from
  the app never changes a parameter halfway through a control tick.
