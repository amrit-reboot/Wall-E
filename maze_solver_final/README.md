# Wall-E final maze solver

Standalone ESP-IDF project for Amrit's SRA ESP32 robot, adapted from
[athralk/Wall-E, 7_maze_solving](https://github.com/athralk/Wall-E/tree/c5f29b40eaa59e706c868d5624adda5b3a12d5bb/7_maze_solving).
Upstream commit: `c5f29b40eaa59e706c868d5624adda5b3a12d5bb`.
The original MIT notice is preserved in LICENSE. This is an attributed adaptation,
not a claim of independently authored navigation logic.

## Build and flash (ESP-IDF 5.5.x terminal)

```powershell
cd C:\GitHub\Wall-E
git fetch origin
git switch final
git pull --ff-only origin final
git submodule update --init --recursive
cd maze_solver_final
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p COM3 flash monitor
```

Use **Wall-E Configuration > Final maze solver** for cruise/pivot duty,
maximum following duty, sensor threshold, exit priority and optional front IR.
Use **WiFi Config** for your 2.4 GHz network credentials.
Use **SRA Board Component Config** for OLED settings.
The flash command includes the SPIFFS web image. Open the printed IP address
for PID tuning, or `/lsa.html` for live sensor readings.

## Navigation behavior

The upstream method is retained: analog PID line following, learned line-width
junction detection, axle approach, sensor-reacquired pivots, acute turns,
dead-end U-turns and widening lost-line sweeps. Startup waits for a line for
500 ms, then applies a short forward kick. It does not immediately drive on
an unrecognized surface. Wide white patches are confirmed before declaring DONE.
As upstream, DONE resumes after being placed on a normal line for one second.

Default priority is **straight, left, right**. Select exit rule **2** for
**left, straight, right**, or **1** for right-first. Straight-first node
backtracking is available but disabled by default, matching upstream.
This does not guarantee solving arbitrary mazes with loops.

Front object detection is disabled by default for the current LSA-only setup.
Enable it only with the active-low detector connected to GPIO27; it triggers
a U-turn only when all five LSA sensors also read white.
Inversion and pink drop-zone sources are included but remain inactive, as in
the referenced revision. Gyro source is also retained but not in the build.

## Dependencies

- ESP-IDF >=5.5,<6.0 (use your 5.5.5 installation).
- Existing `components/sra-board-component` submodule, pinned at
  `cb0a6a7066c110255e661351f350e3b0b68aa699`.
- Its nested LVGL submodule, installed by the recursive submodule command.
- Managed `espressif/mdns` 1.14.0, downloaded by ESP-IDF Component Manager.
- IDF's Wi-Fi, HTTP/WebSocket, SPIFFS, JSON, timer and driver components.

## Integration changes

Modern component registration and explicit dependencies; isolated project name;
menuconfig controls; network startup in a separate task; valid empty-IP OLED
handling before Wi-Fi exists; persistent Wi-Fi reconnect handlers; bounded
partial HTTP reads; numeric PID validation; NVS recovery; removal of obsolete
FATFS defaults. The upstream state-machine modules retain their names to make
future comparisons and maintenance straightforward.

## Validation and tuning

`sh tests/run.sh` compiles host tests for all three exit rules, LSA pattern
classification, line-width learning and node return orientation with compiler
warnings as errors and address/undefined-behavior sanitizers.
In restricted containers use `ASAN_OPTIONS=detect_leaks=0 sh tests/run.sh`.
These navigation tests passed during integration. ESP-IDF is not installed in
the editing environment: a full firmware build and hardware run were **not**
performed. Host tests do not validate ADC polarity, motor direction, networking,
OLED timing or track performance. Verify those on your board before a full run.

Initial cruise duty is 32, following maximum 36, pivot 50 (driver cap 50).
Threshold is 820 on the normalized scale where white is higher. Other timing
settings are in `main/include/config.h`. Tune approach duration and finish
confirmation on your actual junction and finish-box widths.
