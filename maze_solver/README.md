# Wall-E maze solver

Separate ESP-IDF **5.5.x**, classic **ESP32** firmware for the SRA normal-mode
two-wheel board. Build from this directory. No Arduino, LVGL, downloaded managed
components, SPIFFS image or SRA submodule is needed. The dashboard is embedded
in the firmware binary; a normal `flash` updates both firmware and webpage.

## Windows: build and flash

Open the ESP-IDF PowerShell terminal:

```powershell
cd C:\GitHub\Wall-E
git switch AMRIT
git pull --ff-only origin AMRIT
cd maze_solver
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

Use your actual serial port if different. Exit monitor with **Ctrl+]** before
flashing again. Subsequent builds only need `idf.py -p COM3 flash monitor`.
The custom partition table targets a board with at least **4 MB flash**.
Flashing this project replaces the running app; flash from `5_line_following`
to restore the old firmware. It does not change that project's source.

## Dashboard and initial setup

Join Wi-Fi **WallE-Maze**, password **wallemaze**, and open
**http://192.168.4.1**. Stay connected if Windows says the network has no Internet.
Control starts independently of Wi-Fi and the browser. Closing the page or
losing Wi-Fi does not stop the robot. The optional OLED shows state, sensor bits,
motor commands, AP IP and router IP, refreshed every half second.

The robot **moves automatically after boot**: the default first 300 ms is a
low-speed forward launch, even with no line. After that, no white immediately
cancels forward commands and starts a leftward search after a 30 ms stop.
Use a stand for the first motor-direction check. An explicit dashboard **Pause**
stops both motors and overrides autonomous searching until **Resume**.

For real track readings, calibrate before driving:

1. Press **Pause robot**, wait for **Paused**.
2. Open **Sensor details & calibration**. Hold all five sensors at their normal
   height over the actual black track material and capture black.
3. Put all five over white material at the same height and capture white.
4. Both captures are applied together only if every sensor has at least 100 raw
   ADC counts of contrast. Otherwise recapture. Either electrical polarity works.
5. Put the robot back on the line, then resume.

Calibration, threshold, cruise/pivot duty and PID settings save in NVS and
survive resets. NVS uses namespace `maze`, key `config_v1`. Capture buttons do
not move the robot. Raw readings continue while paused. Defaults assume white
produces lower raw voltage, matching the previous follower's inversion; they
are starting values, not measured calibration of your particular LSA.

## Behaviour

- White strength is normalized separately per sensor (0 black, 1000 white),
  filtered, then classified with hysteresis (default on 840, off below 800).
- Normal PID uses symmetric sensor weights, derivative filtering, limited
  integral, corner slowdown and 4 percentage-point command ramps per 10 ms tick.
- At a confirmed edge-only corner, the inside wheel reverses for a tight turn.
- Three/four-sensor wide patterns with an outer sensor confirm a junction over
  20 ms. The robot advances slowly for the configured sensor-to-axle time,
  collecting left/right branches, then chooses **left, straight, right, back**.
- All five white sensors override junction handling, pivots and searching:
  drive straight slowly. There is **no automatic finish-box stop**. This also
  means a junction presenting exactly five white sensors may be crossed straight;
  five sensors alone cannot distinguish that pattern from an all-white area.
- With no white, forward travel stops. Search rotates left continuously with
  opposite wheel commands until the centre detects a line for three consecutive
  readings; then PID resets and forward speed ramps up. An outer-only detection
  does not finish search. A completely white area also allows forward resumption.
- A junction pivot must leave the old centre line before accepting a new narrow
  centred line. After 1.4 s without alignment, it changes to continuous left search.
- Search does not time out into a permanently stopped state. Explicit Pause or
  an ADC/motor initialization/read/write fault are the exceptions. Hardware faults
  latch stopped and require checking the error and resetting the board.

This is local left-hand exploration, **not** a shortest-path mapper or a general
looped-maze solver. Fixed white-line polarity; automatic colour inversion is not
included. In-place search only sees lines within the sensor array's sweep and
cannot bridge an arbitrarily large disconnected gap. Without encoders/heading
feedback, pivots are aligned by LSA readings and are not exact commanded angles.
Equal opposite duty does not guarantee an exact stationary axis on real tyres.

## Minimal dashboard

Five live sensor bars, movement state/reason, signed motor duties, line error,
PID/threshold/speed settings, raw/reference readings, calibration and the last
eight state changes. Polls at approximately 5 Hz without queuing overlapping
requests. Marks disconnected/frozen/controller-stale readings and disables
controls until fresh data returns. Motor readings are **commands**, not measured
speed. The controller samples at a nominal 100 Hz; the webpage does not display
every control sample. Serial prints readings at 5 Hz and on state changes.

## Board mapping and menuconfig

| Function | ESP32 GPIOs, in logical order |
|---|---|
| LSA left to right | 32, 35, 34, 39, 36 (ADC1, averaged raw 12-bit samples) |
| Left motor input pair | 13, 12 |
| Right motor input pair | 26, 25 |
| SSD1306 OLED | SDA 21, SCL 22, address 0x3c |

The motor pin mapping and forward polarity match SRA's `pin_defs.h` and motor
driver. This project uses direct ESP-IDF LEDC 20 kHz PWM with the inactive bridge
input low, rather than importing the entire SRA component. It does not initialize
the bar graph or LED matrix, which share the motor pins. Check normal-mode board
configuration and wheel wiring. All GPIO numbers are ESP32 numbers.

Run `idf.py menuconfig` → **Maze solver** to reverse sensor order, swap motor
connectors, reverse either motor, change startup launch/junction advance, disable
the optional OLED or configure a router. A missing OLED logs a warning and does
not block control. The AP remains available when a router is configured, and
router reconnection handlers remain registered. AP channel may follow the router.

Tune junction advance on the actual sensor-to-axle distance; 120 ms is an initial
guess. Pivot duty must overcome motor friction. Start with cruise 40%, pivot 35%,
Kp 0.9, Ki 0 and Kd 6.5, then tune on track. Saved configuration takes precedence
over compiled defaults; `idf.py -p COM3 erase-flash` clears it if a full reset is
intended (and erases all other flash contents).

## Validation

Host controller tests (GCC/Clang on Linux):

```sh
cc -std=c11 -Wall -Wextra -Werror -I main/include \
  main/maze_controller.c tests/test_controller.c -lm -o /tmp/test_maze
/tmp/test_maze
```

Tests exercise launch, immediate forward stop on loss, sustained search, centre
alignment, pause/resume, all-white behaviour, sharp turns, junction priorities,
turn timeout, transient patterns, symmetric error, invalid calibration/settings
and all 32 binary sensor patterns. These do not substitute for tests on a robot.
See `VALIDATION.md` for the actual build and dashboard checks performed.
