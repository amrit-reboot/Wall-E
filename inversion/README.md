# Standalone line following with automatic inversion

ESP-IDF project for the ESP32 SRA board. This folder contains only line following,
automatic white/black line selection, calibration and bounded line recovery.
It does not invoke or modify any maze solver. No Wi-Fi, OLED, front IR, junction
navigation, finish-zone detection or prescribed route is involved.

## Build and flash

Use an ESP-IDF **5.5.x** terminal (your installed 5.5.5 is suitable):

```powershell
cd C:\GitHub\Wall-E
git fetch origin
git switch inversion
cd inversion
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p COM3 flash monitor
```

The project uses ESP-IDF ADC and LEDC drivers directly, so it does not need the
SRA component submodule or downloaded managed components. Hardware pin mapping
is taken from this repository's `maze_solver/main/maze_board.c`:

| Function | GPIOs |
| --- | --- |
| Five LSA inputs, logical left to right | 32, 35, 34, 39, 36 |
| Motor A direction pair | 13, 12 |
| Motor B direction pair | 26, 25 |

Under **Inversion hardware and tuning**, reverse sensor order, reverse either
motor or swap the motors if your physical assembly needs it. The initial map
matches the existing board driver. No bar graph or LED matrix is initialized;
those can share motor GPIOs. PWM is 20 kHz and each wheel is capped at 60% duty.

## Calibrate and run

1. After reset the motors are stopped. In the serial monitor send `c` followed
   by Enter. During the next **10 seconds**, manually sweep **every sensor** over
   both black and white surfaces, at the height used for driving. Motors stay off.
   The full sweep, rather than a stationary reading, supplies each sensor's
   individual black and white endpoints.
2. Each sensor must span at least 350 raw ADC counts. A rejected calibration
   keeps the motors stopped; correct the height/wiring or sweep and send `c` again.
3. Place the sensor bar over the line, with both outer sensors on background.
   Send `g` then Enter. After **500 ms** of continuous usable line detection it
   starts. It can start on a white line or inside a black-line patch.
4. Send `x` then Enter to stop. Send `g` again to rearm after repositioning.
   Calibration is held in RAM and must be repeated after every reset.

The ADC polarity matches the existing SRA code: raw ADC decreases on white.
Calibration maps each channel to **0 = black, 1000 = white**. The console logs
calibrated readings, polarity, state, position and commanded duties every 200 ms.
Polarity changes produce an immediate separate log message.

## How the detector works

The controller examines both possible polarities rather than assuming a patch
length, patch count, location, direction of travel or timed course sequence.

* The line must be a single continuous run across the array. For **switching
  polarity**, both outer sensors must confidently see the same background colour,
  and an interior run must confidently see its opposite. The run may sit under
  any of the three inner sensors; it does not have to be centred exactly.
* A candidate must persist for 20 ms (three observations with a 10 ms loop).
  Any uncertain or inconsistent observation resets confirmation. During this
  period the robot creeps forward with limited previous steering instead of
  applying PID to the misleading complemented pattern.
* Confirmation switches which colour is the line. The position is recalculated
  immediately, derivative history is cleared, and following resumes. The same
  rule handles switching back. **There is no post-switch lockout.**
* Per-channel hysteresis uses <=350 for confident black and >=650 for confident
  white; readings in between preserve the previous classification but cannot
  confirm a switch. Position uses analogue strengths in the selected continuous
  run. Filtered PD steering, speed reduction on bends and motor slew limiting
  smooth the response.
* Uniform all-white, uniform all-black, split runs and low-contrast frames do not
  trigger a flip. The robot briefly bridges an ambiguous reading for 120 ms,
  then searches toward the last line position (right if no direction is known).
  After 1500 ms without a valid line it latches stopped until `g` rearms it.
  An intermittent inversion candidate cannot reset this timeout.

Example along the photographed section:

| Surface | Physical sensor colours | Selected line |
| --- | --- | --- |
| Before white rectangle | B B W B B | White |
| Inside white rectangle | W W B W W | Black |
| After white rectangle | B B W B B | White |

The photo is used only to identify the type of transition. Its dimensions and
number of rectangles are not encoded in the program.

## Tuning and physical limits

Defaults are 50% cruise, 45% creep, a 45% minimum running target, and a 55%
startup boost for 120 ms. Kp=14 and Kd=0.10 (derivative uses seconds).
These are initial tuning values, not measured speed or a guarantee that your
motors will move at every commanded duty. Adjust in menuconfig for battery,
motor friction and sensor height. The main loop is 10 ms. Other controller
parameters and documented units are in `inv_default_config()`.

If the log says FOLLOW with nonzero L/R but the wheels do not move, the log
reports commands, not measured wheel motion. The original 37-44% commands may
stall loaded motors. Startup now immediately applies the boost instead of
slowly ramping through the stall region. Same-direction targets are shifted
together to respect the minimum while preserving steering differences; both
remain capped at 60%. The minimum applies to targets: slew limiting can pass
through smaller values briefly during direction changes.

With the robot supported and both wheels clear of the surface, enable motor
battery power and send `t` + Enter. This commands both motors at 55% for only
500 ms, then stops automatically. `x` stops early. It does not require calibration
or line detection. Check whether both wheels turn and whether their forward
directions match. If neither turns, inspect motor supply, battery, normal-mode
jumper configuration and motor connections; a FOLLOW log cannot verify them.
If only one turns, check that side. Direction/swap options are in menuconfig.

For an existing build, old cruise/creep settings persist in `sdkconfig`. The
new minimum and startup options take their defaults without erasing your
configuration. You may set cruise=50 and creep=45 in menuconfig explicitly.

A patch must remain under the array long enough to supply the confirmation
samples, and its background must reach both outer sensors. Slow down or shorten
confirmation within the supported range if patches are missed. A line that
covers all five sensors supplies no polarity evidence. A five-sensor snapshot
can also make some junctions look identical to an opposite-colour line; this
project cannot resolve that ambiguity everywhere. Maze integration must gate
the detector using navigation state and additional evidence as appropriate.
Sharp corners, gaps and adverse glare likewise need physical validation.

There is deliberately no all-white finish stop: all-white can occur while
crossing a colour boundary. A sustained uniform region instead reaches the
bounded lost-line stop. Test the complete pictured straight section in both
directions, several times, and then curves before maze integration.

## Tests and integration boundary

On a Linux machine with a C compiler:

```sh
sh inversion/tests/run.sh
```

Tests compile the portable controller with strict warnings, AddressSanitizer and
UndefinedBehaviorSanitizer. They cover 100 short patches with immediate returns,
startup on black, left/right steering, transient and uncertain readings,
uniform fields, edge lines, skewed transitions, bounded search/stop, explicit
reset, timer wrap, irregular timestamps and 20,000 randomized sensor frames.
They run the production controller, without ESP-IDF or fake sensor thresholds.

`inversion_controller.c/.h` are the future maze integration boundary: pass five
calibrated brightness readings and a monotonic millisecond timestamp to
`inv_step()`, then consume its position, polarity and state. For a maze, decide
which module owns recovery and motor output, and suspend polarity decisions
where the navigation state makes these patterns ambiguous.

Validation: the original firmware built and flashed on the user's ESP-IDF
5.5.5 setup. Its supplied log reached FOLLOW and issued nonzero motor commands,
but the user reported no motion. Host controller tests, including startup boost,
minimum duty and retained steering, pass for this revision. ESP-IDF is not
installed in the authoring environment, so this revision needs an ESP32 rebuild
and physical validation. Run `idf.py build` before flashing.

Board adapter derived from this repository's MIT-licensed `maze_board.c` on
`final` at commit `7426ad5`; controller and standalone application are new.
