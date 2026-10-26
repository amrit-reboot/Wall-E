# Maze solver with automatic black/white inversion

Complete, separate ESP-IDF project for the classic ESP32 SRA board. This folder
includes the smoother steering and 48% cruise baseline; the earlier patch and
original maze_solver folder are not required. No external components/submodules
are required. ESP-IDF 5.5.x, 4 MB flash, five LSA sensors, two drive motors.

## Install on Windows

Extract the ZIP into C:\GitHub\Wall-E. The result must be:
C:\GitHub\Wall-E\maze_solver_inversion\CMakeLists.txt

Open the ESP-IDF PowerShell terminal:

```powershell
cd C:\GitHub\Wall-E\maze_solver_inversion
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

Build must succeed before flashing. Change COM3 if necessary. Ctrl+] exits
monitor. No Git patch, commit or push is needed. To return to the previous
firmware, build and flash its original project directory.

## Behavior

- Starts with a brief forward launch, then follows white on black.
- White outer sensors plus black center for 60 ms selects black on white.
- Black outer sensors plus white center for 60 ms restores white on black.
- 300 ms hold after each switch prevents rapid repeated switches.
- Color evidence takes priority over junction detection while confirming;
  confirmation advances slowly at 30% and resets steering history on a switch.
- Left-first junction selection, sharp turns, stop/search on line loss and pause
  work on the active line color. Loss search rotates left until centered.
- All-white slows to 25% duty and stops after 600 ms continuously white.
  Brief crossings reset the timer when any sensor no longer sees white. Stop
  remains latched even if readings change or Pause/Resume is pressed; reset
  ESP32 to begin another run. Detection also works after black-line sections.
- Uniform black in black mode stops, confirms return to white mode, then searches.
- Straight cruise 48%, pivot 35%, maximum wheel duty 60%. Kp 0.65, Ki 0, Kd 1.8.
- No object detection, stored maze map or shortest-path planning.

## Dashboard and calibration

Join Wi-Fi `WallE-Maze`, password `wallemaze`, and open http://192.168.4.1.
The header and OLED show WHITE LINE or BLACK LINE. Sensor labels report the
physical surface; LINE identifies sensors currently used for tracking. The
0–1000 bars always measure white strength, even in black-line mode.

Defaults assume your black 500–700 and white 850–990 are normalized dashboard
values, not raw ADC. White turns on at 800 and off below 750. Black tracking uses
distance below the white boundary, so black does not need to reach an inverted
775 threshold. Pause and capture both surfaces if calibration is needed. All
five sensors should distinguish the surfaces. Settings use a separate NVS
namespace (`maze_inv`); settings from maze_solver are not automatically imported.

Use `idf.py menuconfig` > Maze solver for motor reversal, motor swapping,
sensor order, router credentials, startup and junction advance times.

## Track validation required

The code is host-tested, not physically tuned. Confirm wheel direction with
wheels raised, then test straight white line, black line block, return to white,
sharp corners and junctions. Automatic detection assumes a centered line at
color boundaries. A junction with white edges and black center can produce the
same evidence as inversion; five sensors cannot distinguish every geometry.
The 60 ms confirmation and 300 ms hold must fit block length and robot speed.
Continuous loss search has no timeout, so use the dashboard pause or power switch.

This project adapts the color-pattern approach reviewed in
https://github.com/athralk/Wall-E/tree/maze-solving/11_left_first
at commit 4bc8f3cb3ef311d5b4d1c6d0520ae23fcf669bba. Implementation is integrated
into this controller rather than copying the reference project's dependencies.
The included MIT LICENSE is retained.

## Final white box

In menuconfig > Maze solver > All-white finish confirmation, adjust the default
600 ms. Set 0 to disable. This is a duration heuristic: any sufficiently long
all-white area can trigger it. Measure on your actual track: confirmation must
exceed white-crossing duration but be shorter than time to leave the final box
at 25% duty. If the motors stall at 25%, detection can falsely trigger on a
crossing; test approach motion before relying on this setting. Startup launch
is excluded. The dashboard and OLED show Finished after stopping.
