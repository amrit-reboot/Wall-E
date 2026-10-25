# Validation — 25 October 2026

## Firmware

- Target: classic ESP32; 4 MB flash; default configuration including OLED.
- SDK: official ESP-IDF **v5.5.5**, commit `b774170f`.
- Compiler: Xtensa GCC **14.2.0**, Espressif `esp-14.2.0_20260121`.
- Full project compiled, linked and generated bootloader, partition table,
  `maze_solver.elf`, `maze_solver.bin` and flash argument files successfully.
- Final source rebuild: passed, no compiler warnings/errors.
- Final app image: **848,912 bytes** (`0xcf410`), fits the **3 MB** app partition
  with **73% free**. Bootloader and partition-size checks passed.
- Flash arguments generated for bootloader `0x1000`, partition table `0x8000`,
  app `0x10000`, DIO, 40 MHz, 4 MB.

The Linux build container cannot expose process information to the SDK's
component-manager Python helper. Validation therefore used
`IDF_COMPONENT_MANAGER=0`. This project has no managed/external dependencies;
all required drivers and libraries come from ESP-IDF itself. No SDK source was
patched. Normal Windows ESP-IDF build commands are in `README.md`.

## Controller tests

Passed GCC `-std=c11 -Wall -Wextra -Werror` with AddressSanitizer and
UndefinedBehaviorSanitizer. Leak scanning was disabled because the container
does not expose the required process information; the controller does not
allocate heap memory.

Covered automatic launch, first-sample forward stop on line loss, prolonged
continuous left search, rejection of edge-only search completion, centred
reacquisition, explicit pause/resume, all-white forward behaviour including
during a pivot, sharp left/right turns, left/straight/right priorities, pivot
timeout into search, transient junction patterns, symmetric steering error,
invalid calibration/settings and all 32 possible sensor bit patterns.

## Dashboard tests

Passed automated Chromium/Playwright checks against simulated firmware API
responses: live sensor rendering, editable PID fields preserved during polling,
settings submission, pause/resume, calibration gating and capture feedback,
stale/frozen/disconnected data detection and recovery, signed motor direction,
and desktop/mobile horizontal-overflow checks (1000 px and 390 px).
No browser JavaScript exceptions. Desktop and mobile screenshots visually
inspected. Dashboard JavaScript syntax check passed.

## Hardware boundary

No physical ESP32, LSA, motors or serial port was attached to the build system.
Actual upload, motor polarity, fixed-axis accuracy, OLED communication, sensor
calibration, Wi-Fi radio operation and track performance remain to be checked
on the robot. Generated flash files establish build readiness, not proof of a
successful physical flash. Initial speed, turn timing and PID values need
tuning for the actual chassis and surface.
