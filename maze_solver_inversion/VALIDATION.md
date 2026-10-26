# Validation of this inversion project

Passed host C11 compilation with -Wall -Wextra -Werror, AddressSanitizer and
UndefinedBehaviorSanitizer for test_controller.c and test_inversion.c.
Coverage includes all 32 patterns, startup, following, loss search, turns,
junctions, pause/resume, analog steering, map range extremes, inversion
entry/exit, transient rejection, steering sign in black mode, black junctions,
uniform surfaces and uint32_t clock wraparound.

Full ESP-IDF compilation and flashing have not been performed for this project.
No ESP32/robot is attached. Physical performance, calibration, radio and OLED
must be checked on the hardware. Host tests do not prove firmware build success.

Dashboard JavaScript syntax check passed. Browser tests could not run because
the Playwright Chromium executable is not installed in this environment.

Finish tests also pass: brief crossings, continuous detection, speed cap,
latched stop despite sensor changes/resume, pause timer reset, black-line mode,
detection disabled and timer wraparound. Full firmware build remains unverified.
