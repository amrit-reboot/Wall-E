#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t inversion_board_init(void);
esp_err_t inversion_board_read(int raw[5]);
esp_err_t inversion_board_drive(float left, float right);
void inversion_board_stop(void);
