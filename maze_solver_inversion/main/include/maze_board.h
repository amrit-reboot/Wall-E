#ifndef MAZE_BOARD_H
#define MAZE_BOARD_H
#include "esp_err.h"
esp_err_t maze_board_init(void);
esp_err_t maze_board_read(int raw[5]);
esp_err_t maze_board_drive(float left, float right);
void maze_board_stop(void);
#endif
