#ifndef MAZE_OLED_H
#define MAZE_OLED_H
#include "maze_controller.h"
void maze_oled_init(void);
void maze_oled_update(const maze_sample_t *s, const char *ap, const char *sta);
#endif
