#ifndef MAZE_WEB_H
#define MAZE_WEB_H
#include "maze_controller.h"
void maze_shared_init(void);
void maze_get_settings(maze_config_t *config, bool *paused);
void maze_publish(const maze_sample_t *sample);
void maze_get_sample(maze_sample_t *sample);
void maze_web_start(void);
void maze_network_text(char ap[16], char sta[16]);
#endif
