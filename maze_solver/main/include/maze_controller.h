#ifndef MAZE_CONTROLLER_H
#define MAZE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>

#define MAZE_SENSORS 5
typedef enum {
    MAZE_LAUNCH, MAZE_FOLLOW, MAZE_SHARP_LEFT, MAZE_SHARP_RIGHT,
    MAZE_INSPECT, MAZE_TURN_LEFT, MAZE_TURN_RIGHT, MAZE_SEARCH,
    MAZE_SETTLE, MAZE_PAUSED, MAZE_FAULT
} maze_state_t;
typedef struct {
    float kp, ki, kd;
    int black[5], white[5];
    int threshold, hysteresis;
    int launch_ms, cross_ms;
    float cruise, turn;
} maze_config_t;
typedef struct {
    maze_state_t state;
    uint32_t since, lost_since, wide_since, clear_since;
    bool initialized, armed, left_seen, right_seen, left_old_line;
    bool white_bits[5];
    unsigned hits;
    float filtered[5], error, previous, integral, derivative, left, right;
    uint32_t sequence;
    const char *reason;
} maze_controller_t;
typedef struct {
    uint32_t ms, sequence;
    int raw[5], values[5], bits, count;
    float error, left, right;
    maze_state_t state;
    char reason[96];
    bool ready;
} maze_sample_t;
maze_config_t maze_defaults(void);
bool maze_config_valid(const maze_config_t *c);
void maze_init(maze_controller_t *m, uint32_t now);
void maze_step(maze_controller_t *m, const maze_config_t *c, const int raw[5],
               uint32_t now, bool paused, maze_sample_t *out);
const char *maze_state_name(maze_state_t state);
#endif
