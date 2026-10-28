#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Portable controller: input is calibrated brightness, 0=black .. 1000=white,
 * in physical left-to-right order. No ESP-IDF dependency or motor I/O. */
typedef enum { INV_WAIT, INV_FOLLOW, INV_CONFIRM, INV_BRIDGE,
               INV_SEARCH, INV_STOPPED } inv_state_t;
typedef struct {
    float cruise, creep, maximum, search_duty;
    float kp, kd, derivative_tau, slew_per_second;
    uint32_t confirm_ms, bridge_ms, stop_ms;
    int black_max, white_min;
} inv_config_t;
typedef struct {
    inv_state_t state;
    bool black_line, switched, valid_line;
    unsigned switches;
    uint8_t white_bits;
    float position, left, right;
} inv_output_t;
typedef struct {
    bool black_line, candidate_active, lost, have_previous;
    bool white[5], confident[5];
    uint32_t candidate_since, lost_since, last_ms;
    float last_position, derivative, left, right;
    unsigned switches;
    bool stopped;
} inv_controller_t;
inv_config_t inv_default_config(void);
void inv_init(inv_controller_t *c, uint32_t now);
void inv_step(inv_controller_t *c, const inv_config_t *cfg,
              const int brightness[5], uint32_t now, inv_output_t *out);
const char *inv_state_name(inv_state_t state);
