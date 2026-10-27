#include <assert.h>
#include <stdio.h>
#include "junction.h"
#include "node_memory.h"
static lsa_state_t sample(unsigned mask) {
    lsa_state_t s = {0};
    for (int i = 0; i < LSA_COUNT; ++i) s.on_line[i] = !!(mask & LSA_BIT(i));
    return s;
}
int main(void) {
    for (int bits = 0; bits < 8; ++bits) {
        bool l = bits & 4, s = bits & 2, r = bits & 1;
#if EXIT_RULE == RULE_LEFT_HAND
        exit_t want = l ? EXIT_LEFT : s ? EXIT_STRAIGHT : r ? EXIT_RIGHT : EXIT_NONE;
#elif EXIT_RULE == RULE_RIGHT_HAND
        exit_t want = r ? EXIT_RIGHT : s ? EXIT_STRAIGHT : l ? EXIT_LEFT : EXIT_NONE;
#else
        exit_t want = s ? EXIT_STRAIGHT : l ? EXIT_LEFT : r ? EXIT_RIGHT : EXIT_NONE;
#endif
        assert(choose_exit(l,s,r,false,'A') == want);
    }
    lsa_state_t s = sample(31);
    assert(detect_branches(&s,3,false) == (BRANCH_LEFT | BRANCH_RIGHT));
    s = sample(14); assert(detect_branches(&s,3,false) == 0);
    s = sample(28); assert(detect_branches(&s,3,false) == 0);
    s = sample(22); assert(detect_side_line(&s) == BRANCH_LEFT);
    s = sample(16); assert(detect_sharp(&s) == -1);
    s = sample(1); assert(detect_sharp(&s) == 1);
    s = sample(0); assert(detect_branches(&s,3,false) == 0);
    line_width_t w = {0}; s = sample(4);
    for (int i=0;i<LINE_WIDTH_MIN_SAMPLES;i++) line_width_update(&w,&s);
    assert(line_width_typical(&w)==1);
    bool returning;
    node_memory_reset();
    node_memory_push(true,true,false,EXIT_STRAIGHT);
    assert(node_memory_return(false,true,true,&returning)==EXIT_RIGHT);
    assert(!returning);
    assert(node_memory_return(true,false,true,&returning)==EXIT_RIGHT);
    assert(returning);
    assert(node_memory_return(true,true,true,&returning)==EXIT_NONE);
    puts("navigation tests passed");
}
