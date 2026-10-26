#include "maze_controller.h"
#include <assert.h>
#include <stdio.h>
static maze_controller_t m; static maze_config_t c; static maze_sample_t s;
static uint32_t now;
static void step(unsigned bits, int ticks, bool pause) {
 for(int t=0;t<ticks;t++) {
  int raw[5]; for(int i=0;i<5;i++) raw[i]=(bits&(1<<i))?100:2000;
  maze_step(&m,&c,raw,now,pause,&s); now+=10;
 }
}
static void fresh(void){ c=maze_defaults();c.launch_ms=0;now=0;maze_init(&m,now);step(4,20,false); }
int main(void){
 fresh();step(31,20,false);assert(!m.finished);step(4,20,false);assert(!m.finish_pending);
 step(31,50,false);assert(!m.finished);assert(s.left<=25 && s.right<=25);
 step(31,15,false);assert(m.finished && s.state==MAZE_FINISHED && s.left==0 && s.right==0);
 step(4,50,false);step(0,50,true);step(4,50,false);assert(s.state==MAZE_FINISHED && s.left==0 && s.right==0);
 fresh();step(31,40,false);step(31,20,true);step(31,30,false);assert(!m.finished);
 step(31,40,false);assert(m.finished);
 fresh();step(27,50,false);assert(s.inverted);step(31,70,false);assert(m.finished);
 fresh();m.finish_confirm_ms=0;step(31,200,false);assert(!m.finished && s.left>0);
 fresh();now=UINT32_MAX-200;step(31,70,false);assert(m.finished);
 puts("PASS: brief crossings, continuous-white timer, approach speed, latched stop, pause reset, inversion, disable, clock wrap");
}
