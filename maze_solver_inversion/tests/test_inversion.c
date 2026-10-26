#include "maze_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static maze_controller_t m;
static maze_config_t c;
static maze_sample_t s;
static uint32_t now;
static void sample(unsigned physical, int ticks, bool paused)
{
    for(int t=0;t<ticks;t++) {
        int raw[5];
        for(int i=0;i<5;i++) raw[i]=(1000-((physical&(1<<i)) ? 900 : 650))*4095/1000;
        maze_step(&m,&c,raw,now,paused,&s); now+=10;
        assert(isfinite(s.left)&&isfinite(s.right));
        assert(fabsf(s.left)<=60&&fabsf(s.right)<=60);
    }
}
static void fresh(void)
{ c=maze_defaults(); c.launch_ms=0; now=0; maze_init(&m,now); m.finish_confirm_ms=0; sample(4,20,false); }
int main(void)
{
    fresh(); sample(27,3,false); sample(4,20,false); assert(!s.inverted); // transient rejected
    fresh(); sample(27,20,false);
    assert(s.inverted && s.bits==4 && s.white_bits==27 && s.state==MAZE_FOLLOW);
    assert(s.left>0 && fabsf(s.left-s.right)<.01f);
    sample(27,30,false); sample(4,20,false); // normal white line returns
    assert(!s.inverted && s.bits==4 && s.state==MAZE_FOLLOW);
    fresh(); sample(17,20,false); assert(s.inverted && s.bits==14); // wider black line
    sample(17,40,false); sample(29,20,false); // black under left-inner sensor
    assert(s.inverted && s.error<0 && s.left<s.right);
    fresh(); sample(27,20,false); sample(27,1,true);
    assert(s.state==MAZE_PAUSED && s.left==0 && s.right==0);
    sample(4,40,true); assert(s.inverted); // pause cannot switch modes
    sample(27,40,false); assert(s.inverted && s.state==MAZE_FOLLOW);
    sample(0,60,false); assert(!s.inverted && s.state==MAZE_SEARCH); // uniform black exits, searches
    fresh(); sample(27,20,false); sample(31,60,false);
    assert(!s.inverted && s.state==MAZE_FOLLOW && s.left==s.right && s.left>0);
    fresh(); sample(27,50,false); sample(24,30,false);
    assert(s.inverted && s.state==MAZE_TURN_LEFT); // black left junction (active bits 00111)
    // Confirmation survives uint32_t clock wraparound.
    c=maze_defaults(); c.launch_ms=0; now=UINT32_MAX-30; maze_init(&m,now); m.finish_confirm_ms=0;
    sample(27,20,false); assert(s.inverted && s.state==MAZE_FOLLOW);
    // Map extremes work without requiring black strength to exceed white threshold.
    for(int black=500;black<=700;black+=200) for(int white=850;white<=990;white+=140) {
        fresh();
        for(int t=0;t<60;t++) {
            int raw[5];for(int i=0;i<5;i++)raw[i]=(1000-(i==2?black:white))*4095/1000;
            maze_step(&m,&c,raw,now,false,&s);now+=10;
        }
        assert(s.inverted && s.bits==4 && s.left>0 && fabsf(s.left-s.right)<.01f);
    }
    puts("PASS: inversion entry/exit, debounce, both widths, steering sign, pause, black/white areas, black junction, clock wrap, map extremes");
}
