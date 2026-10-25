#include "maze_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static maze_controller_t m;
static maze_config_t c;
static maze_sample_t s;
static uint32_t now;
static void step(unsigned bits,bool paused)
{
    int raw[5]; for(int i=0;i<5;i++) raw[i]=(bits&(1u<<i)) ? 0 : 4095;
    maze_step(&m,&c,raw,now,paused,&s); now+=10;
    assert(isfinite(s.error)&&isfinite(s.left)&&isfinite(s.right));
    assert(fabsf(s.left)<=60 && fabsf(s.right)<=60);
}
static void repeat(unsigned bits,int ticks)
{ for(int i=0;i<ticks;i++) step(bits,false); }
static void fresh(void)
{
    c=maze_defaults(); c.launch_ms=0; now=0;
    maze_init(&m,now); repeat(4,20);
    assert(s.state==MAZE_FOLLOW);
}
int main(void)
{
    c=maze_defaults(); assert(maze_config_valid(&c));
    maze_init(&m,0); now=0; repeat(0,30);
    assert(s.state==MAZE_LAUNCH && s.left>0 && s.right>0);
    step(0,false); assert(s.left==0&&s.right==0);
    repeat(0,20); assert(s.state==MAZE_SEARCH&&s.left<0&&s.right>0);
    repeat(0,3000); assert(s.state==MAZE_SEARCH&&s.left<0&&s.right>0);
    repeat(1,20); assert(s.state==MAZE_SEARCH); // Outer-only white cannot finish alignment.
    repeat(4,20); assert(s.state==MAZE_FOLLOW&&s.left>0&&s.right>0);

    fresh(); step(0,false); assert(s.state==MAZE_SETTLE&&s.left==0&&s.right==0);
    repeat(0,20); assert(s.state==MAZE_SEARCH);
    step(0,true); assert(s.state==MAZE_PAUSED&&s.left==0&&s.right==0);
    repeat(0,20); assert(s.state==MAZE_SEARCH); // Resume does not redo forward launch.

    fresh(); repeat(31,250); assert(s.state==MAZE_FOLLOW&&s.left==s.right&&s.left>0);
    assert(strstr(s.reason,"All white"));
    fresh(); repeat(1,20); assert(s.state==MAZE_SHARP_LEFT&&s.left<0&&s.right>0);
    fresh(); repeat(16,20); assert(s.state==MAZE_SHARP_RIGHT&&s.left>0&&s.right<0);

    fresh(); repeat(7,30); assert(s.state==MAZE_TURN_LEFT);
    repeat(0,10); assert(s.left<0&&s.right>0);
    repeat(4,15); assert(s.state==MAZE_FOLLOW&&s.left>0&&s.right>0);

    fresh(); repeat(28,6); assert(s.state==MAZE_INSPECT);
    repeat(4,20); assert(s.state==MAZE_FOLLOW); // Right + straight => straight.
    fresh(); repeat(28,6); assert(s.state==MAZE_INSPECT);
    repeat(16,15); assert(s.state==MAZE_TURN_RIGHT); // Only right remains.
    repeat(0,160); assert(s.state==MAZE_SEARCH); // Failed turn never waits forever.

    fresh(); repeat(27,30); assert(s.state==MAZE_TURN_LEFT); // Both branches => left.
    repeat(31,10); assert(s.state==MAZE_FOLLOW&&s.left==s.right&&s.left>0); // All-white overrides pivot.
    step(0,false); assert(s.left==0&&s.right==0); // Stale white filter cannot drive on black.
    fresh(); step(7,false); repeat(4,5); assert(s.state==MAZE_FOLLOW); // No single-read junction.

    fresh(); step(0,true); repeat(4,15); assert(s.state==MAZE_FOLLOW); // Resume centres.
    fresh(); repeat(2,10); float negative=s.error;
    fresh(); repeat(8,10); assert(fabsf(negative+s.error)<.01f); // Symmetric position.

    for(unsigned bits=0;bits<32;bits++) { fresh(); repeat(bits,200); }
    // Per-sensor calibration supports either electrical polarity, and rejects no contrast.
    c=maze_defaults(); c.white[0]=4095; c.black[0]=0; assert(maze_config_valid(&c));
    c.white[0]=c.black[0]; assert(!maze_config_valid(&c));
    c=maze_defaults(); c.kp=NAN; assert(!maze_config_valid(&c));
    puts("PASS: launch, continuous search, centre alignment, pause/resume, all-white, sharp turns,");
    puts("      left/straight/right priorities, turn timeout, noise, symmetry and all 32 LSA patterns");
    return 0;
}
