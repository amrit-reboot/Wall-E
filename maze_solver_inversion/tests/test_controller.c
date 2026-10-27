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
    maze_init(&m,now); m.finish_confirm_ms=0; repeat(4,20);
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

    fresh(); repeat(27,30); assert(s.inverted && s.state==MAZE_FOLLOW); // Black center on white => inversion.
    repeat(31,40); assert(s.state==MAZE_FOLLOW&&s.left==s.right&&s.left>0); // All-white overrides pivot.
    step(0,false); assert(s.left==0&&s.right==0); // Stale white filter cannot drive on black.
    fresh(); step(7,false); repeat(4,5); assert(s.state==MAZE_FOLLOW); // No single-read junction.

    fresh(); step(0,true); repeat(4,15); assert(s.state==MAZE_FOLLOW); // Resume centres.
    fresh(); repeat(2,10); float negative=s.error;
    fresh(); repeat(8,10); assert(fabsf(negative+s.error)<.01f); // Symmetric position.

    // Reported map range is the dashboard's normalized 0..1000 scale.
    // Exercise every black/white extreme through the actual ADC conversion.
    for(int black=500;black<=700;black+=200) {
        for(int white=850;white<=990;white+=140) {
            fresh();
            for(int t=0;t<100;t++) {
                int raw[5];
                for(int i=0;i<5;i++) raw[i]=(1000-(i==2 ? white : black))*4095/1000;
                maze_step(&m,&c,raw,now,false,&s); now+=10;
                assert(s.state==MAZE_FOLLOW && s.bits==4);
                assert(fabsf(s.left-s.right)<.01f);
            }
            assert(fabsf(s.left-c.cruise)<.01f);
        }
    }
    // Small brightness noise must not cause stop/start or alternating hard turns.
    fresh();
    for(int t=0;t<200;t++) {
        int values[5]={650,710+(t%7),900+(t%11),710+(t%5),650};
        int raw[5];
        for(int i=0;i<5;i++) raw[i]=(1000-values[i])*4095/1000;
        maze_step(&m,&c,raw,now,false,&s); now+=t%2 ? 10 : 20;
        assert(s.state==MAZE_FOLLOW && s.left>40 && s.right>40);
        assert(fabsf(s.left-s.right)<5);
    }
    // Subthreshold partial coverage influences position continuously.
    fresh();
    int partial[5]={2047,1024,410,1228,2047};
    for(int t=0;t<20;t++) { maze_step(&m,&c,partial,now,false,&s); now+=10; }
    assert(s.bits==4 && s.error< -1 && s.error> -15);
    assert(s.left<s.right);
    // Timing-aware acceleration: 20 ms permits twice the 10 ms ramp.
    c=maze_defaults(); maze_init(&m,0);
    int centre_raw[5]={4095,4095,0,4095,4095};
    maze_step(&m,&c,centre_raw,0,false,&s); float initial=s.left;
    maze_step(&m,&c,centre_raw,20,false,&s); assert(fabsf(s.left-initial-12)<.01f);

    // At maximum cruise, an offset line must still steer (old headroom was zero).
    fresh(); c.cruise=60;
    repeat(4,20); repeat(8,30);
    assert(s.left-s.right>20 && s.left<=60 && s.right>=0);
    float high_left=s.left, high_right=s.right;
    fresh(); c.cruise=60; repeat(4,20); repeat(2,30);
    assert(fabsf(s.left-high_right)<.01f && fabsf(s.right-high_left)<.01f);
    // A sustained branch chooses left within 130 ms; single-read noise is covered above.
    fresh(); repeat(7,13); assert(s.state==MAZE_TURN_LEFT);
    // Line loss still cuts forward drive immediately, then starts pivoting within 30 ms.
    fresh(); step(0,false); assert(s.left==0 && s.right==0);
    repeat(0,3); assert(s.state==MAZE_SEARCH && s.left<0 && s.right>0);

    for(unsigned bits=0;bits<32;bits++) { fresh(); repeat(bits,200); }
    // Per-sensor calibration supports either electrical polarity, and rejects no contrast.
    c=maze_defaults(); c.white[0]=4095; c.black[0]=0; assert(maze_config_valid(&c));
    c.white[0]=c.black[0]; assert(!maze_config_valid(&c));
    c=maze_defaults(); c.kp=NAN; assert(!maze_config_valid(&c));
    puts("PASS: launch, continuous search, centre alignment, pause/resume, all-white, sharp turns,");
    puts("      left/straight/right priorities, turn timeout, noise, symmetry and all 32 LSA patterns");
    return 0;
}

