#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "inversion_controller.h"

static inv_controller_t c;
static inv_config_t cfg;
static inv_output_t o;
static uint32_t now;
static void reset(void)
{ now=0; cfg=inv_default_config(); inv_init(&c,now); }
static void sample(const int b[5])
{
    now+=10; inv_step(&c,&cfg,b,now,&o);
    assert(isfinite(o.left) && isfinite(o.right));
    assert(fabsf(o.left)<=cfg.maximum && fabsf(o.right)<=cfg.maximum);
}
static void repeat(const int b[5], int count)
{ while(count-->0) sample(b); }
static const int white_line[5]={0,0,1000,0,0};
static const int black_line[5]={1000,1000,0,1000,1000};

int main(void)
{
    reset(); sample(white_line);
    assert(o.left==55 && o.right==55); /* Starts above stiction immediately. */
    repeat(white_line,30); assert(o.left==50 && o.right==50);
    /* Existing sdkconfig cruise=42 must not retain the stall-prone output. */
    reset(); cfg.cruise=42; cfg.creep=32;
    repeat(white_line,30); assert(o.left==45 && o.right==45);
    const int slightly_right[5]={0,0,900,700,0};
    repeat(slightly_right,30);
    assert(o.left>o.right && o.right>=45); /* Preserve small corrections. */
    uint32_t started=c.motion_since;
    repeat(black_line,3); assert(o.switched && c.motion_since==started);
    /* A waiting wrapper can suppress and rearm the startup pulse. */
    reset(); sample(white_line);
    c.motion_started=false; c.left=c.right=0;
    now+=1000; sample(white_line); assert(o.left==55 && o.right==55);

    reset(); repeat(white_line,30);
    assert(o.valid_line && !o.black_line && o.switches==0);
    assert(fabsf(o.position)<0.001f && o.left==o.right);

    /* Every patch can end immediately after confirmation: no lockout. */
    for(int patch=0;patch<100;patch++) {
        sample(black_line); assert(o.state==INV_CONFIRM && !o.black_line);
        sample(black_line); assert(!o.black_line);
        sample(black_line); assert(o.black_line && o.switched && o.valid_line);
        sample(white_line); assert(o.state==INV_CONFIRM && o.black_line);
        sample(white_line); assert(o.black_line);
        sample(white_line); assert(!o.black_line && o.switched && o.valid_line);
    }
    assert(o.switches==200);

    reset(); repeat(black_line,3);
    assert(o.black_line && o.valid_line); /* Can start inside a patch. */
    const int black_left[5]={1000,0,1000,1000,1000};
    repeat(black_left,30); assert(o.position<0 && o.left<o.right);
    const int black_right[5]={1000,1000,1000,0,1000};
    repeat(black_right,30); assert(o.position>0 && o.left>o.right);

    /* Off-centre and wider lines still carry polarity evidence. Analogue
     * readings need not be ideal zero/1000. */
    reset();
    const int offset_black[5]={920,80,110,900,950};
    repeat(offset_black,3);
    assert(o.black_line && o.position<0 && o.switches==1);
    const int wide_white[5]={50,900,950,910,80};
    repeat(wide_white,3);
    assert(!o.black_line && fabsf(o.position)<0.1f && o.switches==2);
    const int split_white[5]={0,1000,0,1000,0};
    repeat(split_white,5);
    assert(!o.valid_line && !o.black_line); /* No averaging separate branches. */

    reset(); repeat(white_line,10);
    for(int i=0;i<100;i++) { sample(black_line); sample(white_line); }
    assert(!o.black_line && o.switches==0); /* Transient noise rejected. */
    const int uncertain[5]={1000,1000,500,1000,1000};
    sample(black_line); sample(uncertain); sample(black_line);
    assert(!o.black_line); /* Mid-band sample breaks confirmation. */
    repeat(white_line,5);

    const int all_white[5]={1000,1000,1000,1000,1000};
    const int all_black[5]={0,0,0,0,0};
    repeat(all_white,5); assert(!o.black_line && o.switches==0);
    repeat(all_black,5); assert(!o.black_line && o.switches==0);
    repeat(white_line,5); assert(o.valid_line);
    const int edge_left[5]={1000,1000,0,0,0};
    repeat(edge_left,10); assert(!o.black_line && o.position<0);
    const int edge_right[5]={0,0,0,1000,1000};
    repeat(edge_right,10); assert(!o.black_line && o.position>0);
    const int skew[5]={1000,1000,0,0,0};
    repeat(skew,5); assert(!o.black_line);

    /* Uniform ground bridges briefly, searches, then latches stopped. */
    reset(); repeat(white_line,10);
    sample(all_black); assert(o.state==INV_BRIDGE);
    repeat(all_black,13); assert(o.state==INV_SEARCH);
    repeat(all_black,150); assert(o.state==INV_STOPPED && o.left==0 && o.right==0);
    repeat(white_line,10); assert(o.state==INV_STOPPED);
    inv_init(&c,now); repeat(white_line,5); assert(o.valid_line);

    /* Intermittent inversion evidence must not postpone the lost-line stop. */
    reset(); repeat(white_line,10);
    for(int i=0;i<100;i++) { sample(black_line); sample(all_white); }
    assert(o.state==INV_STOPPED);

    /* Same behaviour across uint32 millisecond wrap and variable periods. */
    reset(); now=UINT32_MAX-15; inv_init(&c,now);
    repeat(white_line,2); repeat(black_line,3); assert(o.black_line);
    reset();
    inv_step(&c,&cfg,black_line,100,&o); assert(!o.black_line);
    inv_step(&c,&cfg,black_line,125,&o); assert(o.black_line);

    /* Bounded pseudo-random noise and arbitrary spacing never produce NaNs
     * or out-of-limit output. */
    reset(); uint32_t rng=12345;
    for(int k=0;k<20000;k++) {
        int b[5];
        for(int i=0;i<5;i++) { rng=rng*1664525U+1013904223U; b[i]=(int)(rng%1001); }
        sample(b);
    }
    puts("PASS: polarity switching, short/repeated patches, noise, steering, loss recovery, stop latch, clock wrap, randomized bounds");
}
