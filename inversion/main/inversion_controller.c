#include "inversion_controller.h"
#include <math.h>
#include <string.h>

static float clip(float v, float lo, float hi)
{ return fminf(fmaxf(v, lo), hi); }

inv_config_t inv_default_config(void)
{
    return (inv_config_t){.cruise=42, .creep=32, .maximum=60,
        .search_duty=38, .kp=14, .kd=0.10f, .derivative_tau=0.04f,
        .slew_per_second=250, .confirm_ms=20, .bridge_ms=120,
        .stop_ms=1500, .black_max=350, .white_min=650};
}
void inv_init(inv_controller_t *c, uint32_t now)
{
    memset(c, 0, sizeof(*c));
    c->last_ms=now;
}
const char *inv_state_name(inv_state_t state)
{
    switch(state) {
    case INV_WAIT: return "WAIT";
    case INV_FOLLOW: return "FOLLOW";
    case INV_CONFIRM: return "CONFIRM";
    case INV_BRIDGE: return "BRIDGE";
    case INV_SEARCH: return "SEARCH";
    case INV_STOPPED: return "STOPPED";
    }
    return "?";
}

/* One contiguous run in the middle, with the opposite colour at BOTH
 * endpoints. This supplies polarity evidence; an edge-only line cannot.
 * Require confident samples everywhere to reject skewed boundary crossings.
 */
static bool bounded_line(const inv_controller_t *c, bool black)
{
    bool line[5];
    for(int i=0;i<5;i++) {
        if(!c->confident[i]) return false;
        line[i]=black ? !c->white[i] : c->white[i];
    }
    if(line[0] || line[4]) return false;
    int runs=0;
    for(int i=1;i<=3;i++) if(line[i] && !line[i-1]) runs++;
    return runs==1;
}

/* Reject split runs and uniform fields. Analogue strength is referenced to
 * this sample's background, reducing residual calibration mismatch. */
static bool position(const inv_controller_t *c, const int b[5], float *p)
{
    int runs=0, count=0;
    bool previous=false;
    float peak=0, low=1000, strengths[5];
    for(int i=0;i<5;i++) {
        bool on=c->black_line ? !c->white[i] : c->white[i];
        if(on && !previous) runs++;
        previous=on;
        count+=on;
        strengths[i]=c->black_line ? 1000-b[i] : b[i];
        peak=fmaxf(peak,strengths[i]); low=fminf(low,strengths[i]);
    }
    if(runs!=1 || count==5 || peak-low<250) return false;
    float total=0, weighted=0;
    for(int i=0;i<5;i++) {
        bool on=c->black_line ? !c->white[i] : c->white[i];
        float s=on ? fmaxf(0,strengths[i]-low-0.2f*(peak-low)) : 0;
        total+=s; weighted+=(i-2)*s;
    }
    if(total<=0) return false;
    *p=weighted/total;
    return true;
}

static void drive(inv_controller_t *c, const inv_config_t *cfg,
                  float left, float right, float dt, inv_output_t *o)
{
    float step=cfg->slew_per_second*dt;
    c->left+=clip(left-c->left,-step,step);
    c->right+=clip(right-c->right,-step,step);
    c->left=clip(c->left,-cfg->maximum,cfg->maximum);
    c->right=clip(c->right,-cfg->maximum,cfg->maximum);
    o->left=c->left; o->right=c->right;
}

void inv_step(inv_controller_t *c, const inv_config_t *cfg,
              const int b[5], uint32_t now, inv_output_t *o)
{
    memset(o,0,sizeof(*o));
    uint32_t elapsed=now-c->last_ms;
    float dt=clip(elapsed*0.001f,0.001f,0.05f);
    c->last_ms=now;
    for(int i=0;i<5;i++) {
        if(b[i]>=cfg->white_min) c->white[i]=true;
        else if(b[i]<=cfg->black_max) c->white[i]=false;
        c->confident[i]=b[i]>=cfg->white_min || b[i]<=cfg->black_max;
        if(c->white[i]) o->white_bits|=(uint8_t)(1U<<i);
    }
    o->black_line=c->black_line; o->switches=c->switches;
    if(c->stopped) { o->state=INV_STOPPED; return; }

    float p=0;
    bool opposite=bounded_line(c,!c->black_line);
    if(opposite) {
        if(!c->candidate_active) {
            c->candidate_active=true; c->candidate_since=now;
        }
        if(now-c->candidate_since>=cfg->confirm_ms) {
            c->black_line=!c->black_line;
            c->switches++;
            c->candidate_active=false;
            c->have_previous=false; c->derivative=0;
            o->switched=true;
        }
    } else c->candidate_active=false;

    bool valid=!c->candidate_active && position(c,b,&p);
    o->valid_line=valid;
    o->black_line=c->black_line; o->switches=c->switches;
    if(valid) {
        c->lost=false;
        float d=c->have_previous ? (p-c->last_position)/dt : 0;
        c->derivative+=(dt/(cfg->derivative_tau+dt))*(d-c->derivative);
        c->last_position=p; c->have_previous=true;
        float correction=cfg->kp*p+cfg->kd*c->derivative;
        float base=cfg->cruise-fminf(10,fabsf(p)*5);
        o->position=p; o->state=INV_FOLLOW;
        drive(c,cfg,clip(base+correction,0,cfg->maximum),
                    clip(base-correction,0,cfg->maximum),dt,o);
        return;
    }

    /* A candidate never resets the lost timer: intermittent noise cannot
     * keep the robot moving forever without a confirmed usable line. */
    if(!c->lost) { c->lost=true; c->lost_since=now; }
    uint32_t lost_ms=now-c->lost_since;
    o->position=c->last_position;
    c->have_previous=false; c->derivative=0;
    if(lost_ms>=cfg->stop_ms) {
        c->stopped=true; c->left=c->right=0; o->state=INV_STOPPED;
        return;
    }
    if(c->candidate_active || lost_ms<cfg->bridge_ms) {
        o->state=c->candidate_active ? INV_CONFIRM : INV_BRIDGE;
        /* Do not steer using the misleading complemented sensor centroid.
         * Retain only a small, bounded amount of the last known curvature. */
        float correction=clip(cfg->kp*c->last_position,-8,8);
        drive(c,cfg,cfg->creep+correction,cfg->creep-correction,dt,o);
    } else {
        o->state=INV_SEARCH;
        float direction=c->last_position<0 ? -1 : 1;
        drive(c,cfg,direction*cfg->search_duty,-direction*cfg->search_duty,dt,o);
    }
}
