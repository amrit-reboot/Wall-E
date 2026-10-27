#include "maze_controller.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static void pid_reset(maze_controller_t *m)
{ m->previous = m->integral = m->derivative = 0; m->pid_ready=false; }
static void enter(maze_controller_t *m, maze_state_t s, uint32_t now, const char *reason)
{
    m->state = s; m->since = now; m->reason = reason; m->sequence++;
    m->hits = 0; m->left_old_line = false;
    pid_reset(m);
}
const char *maze_state_name(maze_state_t s)
{
    static const char *names[] = {"Starting", "Following", "Sharp left", "Sharp right",
        "Inspecting junction", "Turning left", "Turning right", "Searching left",
        "Stopping forward", "Paused", "Sensor fault", "Finished"};
    return (unsigned)s < sizeof(names)/sizeof(names[0]) ? names[s] : "Unknown";
}
maze_config_t maze_defaults(void)
{
    return (maze_config_t){.kp=.65f, .ki=0, .kd=1.8f,
        .black={4095,4095,4095,4095,4095}, .white={0,0,0,0,0},
        .threshold=775, .hysteresis=25, .launch_ms=300, .cross_ms=80,
        .cruise=54, .turn=42};
}
bool maze_config_valid(const maze_config_t *c)
{
    if (!isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd) ||
        c->kp < 0 || c->kp > 100 || c->ki < 0 || c->ki > 100 || c->kd < 0 || c->kd > 100 ||
        c->threshold < 100 || c->threshold > 950 || c->hysteresis < 0 || c->hysteresis > 50 ||
        c->launch_ms < 0 || c->launch_ms > 1000 || c->cross_ms < 20 || c->cross_ms > 400 ||
        !isfinite(c->cruise) || c->cruise < 25 || c->cruise > 60 ||
        !isfinite(c->turn) || c->turn < 25 || c->turn > 60) return false;
    for (int i=0; i<5; ++i)
        if (c->black[i]<0 || c->black[i]>4095 || c->white[i]<0 || c->white[i]>4095 ||
            abs(c->white[i]-c->black[i])<100) return false;
    return true;
}
void maze_init(maze_controller_t *m, uint32_t now)
{
    memset(m, 0, sizeof(*m)); m->armed=true; m->finish_confirm_ms=600;
    enter(m, MAZE_LAUNCH, now, "Initial forward launch");
}
static float slew(float old, float target, float loops)
{
    // Pass through zero before reversing; accelerate 6 and decelerate 10 duty points per 10 ms.
    if (old*target < 0) target=0;
    float rate=fabsf(target)<fabsf(old) ? 10.0f : 6.0f;
    return old + clamp(target-old, -rate*loops, rate*loops);
}
void maze_step(maze_controller_t *m, const maze_config_t *c, const int raw[5],
               uint32_t now, bool paused, maze_sample_t *out)
{
    memset(out,0,sizeof(*out)); out->ms=now; out->ready=true;
    uint32_t elapsed=m->initialized ? now-m->last_step : 10;
    float loops=clamp((float)elapsed/10.0f,.1f,5.0f);
    m->last_step=now;
    float sensor_alpha=1.0f-powf(.4f,loops);
    if(m->finished) {
        m->left=m->right=0;
        out->state=MAZE_FINISHED; out->sequence=m->sequence;
        out->inverted=m->inverted;
        for(int i=0;i<5;i++) {
            out->raw[i]=raw[i];
            out->values[i]=(int)clamp(1000.0f*(raw[i]-c->black[i])/(c->white[i]-c->black[i]),0,1000);
            bool white=out->values[i]>=c->threshold;
            if(white) out->white_bits|=1<<i;
            if(m->inverted ? !white : white) { out->bits|=1<<i; out->count++; }
        }
        snprintf(out->reason,sizeof(out->reason),"Finish confirmed: reset ESP32 to restart");
        return;
    }
    int count=0, instant=0, physical=0;
    float strength[5], sum=0, weighted=0;
    static const float weights[5]={-50,-25,0,25,50};
    for (int i=0;i<5;i++) {
        strength[i]=clamp(1000.0f*(raw[i]-c->black[i])/(c->white[i]-c->black[i]),0,1000);
        m->filtered[i]=m->initialized ? m->filtered[i]+sensor_alpha*(strength[i]-m->filtered[i]) : strength[i];
        float boundary=c->threshold+(m->surface_white[i] ? -c->hysteresis : c->hysteresis);
        m->surface_white[i]=m->filtered[i]>=boundary;
        if(m->surface_white[i]) physical|=1<<i;
        out->raw[i]=raw[i]; out->values[i]=(int)m->filtered[i];
    }
    m->initialized=true;
    // Use unfiltered, well-separated evidence for switching; never infer from gray.
    bool outer_white=strength[0]>=c->threshold+c->hysteresis && strength[4]>=c->threshold+c->hysteresis;
    bool outer_black=strength[0]<c->threshold-c->hysteresis && strength[4]<c->threshold-c->hysteresis;
    bool all_white=true, all_black=true;
    for(int i=0;i<5;i++) {
        all_white &= strength[i]>=c->threshold+c->hysteresis;
        all_black &= strength[i]<c->threshold-c->hysteresis;
    }
    bool black_line=outer_white && strength[2]<c->threshold-c->hysteresis;
    bool white_line=outer_black && strength[2]>=c->threshold+c->hysteresis;
    bool eligible=!paused && (m->state==MAZE_FOLLOW || m->state==MAZE_LAUNCH || m->state==MAZE_SETTLE || (m->inverted && (all_white||all_black)));
    bool held=m->color_locked && now-m->color_changed<300;
    bool color_evidence=eligible && !held && (m->inverted ? (white_line||all_black||all_white) : black_line);
    m->color_pending=color_evidence;
    if(color_evidence) {
        if(!m->color_candidate) { m->color_candidate=true; m->color_since=now; }
        if(now-m->color_since>=60) {
            m->inverted=!m->inverted; m->color_changed=now; m->color_locked=true;
            m->color_candidate=m->color_pending=false;
            m->armed=false; m->clear_since=now; m->wide_since=0;
            m->left_seen=m->right_seen=false;
            enter(m,MAZE_FOLLOW,now,m->inverted ? "Black line mode" : "White line mode");
        }
    } else m->color_candidate=false;
    for(int i=0;i<5;i++) {
        m->white_bits[i]=m->inverted ? !m->surface_white[i] : m->surface_white[i];
        if(m->inverted ? strength[i]<c->threshold+c->hysteresis : strength[i]>=c->threshold-c->hysteresis) instant++;
        if(m->white_bits[i]) { count++; out->bits|=1<<i; }
        // Black readings on this map reach 500..700, so 1000-value with the
        // white threshold would miss them. Use distance below the white boundary.
        float signal=m->inverted ? fmaxf(0,c->threshold+c->hysteresis-m->filtered[i]) :
            fmaxf(0,m->filtered[i]-(c->threshold-3*c->hysteresis));
        sum+=signal; weighted+=weights[i]*signal;
    }
    out->white_bits=physical; out->inverted=m->inverted; out->count=count;
    if(sum>0) m->error=weighted/sum;
    bool centre=m->white_bits[2], left=m->white_bits[0], right=m->white_bits[4];
    bool wide=(left||right)&&count>=3;
    bool normal=centre&&count>0&&count<=3;
    float l=0,r=0;

    if (paused) {
        m->finish_pending=false;
        if(m->state!=MAZE_PAUSED) enter(m,MAZE_PAUSED,now,"Paused from dashboard");
        m->left=m->right=0;
        goto done;
    }
    if(m->state==MAZE_PAUSED) {
        m->armed=false; m->clear_since=now;
        enter(m, instant ? MAZE_FOLLOW : MAZE_SETTLE, now,"Resumed from dashboard");
    }
    // Physical all-white evidence is independent of the current line polarity.
    // A short crossing clears the timer; stopping is latched until reset.
    if(m->finish_confirm_ms && m->state!=MAZE_LAUNCH && all_white && physical==31) {
        if(!m->finish_pending) { m->finish_pending=true; m->finish_since=now; }
        if(now-m->finish_since>=m->finish_confirm_ms) {
            m->finished=true; m->left=m->right=0;
            enter(m,MAZE_FINISHED,now,"Finish confirmed: reset ESP32 to restart");
            goto done;
        }
        pid_reset(m); m->armed=false; m->clear_since=now; m->wide_since=0;
        m->reason="Checking white box: moving slowly";
        // Bound approach speed immediately, including arrival from a pivot.
        m->left=clamp(m->left,0,25); m->right=clamp(m->right,0,25);
        l=r=25; goto drive;
    } else m->finish_pending=false;
    // Slow straight confirmation prevents a color boundary being taken as a junction.
    // An all-black exit is stopped while confirming, rather than driven blindly.
    if(m->color_pending) {
        pid_reset(m);
        if(all_black) { m->left=m->right=0; goto done; }
        l=r=30; goto drive;
    }
    if(m->inverted && all_white) {
        pid_reset(m); l=r=30; goto drive;
    }
    // In black-line mode a uniform black patch is not reliable line evidence.
    if(m->inverted && all_black) {
        m->left=m->right=0;
        if(m->state!=MAZE_SETTLE) enter(m,MAZE_SETTLE,now,"Uniform black: confirm white-line recovery");
        goto done;
    }
    // No forward coasting once every sensor reports no white. Startup is the only exception.
    if(m->state!=MAZE_LAUNCH && instant==0 && m->state!=MAZE_SEARCH &&
       m->state!=MAZE_SETTLE && m->state!=MAZE_TURN_LEFT && m->state!=MAZE_TURN_RIGHT) {
        if(m->state==MAZE_INSPECT && (m->left_seen||m->right_seen)) {
            enter(m,m->left_seen ? MAZE_TURN_LEFT : MAZE_TURN_RIGHT,now,"Branch confirmed; forward line ended");
            m->left_old_line=true;
        } else enter(m,MAZE_SETTLE,now,"No line: stop forward, then search left");
        m->left=m->right=0;
        goto done;
    }
    if(m->state==MAZE_LAUNCH) {
        if(now-m->since < (uint32_t)c->launch_ms) { l=r=35; goto drive; }
        enter(m,instant ? MAZE_FOLLOW : MAZE_SETTLE,now,"Startup launch complete");
    }
    // An actual all-white area overrides a pivot/search as well as normal following.
    // Require the unfiltered readings too, so a stale filtered pattern cannot drive on black.
    if(all_white && physical==31 && !m->inverted) {
        if(m->state!=MAZE_FOLLOW || strcmp(m->reason,"All white: straight, no finish stop"))
            enter(m,MAZE_FOLLOW,now,"All white: straight, no finish stop");
        if(m->left*m->right<0) m->left=m->right=0;
        m->armed=false; m->clear_since=now; m->wide_since=0;
        pid_reset(m); l=r=30; goto drive;
    }
    if(m->state==MAZE_SETTLE) {
        m->left=m->right=0;
        if(instant>0 && count>0) enter(m,MAZE_FOLLOW,now,"White returned before search");
        else if(now-m->since>=20) enter(m,MAZE_SEARCH,now,"No line: continuous left search");
        goto done;
    }
    if(m->state==MAZE_SEARCH) {
        l=-c->turn; r=c->turn;
        // Do not resume just because an outer sensor grazes the line.
        m->hits=(normal || count==5) && instant>0 ? m->hits+1 : 0;
        if(m->hits>=3) {
            m->left=m->right=0; m->armed=false; m->clear_since=now;
            enter(m,MAZE_FOLLOW,now,"Line centred: resume forward");
            goto done;
        }
        goto drive;
    }
    if(m->state==MAZE_TURN_LEFT || m->state==MAZE_TURN_RIGHT) {
        bool turn_left=m->state==MAZE_TURN_LEFT;
        l=turn_left ? -c->turn : c->turn; r=-l;
        if(!centre) m->left_old_line=true;
        m->hits=(m->left_old_line && normal && now-m->since>=60) ? m->hits+1 : 0;
        if(m->hits>=3) {
            m->left=m->right=0; m->armed=false; m->clear_since=now;
            enter(m,MAZE_FOLLOW,now,"Turn aligned with active line");
            goto done;
        }
        if(now-m->since>=1400) {
            enter(m,MAZE_SEARCH,now,"Turn not aligned: continue searching left");
            l=-c->turn; r=c->turn;
        }
        goto drive;
    }
    if(m->state==MAZE_INSPECT) {
        m->left_seen|=left; m->right_seen|=right;
        l=r=40;
        if(now-m->since >= (uint32_t)c->cross_ms) {
            bool straight=m->white_bits[1]||centre||m->white_bits[3];
            if(m->left_seen) enter(m,MAZE_TURN_LEFT,now,"Junction priority: LEFT");
            else if(straight) enter(m,MAZE_FOLLOW,now,"No left branch: STRAIGHT");
            else if(m->right_seen) enter(m,MAZE_TURN_RIGHT,now,"No left/straight: RIGHT");
            else enter(m,MAZE_SEARCH,now,"Dead end: search left toward return line");
            m->armed=false; m->clear_since=now; m->left=m->right=0;
            goto done;
        }
        goto drive;
    }
    if(!m->armed) {
        if(wide) m->clear_since=now;
        else if(now-m->clear_since>=60) m->armed=true;
    }
    if(wide && m->armed && !(m->color_locked && now-m->color_changed<300)) {
        if(!m->wide_since) { m->wide_since=now; m->left_seen=left; m->right_seen=right; }
        m->left_seen|=left; m->right_seen|=right;
        if(now-m->wide_since>=20) {
            enter(m,MAZE_INSPECT,now,"Wide pattern confirmed: inspecting branches");
            m->wide_since=0; l=r=40; goto drive;
        }
    } else m->wide_since=0;
    if((left||right)&&!centre&&count<=2&&!(left&&right)) {
        maze_state_t next=left ? MAZE_SHARP_LEFT : MAZE_SHARP_RIGHT;
        if(m->state!=next) enter(m,next,now,"Edge line: pivot through sharp corner");
        l=left ? -25 : c->turn; r=left ? c->turn : -25;
        goto drive;
    }
    if(m->state!=MAZE_FOLLOW || !strcmp(m->reason,"All white: straight, no finish stop"))
        enter(m,MAZE_FOLLOW,now,"Tracking active line");
    // Seed after turns/pause so re-entry cannot generate a derivative kick.
    float error=fabsf(m->error)<.7f ? 0 : m->error;
    if(!m->pid_ready) { m->previous=error; m->pid_ready=true; }
    float alpha=1.0f-powf(.75f,loops);
    m->derivative+=alpha*((error-m->previous)/loops-m->derivative);
    m->previous=error;
    float base=clamp(c->cruise-.35f*fabsf(error)-.8f*fabsf(m->derivative),25,c->cruise);
    // Reserve steering authority even near the cruise ceiling. Slow the
    // common base when needed so neither wheel clips the differential.
    float limit=30;
    float candidate=clamp(m->integral+error*loops,-30,30);
    float requested=c->kp*error+c->ki*candidate+c->kd*m->derivative;
    if(fabsf(requested)<=limit || requested*error<0) m->integral=candidate;
    float correction=clamp(c->kp*error+c->ki*m->integral+c->kd*m->derivative,-limit,limit);
    base=fminf(base,60-fabsf(correction));
    l=clamp(base+correction,0,60); r=clamp(base-correction,0,60);
drive:
    m->left=slew(m->left,l,loops); m->right=slew(m->right,r,loops);
done:
    out->left=m->left; out->right=m->right; out->state=m->state;
    out->error=m->error; out->sequence=m->sequence;
    snprintf(out->reason,sizeof(out->reason),"%s",m->reason ? m->reason : "");
}

