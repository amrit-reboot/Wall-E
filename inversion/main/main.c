#include <limits.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "inversion_board.h"
#include "inversion_controller.h"

static QueueHandle_t commands;
static void console_task(void *arg)
{
    (void)arg;
    for(;;) {
        int ch=getchar();
        if(ch=='c' || ch=='g' || ch=='x' || ch=='t') {
            char command=(char)ch;
            xQueueSend(commands,&command,0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static uint32_t now_ms(void)
{ return (uint32_t)(esp_timer_get_time()/1000); }

void app_main(void)
{
    commands=xQueueCreate(8,sizeof(char));
    if(!commands) { ESP_LOGE("inversion","Cannot allocate command queue"); return; }
    esp_err_t err=inversion_board_init();
    if(err!=ESP_OK) { inversion_board_stop(); ESP_ERROR_CHECK(err); }
    inversion_board_stop();
    if(xTaskCreate(console_task,"inv_console",3072,NULL,2,NULL)!=pdPASS) {
        ESP_LOGE("inversion","Cannot start console; motors remain stopped"); return;
    }
    inv_config_t cfg=inv_default_config();
    cfg.cruise=CONFIG_INV_CRUISE; cfg.creep=CONFIG_INV_CREEP;
    cfg.confirm_ms=CONFIG_INV_CONFIRM_MS;
    cfg.minimum=CONFIG_INV_MIN_DUTY; cfg.start_duty=CONFIG_INV_START_DUTY;
    cfg.start_ms=CONFIG_INV_START_MS;
    cfg.kp=CONFIG_INV_KP_X10/10.0f; cfg.kd=CONFIG_INV_KD_X1000/1000.0f;
    inv_controller_t controller;
    inv_init(&controller,now_ms());
    int low[5]={0}, high[5]={0};
    bool calibrating=false, calibrated=false, armed=false, running=false;
    bool stable_line=false;
    bool testing=false;
    uint32_t test_since=0;
    uint32_t calibration_start=0, line_since=0, last_log=0;
    TickType_t wake=xTaskGetTickCount();
    ESP_LOGI("inversion","c + Enter: calibrate; sweep EVERY sensor across black and white for %d ms, motors off",
             CONFIG_INV_CALIBRATION_MS);
    ESP_LOGI("inversion","g + Enter: arm/start on a line; x + Enter: stop. Calibration is required after every reset.");
    ESP_LOGI("inversion","t + Enter: wheels-lifted motor test, 55%% for 500 ms; x stops early");
    for(;;) {
        uint32_t now=now_ms();
        char command;
        while(xQueueReceive(commands,&command,0)==pdTRUE) {
            if(command=='x' || command=='c' || command=='t') {
                armed=running=stable_line=false;
                testing=false;
                inversion_board_stop(); inv_init(&controller,now);
                calibrating=false;
            }
            if(command=='t') {
                testing=true; test_since=now;
                ESP_LOGW("inversion","MOTOR TEST: commanding both motors 55%% for 500 ms, independent of sensors");
            } else if(command=='c') {
                calibrated=false; calibrating=true; calibration_start=now;
                for(int i=0;i<5;i++) { low[i]=INT_MAX; high[i]=0; }
                ESP_LOGI("inversion","CALIBRATING: manually sweep all five sensors over BOTH surfaces at running height");
            } else if(command=='g') {
                testing=false; inversion_board_stop();
                if(!calibrated || calibrating) {
                    ESP_LOGW("inversion","Calibrate first using c");
                } else {
                    inv_init(&controller,now); armed=true; running=false; stable_line=false;
                    inversion_board_stop();
                    ESP_LOGI("inversion","ARMED: place on a narrow white OR black line; starts after 500 ms of stable detection");
                }
            } else if(command=='x') ESP_LOGI("inversion","STOPPED by operator");
        }
        int raw[5], brightness[5]={0};
        err=inversion_board_read(raw);
        if(err!=ESP_OK) { inversion_board_stop(); ESP_ERROR_CHECK(err); }
        if(testing) {
            if(now-test_since<500) {
                err=inversion_board_drive(55,55);
                if(err!=ESP_OK) { inversion_board_stop(); ESP_ERROR_CHECK(err); }
            } else {
                testing=false; inversion_board_stop();
                ESP_LOGI("inversion","MOTOR TEST ended; did both wheels turn? Motors stopped.");
            }
        }
        if(calibrating) {
            for(int i=0;i<5;i++) {
                if(raw[i]<low[i]) low[i]=raw[i];
                if(raw[i]>high[i]) high[i]=raw[i];
            }
            if(now-calibration_start>=CONFIG_INV_CALIBRATION_MS) {
                calibrating=false; calibrated=true;
                for(int i=0;i<5;i++) {
                    int span=high[i]-low[i];
                    ESP_LOGI("inversion","Sensor %d: white ADC=%d black ADC=%d span=%d",i,low[i],high[i],span);
                    if(span<350) calibrated=false;
                }
                if(calibrated) ESP_LOGI("inversion","Calibration OK. Put on the line, then g + Enter.");
                else ESP_LOGE("inversion","Calibration rejected: every sensor needs ADC span >=350. Check height/wiring and repeat c.");
            }
        }
        if(calibrated) {
            for(int i=0;i<5;i++) {
                int v=(high[i]-raw[i])*1000/(high[i]-low[i]);
                brightness[i]=v<0 ? 0 : v>1000 ? 1000 : v;
            }
        }
        inv_output_t out={.state=INV_WAIT};
        if(testing) out.left=out.right=55;
        if(armed) {
            inv_step(&controller,&cfg,brightness,now,&out);
            if(!running) {
                bool endpoints_same=controller.white[0]==controller.white[4];
                if(out.valid_line && endpoints_same) {
                    if(!stable_line) { stable_line=true; line_since=now; }
                    if(now-line_since>=500) running=true;
                } else stable_line=false;
                /* Arming never accumulates motor duty behind the stop gate. */
                controller.left=controller.right=0;
                controller.have_previous=false;
                controller.motion_started=false;
                out.left=out.right=0;
                if(!running) out.state=INV_WAIT;
                /* A missing line during arming is allowed indefinitely. */
                controller.stopped=false; controller.lost=false;
            }
            if(out.state==INV_STOPPED) {
                armed=running=false;
                inversion_board_stop();
                ESP_LOGW("inversion","Line unavailable for %lu ms; stopped. Reposition, then g to restart.",
                         (unsigned long)cfg.stop_ms);
            } else if(running) {
                err=inversion_board_drive(out.left,out.right);
                if(err!=ESP_OK) { inversion_board_stop(); ESP_ERROR_CHECK(err); }
            }
            if(out.switched) ESP_LOGI("inversion","Polarity -> %s line, switch %u",
                                     out.black_line ? "BLACK" : "WHITE",out.switches);
        }
        if(now-last_log>=200) {
            ESP_LOGI("inversion","%s %s line b=[%d %d %d %d %d] pos=%.2f L=%.1f R=%.1f",
                     testing ? "MOTOR_TEST" : calibrating ? "CALIBRATE" : inv_state_name(out.state),
                     controller.black_line ? "BLACK" : "WHITE",
                     brightness[0],brightness[1],brightness[2],brightness[3],brightness[4],
                     (double)out.position,(double)out.left,(double)out.right);
            last_log=now;
        }
        TickType_t period=pdMS_TO_TICKS(10);
        xTaskDelayUntil(&wake,period ? period : 1);
    }
}
