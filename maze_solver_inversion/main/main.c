#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "maze_board.h"
#include "maze_controller.h"
#include "maze_web.h"
#include "maze_oled.h"

static void control_task(void *arg)
{
    (void)arg;
    esp_err_t err=maze_board_init();
    maze_controller_t controller;
    maze_init(&controller,(uint32_t)(esp_timer_get_time()/1000));
    controller.finish_confirm_ms=CONFIG_MAZE_FINISH_CONFIRM_MS;
    maze_sample_t sample={0};
    TickType_t tick=xTaskGetTickCount();
    uint32_t last_log=0, last_sequence=0;
    for(;;) {
        uint32_t now=(uint32_t)(esp_timer_get_time()/1000);
        int raw[5];
        if(err==ESP_OK) err=maze_board_read(raw);
        if(err==ESP_OK) {
            maze_config_t config; bool paused;
            maze_get_settings(&config,&paused);
            maze_step(&controller,&config,raw,now,paused,&sample);
            err=maze_board_drive(sample.left,sample.right);
        }
        if(err!=ESP_OK) {
            maze_board_stop(); sample.state=MAZE_FAULT; sample.left=sample.right=0;
            sample.ms=now; sample.ready=true;
            snprintf(sample.reason,sizeof(sample.reason),"Hardware error: %s; reset after checking wiring",esp_err_to_name(err));
        }
        maze_publish(&sample);
        if(now-last_log>=200 || sample.sequence!=last_sequence) {
            ESP_LOGI("maze","%s raw=[%d %d %d %d %d] white=%d%d%d%d%d error=%.1f L=%.0f R=%.0f | %s",
                maze_state_name(sample.state),sample.raw[0],sample.raw[1],sample.raw[2],sample.raw[3],sample.raw[4],
                !!(sample.bits&1),!!(sample.bits&2),!!(sample.bits&4),!!(sample.bits&8),!!(sample.bits&16),
                (double)sample.error,(double)sample.left,(double)sample.right,sample.reason);
            last_log=now; last_sequence=sample.sequence;
        }
        xTaskDelayUntil(&tick,pdMS_TO_TICKS(10));
    }
}
static void service_task(void *arg)
{
    (void)arg;
    maze_web_start(); // Never blocks control_task on router association or HTTP clients.
    maze_oled_init();
    for(;;) {
        maze_sample_t s; char ap[16],sta[16];
        maze_get_sample(&s); maze_network_text(ap,sta);
        maze_oled_update(&s,ap,sta);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void app_main(void)
{
    maze_shared_init();
    if(xTaskCreate(control_task,"maze_control",6144,NULL,5,NULL)!=pdPASS) {
        ESP_LOGE("maze","Cannot start controller"); return;
    }
    if(xTaskCreate(service_task,"maze_services",8192,NULL,2,NULL)!=pdPASS)
        ESP_LOGE("maze","Dashboard task unavailable; controller still running");
}
