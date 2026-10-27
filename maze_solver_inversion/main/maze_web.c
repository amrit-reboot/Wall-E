#include "maze_web.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cJSON.h"

static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static maze_config_t settings;
static maze_sample_t latest;
static bool paused, nvs_ready;
static esp_netif_t *ap_netif,*sta_netif;
static esp_timer_handle_t reconnect_timer;
typedef struct { uint32_t sequence,ms; char text[96]; } event_t;
static event_t events[8];
static unsigned event_count;
static int capture_black[5],capture_white[5];
static bool have_black,have_white;

static bool save_config(const maze_config_t *c)
{
    if(!nvs_ready) return false;
    nvs_handle_t h;
    if(nvs_open("maze_inv",NVS_READWRITE,&h)!=ESP_OK) return false;
    esp_err_t e=nvs_set_blob(h,"config_v3",c,sizeof(*c));
    if(e==ESP_OK) e=nvs_commit(h);
    nvs_close(h); return e==ESP_OK;
}
void maze_shared_init(void)
{
    settings=maze_defaults();
    settings.launch_ms=CONFIG_MAZE_LAUNCH_MS;
    settings.cross_ms=CONFIG_MAZE_CROSS_MS;
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES || e==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        e=nvs_flash_erase(); if(e==ESP_OK) e=nvs_flash_init();
    }
    nvs_ready=e==ESP_OK;
    if(nvs_ready) {
        nvs_handle_t h;
        if(nvs_open("maze_inv",NVS_READONLY,&h)==ESP_OK) {
            maze_config_t stored; size_t n=sizeof(stored);
            bool migrated=false;
            if(nvs_get_blob(h,"config_v3",&stored,&n)==ESP_OK && n==sizeof(stored) && maze_config_valid(&stored)) {
                settings=stored;
            } else {
                migrated=true;
                n=sizeof(stored);
                if(nvs_get_blob(h,"config_v2",&stored,&n)==ESP_OK && n==sizeof(stored) && maze_config_valid(&stored)) {
                    // Apply the faster profile once; retain calibration and PID tuning.
                    maze_config_t faster=maze_defaults();
                    settings=stored;
                    settings.cruise=faster.cruise; settings.turn=faster.turn;
                    settings.cross_ms=CONFIG_MAZE_CROSS_MS;
                } else {
                    n=sizeof(stored);
                    if(nvs_get_blob(h,"config_v1",&stored,&n)==ESP_OK && n==sizeof(stored) && maze_config_valid(&stored)) {
                        memcpy(settings.black,stored.black,sizeof(settings.black));
                        memcpy(settings.white,stored.white,sizeof(settings.white));
                    }
                }
            }
            nvs_close(h);
            if(migrated) save_config(&settings);
        }
    }
}
void maze_get_settings(maze_config_t *c,bool *p)
{ portENTER_CRITICAL(&lock); *c=settings; *p=paused; portEXIT_CRITICAL(&lock); }
void maze_get_sample(maze_sample_t *s)
{ portENTER_CRITICAL(&lock); *s=latest; portEXIT_CRITICAL(&lock); }
void maze_publish(const maze_sample_t *s)
{
    portENTER_CRITICAL(&lock);
    if(!latest.ready || latest.sequence!=s->sequence || latest.state!=s->state) {
        event_t *e=&events[event_count%8]; e->ms=s->ms; e->sequence=s->sequence;
        memcpy(e->text,s->reason,sizeof(e->text)); event_count++;
    }
    latest=*s;
    portEXIT_CRITICAL(&lock);
}
void maze_network_text(char ap[16],char sta[16])
{
    esp_netif_ip_info_t ip;
    snprintf(ap,16,"0.0.0.0"); snprintf(sta,16,"0.0.0.0");
    if(ap_netif && esp_netif_get_ip_info(ap_netif,&ip)==ESP_OK)
        snprintf(ap,16,IPSTR,IP2STR(&ip.ip));
    if(sta_netif && esp_netif_get_ip_info(sta_netif,&ip)==ESP_OK)
        snprintf(sta,16,IPSTR,IP2STR(&ip.ip));
}
static void reconnect(void *arg) { (void)arg; esp_wifi_connect(); }
static void wifi_event(void *arg,esp_event_base_t base,int32_t id,void *data)
{
    (void)arg;
    if(base==WIFI_EVENT && id==WIFI_EVENT_STA_START) esp_wifi_connect();
    else if(base==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) {
        esp_timer_stop(reconnect_timer);
        esp_timer_start_once(reconnect_timer,2000000);
    } else if(base==IP_EVENT && id==IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *got=data;
        ESP_LOGI("maze_wifi","Router IP: " IPSTR,IP2STR(&got->ip_info.ip));
    }
}
static esp_err_t network_start(void)
{
    esp_err_t e;
#define NET_TRY(x) do { e=(x); if(e!=ESP_OK) return e; } while(0)
    NET_TRY(esp_netif_init()); NET_TRY(esp_event_loop_create_default());
    ap_netif=esp_netif_create_default_wifi_ap();
    if(!ap_netif) return ESP_ERR_NO_MEM;
    bool station=strlen(CONFIG_MAZE_STA_SSID)>0;
    if(station) { sta_netif=esp_netif_create_default_wifi_sta(); if(!sta_netif) return ESP_ERR_NO_MEM; }
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    NET_TRY(esp_wifi_init(&init));
    // Keep Wi-Fi credentials out of the persistent controller configuration.
    NET_TRY(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    NET_TRY(esp_wifi_set_mode(station ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    wifi_config_t ap={.ap={.ssid="WallE-Maze",.password="wallemaze",
        .ssid_len=10,.channel=1,.max_connection=4,.authmode=WIFI_AUTH_WPA2_PSK}};
    NET_TRY(esp_wifi_set_config(WIFI_IF_AP,&ap));
    if(station) {
        const esp_timer_create_args_t t={.callback=reconnect,.name="maze_reconnect"};
        NET_TRY(esp_timer_create(&t,&reconnect_timer));
        NET_TRY(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event,NULL));
        NET_TRY(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event,NULL));
        wifi_config_t sta={0};
        strlcpy((char *)sta.sta.ssid,CONFIG_MAZE_STA_SSID,sizeof(sta.sta.ssid));
        strlcpy((char *)sta.sta.password,CONFIG_MAZE_STA_PASSWORD,sizeof(sta.sta.password));
        NET_TRY(esp_wifi_set_config(WIFI_IF_STA,&sta));
    }
    NET_TRY(esp_wifi_start());
    ESP_LOGI("maze_wifi","Dashboard: join WallE-Maze (password wallemaze), open http://192.168.4.1");
    return ESP_OK;
#undef NET_TRY
}
static esp_err_t json_send(httpd_req_t *req,cJSON *root)
{
    if(!root) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Out of memory");
    char *body=cJSON_PrintUnformatted(root); cJSON_Delete(root);
    if(!body) return httpd_resp_send_err(req,HTTPD_500_INTERNAL_SERVER_ERROR,"Out of memory");
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    esp_err_t e=httpd_resp_sendstr(req,body); cJSON_free(body); return e;
}
static esp_err_t state_get(httpd_req_t *req)
{
    maze_sample_t s; maze_config_t c; bool p;
    event_t history[8]; unsigned total;
    maze_get_sample(&s); maze_get_settings(&c,&p);
    portENTER_CRITICAL(&lock); memcpy(history,events,sizeof(history)); total=event_count; portEXIT_CRITICAL(&lock);
    cJSON *root=cJSON_CreateObject(); if(!root) return json_send(req,NULL);
    cJSON_AddBoolToObject(root,"ready",s.ready); cJSON_AddBoolToObject(root,"paused",p);
    cJSON_AddNumberToObject(root,"ms",s.ms);
    cJSON_AddNumberToObject(root,"age_ms",(uint32_t)(esp_timer_get_time()/1000)-s.ms);
    cJSON_AddStringToObject(root,"state",maze_state_name(s.state));
    cJSON_AddStringToObject(root,"reason",s.reason);
    cJSON_AddNumberToObject(root,"bits",s.bits);
    cJSON_AddNumberToObject(root,"white_bits",s.white_bits);
    cJSON_AddBoolToObject(root,"inverted",s.inverted);
    cJSON_AddNumberToObject(root,"left",s.left); cJSON_AddNumberToObject(root,"right",s.right);
    cJSON_AddNumberToObject(root,"error",s.error);
    cJSON_AddItemToObject(root,"raw",cJSON_CreateIntArray(s.raw,5));
    cJSON_AddItemToObject(root,"values",cJSON_CreateIntArray(s.values,5));
    cJSON_AddItemToObject(root,"black",cJSON_CreateIntArray(c.black,5));
    cJSON_AddItemToObject(root,"white",cJSON_CreateIntArray(c.white,5));
    cJSON_AddNumberToObject(root,"kp",c.kp); cJSON_AddNumberToObject(root,"ki",c.ki); cJSON_AddNumberToObject(root,"kd",c.kd);
    cJSON_AddNumberToObject(root,"threshold",c.threshold); cJSON_AddNumberToObject(root,"hysteresis",c.hysteresis);
    cJSON_AddNumberToObject(root,"cruise",c.cruise); cJSON_AddNumberToObject(root,"turn",c.turn);
    cJSON_AddBoolToObject(root,"black_captured",have_black); cJSON_AddBoolToObject(root,"white_captured",have_white);
    char ap[16],sta[16]; maze_network_text(ap,sta);
    cJSON_AddStringToObject(root,"ap",ap); cJSON_AddStringToObject(root,"sta",sta);
    cJSON *list=cJSON_AddArrayToObject(root,"events");
    unsigned start=total>8 ? total-8 : 0;
    for(unsigned i=start;i<total;i++) {
        event_t *e=&history[i%8]; cJSON *item=cJSON_CreateObject();
        cJSON_AddNumberToObject(item,"ms",e->ms); cJSON_AddStringToObject(item,"text",e->text);
        cJSON_AddItemToArray(list,item);
    }
    return json_send(req,root);
}
static cJSON *receive_json(httpd_req_t *req)
{
    char body[768];
    if(req->content_len<=0 || req->content_len>=(int)sizeof(body)) return NULL;
    int done=0,retries=0;
    while(done<req->content_len) {
        int n=httpd_req_recv(req,body+done,req->content_len-done);
        if(n==HTTPD_SOCK_ERR_TIMEOUT && retries++<2) continue;
        if(n<=0) return NULL;
        done+=n;
    }
    body[done]=0; return cJSON_Parse(body);
}
static esp_err_t bad(httpd_req_t *r,const char *text)
{ return httpd_resp_send_err(r,HTTPD_400_BAD_REQUEST,text); }
static esp_err_t run_post(httpd_req_t *req)
{
    cJSON *root=receive_json(req);
    cJSON *v=cJSON_GetObjectItemCaseSensitive(root,"running");
    if(!cJSON_IsBool(v)) { cJSON_Delete(root); return bad(req,"running must be boolean"); }
    bool stop=!cJSON_IsTrue(v);
    portENTER_CRITICAL(&lock); paused=stop; portEXIT_CRITICAL(&lock);
    have_black=have_white=false;
    cJSON_Delete(root); return httpd_resp_sendstr(req,stop ? "Paused" : "Running");
}
static esp_err_t settings_post(httpd_req_t *req)
{
    cJSON *root=receive_json(req);
    const char *keys[]={"kp","ki","kd","threshold","cruise","turn"};
    float values[6];
    for(int i=0;i<6;i++) {
        cJSON *v=cJSON_GetObjectItemCaseSensitive(root,keys[i]);
        if(!cJSON_IsNumber(v) || !isfinite(v->valuedouble)) {
            cJSON_Delete(root); return bad(req,"All tuning fields must be finite numbers");
        }
        values[i]=(float)v->valuedouble;
    }
    cJSON_Delete(root);
    maze_config_t c; bool p; maze_get_settings(&c,&p);
    // Check floating-point ranges before conversion to an integer threshold.
    if(values[3]<100 || values[3]>950) return bad(req,"Threshold must be 100..950");
    c.kp=values[0]; c.ki=values[1]; c.kd=values[2]; c.threshold=(int)values[3]; c.cruise=values[4]; c.turn=values[5];
    if(!maze_config_valid(&c)) return bad(req,"PID 0..100; duty 25..60; threshold 100..950");
    portENTER_CRITICAL(&lock); settings=c; portEXIT_CRITICAL(&lock);
    bool saved=save_config(&c);
    return httpd_resp_sendstr(req,saved ? "Applied and saved" : "Applied in RAM; persistent save unavailable");
}
static esp_err_t calibrate_post(httpd_req_t *req)
{
    maze_config_t c; bool p; maze_get_settings(&c,&p);
    maze_sample_t s; maze_get_sample(&s);
    if(!p || s.state!=MAZE_PAUSED) return bad(req,"Pause robot and wait for Paused status first");
    cJSON *root=receive_json(req); cJSON *v=cJSON_GetObjectItemCaseSensitive(root,"surface");
    if(!cJSON_IsString(v)) { cJSON_Delete(root); return bad(req,"surface must be black or white"); }
    if(!strcmp(v->valuestring,"black")) { memcpy(capture_black,s.raw,sizeof(capture_black)); have_black=true; }
    else if(!strcmp(v->valuestring,"white")) { memcpy(capture_white,s.raw,sizeof(capture_white)); have_white=true; }
    else { cJSON_Delete(root); return bad(req,"Unknown calibration surface"); }
    cJSON_Delete(root);
    if(!have_black || !have_white) return httpd_resp_sendstr(req,"Captured; capture the other surface to apply calibration");
    memcpy(c.black,capture_black,sizeof(c.black)); memcpy(c.white,capture_white,sizeof(c.white));
    if(!maze_config_valid(&c)) return bad(req,"Insufficient contrast: each sensor needs at least 100 ADC counts between black and white; recapture");
    portENTER_CRITICAL(&lock); settings=c; portEXIT_CRITICAL(&lock);
    bool saved=save_config(&c); have_black=have_white=false;
    return httpd_resp_sendstr(req,saved ? "Calibration applied and saved" : "Calibration applied in RAM; save unavailable");
}
extern const unsigned char dashboard_start[] asm("_binary_index_html_start");
extern const unsigned char dashboard_end[] asm("_binary_index_html_end");
static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req,(const char *)dashboard_start,dashboard_end-dashboard_start-1);
}
void maze_web_start(void)
{
    esp_err_t e=network_start();
    if(e!=ESP_OK) { ESP_LOGE("maze_web","Network unavailable: %s",esp_err_to_name(e)); return; }
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG(); cfg.stack_size=8192; cfg.lru_purge_enable=true;
    cfg.recv_wait_timeout=2; cfg.send_wait_timeout=2;
    httpd_handle_t server=NULL;
    e=httpd_start(&server,&cfg);
    if(e!=ESP_OK) { ESP_LOGE("maze_web","HTTP unavailable: %s",esp_err_to_name(e)); return; }
    const httpd_uri_t routes[]={
        {.uri="/",.method=HTTP_GET,.handler=index_get},
        {.uri="/api/state",.method=HTTP_GET,.handler=state_get},
        {.uri="/api/run",.method=HTTP_POST,.handler=run_post},
        {.uri="/api/settings",.method=HTTP_POST,.handler=settings_post},
        {.uri="/api/calibrate",.method=HTTP_POST,.handler=calibrate_post}};
    for(unsigned i=0;i<sizeof(routes)/sizeof(routes[0]);i++) {
        e=httpd_register_uri_handler(server,&routes[i]);
        if(e!=ESP_OK) ESP_LOGE("maze_web","Route registration failed: %s",esp_err_to_name(e));
    }
}

