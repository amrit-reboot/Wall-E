#include "inversion_board.h"
#include <math.h>
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

// SRA normal mode pin map, matching the repository's pin_defs.h.
// Never initialize the bar graph/LED matrix: they share the motor pins.
static const int lsa_gpio[5]={32,35,34,39,36};
static const int motor_gpio[4]={13,12,26,25};
static adc_oneshot_unit_handle_t adc;
static adc_channel_t channels[5];
static bool motors_ready;

esp_err_t inversion_board_init(void)
{
    const ledc_timer_config_t timer={.speed_mode=LEDC_LOW_SPEED_MODE,
        .duty_resolution=LEDC_TIMER_10_BIT,.timer_num=LEDC_TIMER_0,
        .freq_hz=20000,.clk_cfg=LEDC_AUTO_CLK};
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer),"board","PWM timer");
    for(int i=0;i<4;i++) {
        const ledc_channel_config_t ch={.gpio_num=motor_gpio[i],.speed_mode=LEDC_LOW_SPEED_MODE,
            .channel=(ledc_channel_t)i,.intr_type=LEDC_INTR_DISABLE,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch),"board","PWM channel");
    }
    motors_ready=true;
    const adc_oneshot_unit_init_cfg_t unit={.unit_id=ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit,&adc),"board","ADC init");
    const adc_oneshot_chan_cfg_t cfg={.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_12};
    for(int i=0;i<5;i++) {
        adc_unit_t u;
        ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(lsa_gpio[i],&u,&channels[i]),"board","ADC pin");
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc,channels[i],&cfg),"board","ADC channel");
    }
    return ESP_OK;
}
esp_err_t inversion_board_read(int raw[5])
{
    for(int i=0;i<5;i++) {
        int sum=0;
        for(int j=0;j<8;j++) {
            int value;
            ESP_RETURN_ON_ERROR(adc_oneshot_read(adc,channels[i],&value),"board","ADC read");
            sum+=value;
        }
#ifdef CONFIG_INV_REVERSE_SENSORS
        raw[4-i]=sum/8;
#else
        raw[i]=sum/8;
#endif
    }
    return ESP_OK;
}
static esp_err_t wheel(int index, float duty)
{
    unsigned active=(unsigned)(index*2+(duty<0));
    unsigned inactive=(unsigned)(index*2+(duty>=0));
    uint32_t ticks=(uint32_t)(fminf(fabsf(duty),60)*1023/100);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)inactive,0),"board","PWM off");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)inactive),"board","PWM off update");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)active,ticks),"board","PWM duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)active);
}
esp_err_t inversion_board_drive(float left,float right)
{
#ifdef CONFIG_INV_REVERSE_LEFT
    left=-left;
#endif
#ifdef CONFIG_INV_REVERSE_RIGHT
    right=-right;
#endif
#ifdef CONFIG_INV_SWAP_MOTORS
    ESP_RETURN_ON_ERROR(wheel(1,left),"board","left motor");
    return wheel(0,right);
#else
    ESP_RETURN_ON_ERROR(wheel(0,left),"board","left motor");
    return wheel(1,right);
#endif
}
void inversion_board_stop(void)
{
    if(!motors_ready) return;
    for(int i=0;i<4;i++) ledc_stop(LEDC_LOW_SPEED_MODE,(ledc_channel_t)i,0);
}

