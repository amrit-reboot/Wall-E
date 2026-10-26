#include "maze_oled.h"
#include "sdkconfig.h"
#ifdef CONFIG_MAZE_OLED
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
static i2c_master_dev_handle_t device;
static bool enabled;
// Original compact 3x5 font, rows packed in 3-bit groups, A-Z then 0-9.
static const uint16_t glyphs[]={
    0x2bed,0x6bae,0x3923,0x6b6e,0x79a7,0x79a4,0x396b,0x5bed,0x7497,
    0x126a,0x5bad,0x4927,0x5fed,0x5ffd,0x2b6a,0x6ba4,0x2b7b,0x6bad,
    0x388e,0x7492,0x5b6f,0x5b6a,0x5bfd,0x5aad,0x5a92,0x72a7,0x7b6f,
    0x2c97,0x62a7,0x628e,0x5bc9,0x798e,0x39aa,0x7292,0x2aaa,0x2ace};
static uint8_t frame[1024];
static void text(int page,const char *s)
{
    for(int x=0; *s && x<126; x+=4,s++) {
        int ch=toupper((unsigned char)*s); uint16_t g=0;
        if(ch>='A'&&ch<='Z') g=glyphs[ch-'A'];
        else if(ch>='0'&&ch<='9') g=glyphs[26+ch-'0'];
        else if(ch=='.') g=2; else if(ch=='-') g=7<<6; else if(ch==':') g=(2<<9)|(2<<3);
        for(int col=0;col<3;col++) for(int row=0;row<5;row++)
            if(g&(1<<(14-row*3-col))) frame[page*128+x+col]|=1<<row;
    }
}
static esp_err_t transmit(const uint8_t *bytes,size_t n)
{ return i2c_master_transmit(device,bytes,n,30); }
void maze_oled_init(void)
{
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t b={.i2c_port=I2C_NUM_0,.sda_io_num=21,.scl_io_num=22,
        .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,.flags.enable_internal_pullup=true};
    if(i2c_new_master_bus(&b,&bus)!=ESP_OK) return;
    if(i2c_master_probe(bus,0x3c,30)!=ESP_OK) { ESP_LOGW("maze_oled","No OLED at 0x3c; control continues"); return; }
    const i2c_device_config_t d={.dev_addr_length=I2C_ADDR_BIT_LEN_7,.device_address=0x3c,.scl_speed_hz=400000};
    if(i2c_master_bus_add_device(bus,&d,&device)!=ESP_OK) return;
    const uint8_t init[]={0,0xae,0xd5,0x80,0xa8,0x3f,0xd3,0,0x40,0x8d,0x14,
        0x20,0,0xa1,0xc8,0xda,0x12,0x81,0x7f,0xd9,0xf1,0xdb,0x40,0xa4,0xa6,0xaf};
    enabled=transmit(init,sizeof(init))==ESP_OK;
}
void maze_oled_update(const maze_sample_t *s,const char *ap,const char *sta)
{
    if(!enabled) return;
    memset(frame,0,sizeof(frame)); char line[40];
    text(0,"WALL E MAZE"); text(1,maze_state_name(s->state));
    snprintf(line,sizeof(line),"LSA %d%d%d%d%d",!!(s->bits&1),!!(s->bits&2),!!(s->bits&4),!!(s->bits&8),!!(s->bits&16));
    text(2,line);
    text(4,s->inverted ? "BLACK LINE" : "WHITE LINE");
    snprintf(line,sizeof(line),"L %.0f R %.0f",(double)s->left,(double)s->right); text(3,line);
    text(5,"AP"); text(6,ap);
    if(strcmp(sta,"0.0.0.0")) { text(5,"                  STA"); text(7,sta); }
    const uint8_t window[]={0,0x21,0,127,0x22,0,7};
    if(transmit(window,sizeof(window))!=ESP_OK) { enabled=false; return; }
    for(int i=0;i<1024;i+=128) {
        uint8_t packet[129]; packet[0]=0x40; memcpy(packet+1,frame+i,128);
        if(transmit(packet,sizeof(packet))!=ESP_OK) { enabled=false; break; }
    }
}
#else
void maze_oled_init(void) {}
void maze_oled_update(const maze_sample_t *s,const char *ap,const char *sta)
{ (void)s; (void)ap; (void)sta; }
#endif
