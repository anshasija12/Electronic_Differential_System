#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "math.h"
#include "freertos/timers.h"
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"
#include "esp_system.h"
#include "driver/uart.h"

static const int RX_BUF_SIZE = 1024;

typedef struct recv_data 
{
    uint8_t ign;
    uint8_t acc;
    uint8_t steer;
} recv_data;

recv_data data;

typedef struct send_data 
{
    int m1_curr;
    int m2_curr;
    int bat_curr;
    int bat_volt;//
    int m1_volt;//
    int m2_volt;//
    int rpm_m1;//
    int rpm_m2;//
    int w1;//
    int w2;//
    int ang//
} send_data;

send_data sensor;

#define voltage_gain 4.4196875f

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

char bffer[256] ;

#define m1_i ADC1_CHANNEL_6
#define m2_i ADC1_CHANNEL_7
#define bat_i ADC1_CHANNEL_0
#define m1_v ADC1_CHANNEL_4
#define m2_v ADC1_CHANNEL_5
#define bat_v ADC1_CHANNEL_3

#define m1 GPIO_NUM_2 
#define m2 GPIO_NUM_4

#define CH0 LEDC_CHANNEL_0
#define CH1 LEDC_CHANNEL_1
#define PWM_FREQ 5000 
#define PWM_RESOLUTION LEDC_TIMER_13_BIT 

#define encoder1  GPIO_NUM_27
#define encoder2  GPIO_NUM_26

#define left_indi GPIO_NUM_12
#define right_indi GPIO_NUM_13
#define start_indi GPIO_NUM_14

volatile long pulse1 = 0;

volatile long pulse2 = 0;

static esp_err_t wifi_init(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();
    esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return ESP_OK;
}

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void data_recv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingdata, int data_len)
{
    memcpy(&data, incomingdata, sizeof(data));
}

float m1cvalue(void)
{
    float x;
    if(sensor.m1_curr<=86)//gain7, 1ohm shunt
    {
        x = (sensor.m1_curr/7.0f);
        return x;
    }
    if(sensor.m2_curr>86&&sensor.m2_curr<=149) //gain6
    {
        x = (sensor.m1_curr/6.0f);
        return x;
    }
    if(sensor.m2_curr>149 &&sensor.m2_curr<=372)//5.5
    {
        x = (sensor.m1_curr/5.5f);
        return x;
    }
    if(sensor.m2_curr>372 &&sensor.m2_curr<=2941)//4.75
    {
        x = (sensor.m1_curr/4.75f);
        return x;
    }
    if(sensor.m1_curr>2941)
    {
        x = 404;
        return x;
    }
    return 405; 
}

float m2cvalue(void)
{
    float x;
    if(sensor.m2_curr<=86)//gain7, 1ohm shunt
    {
        x = (sensor.m2_curr/7.0f);
        return x;
    }
    if(sensor.m2_curr>86&&sensor.m2_curr<=149) //gain6
    {
        x = (sensor.m2_curr/6.0f);
        return x;
    }
    if(sensor.m2_curr>149 &&sensor.m2_curr<=372)//5.5
    {
        x = (sensor.m2_curr/5.5f);
        return x;
    }
    if(sensor.m2_curr>372 &&sensor.m2_curr<=2941)//4.75
    {
        x = (sensor.m2_curr/4.75f);
        return x;
    }
    if(sensor.m2_curr>2941)
    {
        x = 404;
        return x;
    }
    return 405;
}

float batcval(void)
{
    float x;
    if(sensor.bat_curr<=86)//gain7, .5ohm shunt
    {
        x = (sensor.bat_curr*2)/7.0f;
        return x;
    }
    if(sensor.m2_curr>86&&sensor.m2_curr<=149) //gain6
    {
        x = (sensor.bat_curr*2)/6.0f;
        return x;
    }
    if(sensor.m2_curr>149 &&sensor.m2_curr<=372)//5.5
    {
        x = (sensor.bat_curr*2)/5.5f;
        return x;
    }
    if(sensor.m2_curr>372 &&sensor.m2_curr<=2941)//4.75
    {
        x = (sensor.bat_curr*2)/4.75f;
        return x;
    }
    if(sensor.bat_curr>2941)
    {
        x = 404;
        return x;
    }
    return 405;
}

void tx_task(void)
{
    float m1v = voltage_gain*sensor.m1_volt;
    float m2v = voltage_gain*sensor.m2_volt;
    float batv = voltage_gain*sensor.bat_volt;
    float m1c = m1cvalue();
    float m2c = m2cvalue();
    float batc = batcval();
    int a = data.ign;
    //int y =55;
    //float z = 100; //%d %d %f %f %f     %f %f %f
    sprintf(bffer, " %d ",a/*sensor.rpm_m1, sensor.rpm_m2,m1v,m2v,batv,*//*,m1c,m2c,batc*/);
    //sprintf(bffer,"%d %f ",y, z);
    uart_write_bytes(UART_NUM_2, bffer, sizeof(bffer));    
}

static esp_err_t now_init(void)
{
    esp_now_init();
    esp_now_register_recv_cb(data_recv);
    //esp_now_register_send_cb(OnDataSent);
    return ESP_OK;
}

void IRAM_ATTR Pulsecounter_1() 
{
    pulse1++;
}

void IRAM_ATTR Pulsecounter_2() 
{
    pulse2++;
}

void gpio_interrupt_init() 
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // rising edge
    io_conf.pin_bit_mask = (1ULL << encoder1) | (1ULL << encoder2);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    //flag reset
    gpio_install_isr_service(0);

    gpio_isr_handler_add(encoder1, Pulsecounter_1, NULL);

    gpio_isr_handler_add(encoder2, Pulsecounter_2, NULL);   
}

void initialize_pwm() 
{
    // Initialize timer for PWM
    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = PWM_FREQ;
    timer_conf.duty_resolution = PWM_RESOLUTION;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    // Initialize first PWM channel (CH0) using GPIO pin 2 (m1)
    ledc_channel_config_t channel_conf_ch0;
    channel_conf_ch0.gpio_num = m1;
    channel_conf_ch0.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_conf_ch0.channel = CH0;
    channel_conf_ch0.intr_type = LEDC_INTR_DISABLE;
    channel_conf_ch0.timer_sel = LEDC_TIMER_0;
    channel_conf_ch0.duty = 0;
    channel_conf_ch0.hpoint = 0;
    ledc_channel_config(&channel_conf_ch0);

    // Initialize second PWM channel (CH1) using GPIO pin 4 (m2)
    ledc_channel_config_t channel_conf_ch1;
    channel_conf_ch1.gpio_num = m2;
    channel_conf_ch1.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_conf_ch1.channel = CH1;
    channel_conf_ch1.intr_type = LEDC_INTR_DISABLE;
    channel_conf_ch1.timer_sel = LEDC_TIMER_0;
    channel_conf_ch1.duty = 0;
    channel_conf_ch1.hpoint = 0;
    ledc_channel_config(&channel_conf_ch1);
}

int RDsteer()
{
    int x = data.steer;
    int y;
    x = (x*30)/254;
    y = 15 - x ;
    return y;
}

void init_sensors()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(m1_i, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(m2_i, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(bat_i, ADC_ATTEN_DB_11); 
    adc1_config_channel_atten(bat_v, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(m1_v, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(m2_v, ADC_ATTEN_DB_11);
}

void get_sensor_val_update()
{
    sensor.m1_curr = adc1_get_raw(m1_i);
    sensor.m2_curr = adc1_get_raw(m2_i);
    sensor.bat_curr = adc1_get_raw(bat_i);
    sensor.bat_volt = adc1_get_raw(bat_v);
    sensor.m1_volt= adc1_get_raw(m1_v);
    sensor.m2_volt= adc1_get_raw(m2_v);
}

void rpm(TimerHandle_t xTimer) 
{
    sensor.rpm_m1 = (pulse1*60)/11;
    sensor.rpm_m2 = (pulse2*60)/11;
    int rpm1 = (pulse1*60)/11;
    int rpm2 = (pulse2*60)/11;
    pulse1 =0;
    pulse2 =0;
    get_sensor_val_update();
    tx_task();   
}

void indicator_init()
{
    gpio_config_t io_conf_indi;
    io_conf_indi.intr_type = GPIO_INTR_DISABLE;
    io_conf_indi.mode = GPIO_MODE_OUTPUT;
    io_conf_indi.pin_bit_mask = (1ULL << left_indi) | (1ULL << right_indi) | (1ULL << start_indi);
    io_conf_indi.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf_indi.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_indi);
    gpio_set_level(left_indi|right_indi|start_indi, 0);
}

void app_main() 
{
    uart_init();
    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(now_init());
    
    indicator_init();
    init_sensors();
    gpio_interrupt_init();

    TimerHandle_t timer= xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, rpm);
    xTimerStart(timer, 0);

    while(1)
    {
        vTaskDelay(1);
        if(data.ign==1)
        {
            gpio_set_level(start_indi, 1);   
    
            gpio_config_t io_conf_m1;
            io_conf_m1.intr_type = GPIO_INTR_DISABLE;
            io_conf_m1.mode = GPIO_MODE_OUTPUT;
            io_conf_m1.pin_bit_mask = (1ULL << m1);
            io_conf_m1.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf_m1.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_config(&io_conf_m1);

            gpio_config_t io_conf_m2;
            io_conf_m2.intr_type = GPIO_INTR_DISABLE;
            io_conf_m2.mode = GPIO_MODE_OUTPUT;
            io_conf_m2.pin_bit_mask = (1ULL << m2);
            io_conf_m2.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf_m2.pull_up_en = GPIO_PULLUP_DISABLE;
            gpio_config(&io_conf_m2);
    
            initialize_pwm();
            while(data.ign==1) 
            {
                int ang = RDsteer();
                vTaskDelay(1);
                if(ang==0)
                {
                    sensor.ang= ang;
                    gpio_set_level(right_indi, 1);
                    gpio_set_level(left_indi, 1);
                
                    int pot_value = data.acc;
                    uint32_t duty_cycle = (pot_value / 255.0) * (1 << PWM_RESOLUTION);

                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, duty_cycle);
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, duty_cycle);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);

                    sensor.w1= duty_cycle;
                    sensor.w2= duty_cycle;
                
                    printf("w1: %lu, w2: %lu, ang: %d\n ", duty_cycle, duty_cycle, ang);
                }
                
                if(ang<0)//left turn
                {
                    gpio_set_level(left_indi, 1);
                    gpio_set_level(right_indi, 0);    

                    sensor.ang= ang; 

                    float angle = (-1*ang/180.0f)*M_PI;
                    float r3= 20/tan(angle) -7.5;
                    float r4= 20/tan(angle) +7.5;
                    float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

                    int pot = (data.acc/255.0) * (1 << PWM_RESOLUTION);
                    uint32_t w1_pwm = (pot*r3)/rcg;
                    uint32_t w2_pwm = (pot*r4)/rcg;
        
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w1_pwm);
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w2_pwm);

                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);

                    sensor.w1 = w1_pwm;
                    sensor.w2 = w2_pwm;

                    printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
                }

               if(ang>0)// right turn
                {
                    gpio_set_level(left_indi, 0);
                    gpio_set_level(right_indi, 1);
                    
                    sensor.ang= ang;

                    float angle = (1*ang/180.0f)*M_PI;
                    float r3= 20/tan(angle) -7.5;
                    float r4= 20/tan(angle) +7.5;
                    float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

                    int pot = (data.acc/255.0) * (1 << PWM_RESOLUTION);
                    uint32_t w1_pwm = (pot*r3)/rcg;
                    uint32_t w2_pwm = (pot*r4)/rcg;
        
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w2_pwm);
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w1_pwm);

                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);

                    sensor.w1 = w1_pwm;
                    sensor.w2 = w2_pwm;
                
                    printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
                }
            }
        }           
        if(data.ign==0)
        {
            //bnd kro motor
            vTaskDelay(1);
            gpio_set_level(start_indi, 0);
            gpio_set_level(left_indi, 0);
            gpio_set_level(right_indi, 0);
        }  
    }  
}


