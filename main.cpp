


// //#include "Arduino.h"
// #include <stdlib.h>
// #include <time.h>
// #include <string.h>
// #include <assert.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "freertos/timers.h"
// #include "nvs_flash.h"
// #include "esp_random.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "esp_wifi.h"
// #include "esp_log.h"
// #include "esp_mac.h"
// #include "esp_now.h"
// #include "esp_crc.h"
// #include "espnow_example.h"

// uint8_t acc;

// static esp_err_t wifi_init(void)
// {
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_netif_init();
//     esp_event_loop_create_default();
//     nvs_flash_init();
//     esp_wifi_init(&cfg);
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_set_storage(WIFI_STORAGE_FLASH);
//     esp_wifi_start();
//     return ESP_OK;
// }

// // void data_recv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
// // {
// //     memcpy(&acc, data, data_len); //memcmp   
// // }

// // static esp_err_t esp_now_init(void)
// // {
// //     esp_now_register_recv_cb(data_recv);
// //     return ESP_OK;
// // }

// void app_main()
// {
//     ESP_ERROR_CHECK(wifi_init());
//     //ESP_ERROR_CHECK(esp_now_init());
// }








// extern "C" 
// {
//     #include <stdio.h>

//     void app_main(void)
//     {

//         Serial.begin(115200);
//         WiFi.mode(WIFI_STA);
//         Serial.println(WiFi.getMode());
 
//         while(1)
//         {
//             Serial.println("xsc");
//             Serial.println(WiFi.getMode());
//             delay(100);
            
//         }
//     }
// }



// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>
//     void app_main(void)
//     {
//         uint64_t pipe = 0xE8E8F0F0E1LL;

//         int data;

//         RF24 radio(4,5); // CE, CSN
//         //pinMode(2,OUTPUT);
//         //digitalWrite(2,HIGH);

//         radio.begin();
//         radio.openReadingPipe(0, pipe);
//         radio.setPALevel(RF24_PA_LOW);
//         radio.setDataRate(RF24_250KBPS);
//         //radio.startListening();
      
         
//         //Serial.begin(115200);
//         //printf_begin();

//         //radio.printDetails();
//         //Serial.println(radio.isChipConnected());
//         radio.startListening();
//         while(1)
//         {
            
//             //radio.available();
            



//             //printf("\n Started\n ");
//             //bool report = radio.available();
//             //Serial.println(report);
//             //while(report)
//             //{
//               //  printf("\n inside\n ");
//                 //delay(1000);
//             //}
//             radio.read(&data, sizeof(data));
//             printf("%d", data);
//             printf("\n clr\n");    
//             //radio.printPrettyDetails(); 
//             delay(100);   
//         }
//     }
// }


// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>
//     void app_main(void)
//     {
//         const byte address[6] = "00001";

//         RF24 radio(4,5,10000000UL); // CE, CSN
//         //pinMode(2,OUTPUT);
//         //digitalWrite(2,HIGH);
//         radio.begin();
//         radio.openWritingPipe(address);
//         radio.setPALevel(RF24_PA_MIN);
//         radio.stopListening();


//         // radio.begin();
//         // radio.openReadingPipe(0, address);
//         // radio.setPALevel(RF24_PA_MIN);
//         // radio.startListening();
        
         
//         Serial.begin(115200);
//         printf_begin();

//         radio.printDetails();
//         Serial.println(radio.isChipConnected());
//         while(1)
//         {
//             const char text[] = "Hello World";
//             radio.write(&text, sizeof(text));
//             delay(1000);
//             // char text[32];
//             // if(radio.available())
//             // {
//             //     radio.read(&text, sizeof(text));
//             //     printf("%s", text);
//             // }
//             // delay(1);
//         }
// }
// }

// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>
//     void app_main(void)
//     {
//         //const byte address[6] = "00001";

//         RF24 radio(4,5,10000000UL); // CE, CSN
//         //pinMode(2,OUTPUT);
//         //digitalWrite(2,HIGH);

//         radio.begin();
//         radio.openReadingPipe(0, 00001);
//         radio.setPALevel(RF24_PA_MIN);
//         radio.startListening();
        
         
//         Serial.begin(115200);
//         printf_begin();

//         radio.printDetails();
//         Serial.println(radio.isChipConnected());
//         while(1)
//         {
//             printf("\n Started\n ");
//             char text[32];
//             if(radio.available())
//             {
//                 printf("\n inside\n ");
//                 radio.read(&text, sizeof(text));
//                 printf("%s", text);
//             }
//             delay(100);
//             printf("\n clr\n");
            
//         }
// }
// }


// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>
//     void app_main(void)
//     {
//         byte addresses[][6] = {"1Node", "2Node"};
//         RF24 radio(4,5,10000000UL); // CE, CSN
//         pinMode(2,OUTPUT);
//         digitalWrite(2,HIGH);
//         radio.begin();
//         radio.setPALevel(RF24_PA_LOW);
  
//         radio.openWritingPipe(addresses[0]);
//         radio.openReadingPipe(1, addresses[1]); 
//         radio.startListening();
  
//         Serial.begin(115200);
//         printf_begin();

//         radio.printDetails();
//         Serial.println(radio.isChipConnected());
//         while(1)
//         {
//             delay(100);
//             // if (radio.available()) 
//             // {
//             //     char text[32] = "";
//             //     radio.read(&text, sizeof(text));
//             //     Serial.println(text);
//             //     delay(1000);
//             // }
//         }
// }
// }


// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>
//     void app_main(void)
//     {
//         const byte address[6] = "00001";

//         RF24 radio(32, 33); // CE, CSN
//         pinMode(2,OUTPUT);
//         Serial.begin(115200);
//         radio.begin();
//         radio.openReadingPipe(0, address);
//         radio.setPALevel(RF24_PA_MIN);
//         radio.startListening();         
//         while(1)
//         {
//             if (radio.available()) 
//             {
//                 char text[32] = "";
//                 radio.read(&text, sizeof(text));
//                 Serial.println(text);
//                 delay(1000);
//             }
//         }
// }
// }


// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>

//     void app_main(void)
//     {
//         pinMode(2, OUTPUT);
//         pinMode(5, OUTPUT);
//         pinMode(18, OUTPUT);
 
//         while(1)
//         {
//             //digitalWrite(2,HIGH);
//             delay(500);
//             //digitalWrite(2,LOW);
//             delay(500);
//         }
//         }
// }



// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>


// extern "C" 
// {
//     #include <stdio.h>

//     void app_main(void)
//     {
//         RF24 radio(32, 33); // CE, CSN
//         pinMode(2,OUTPUT);
//         pinMode(25,INPUT);
//         radio.begin();
//         const byte address[6] = "00001";
//         radio.openWritingPipe(address);
//         radio.setPALevel(RF24_PA_MIN);
//         radio.stopListening();

         
//         while(1)
//         {
//             int value = digitalRead(25);
//             digitalWrite(2,HIGH);
//             delay(500);            
//             radio.write(&value, sizeof(value));
//             digitalWrite(2,LOW);
//             //printf("\n %d\n",value );
//             delay(500);
//         }
//         }
// }


// #include "Arduino.h"
// #include "printf.h"

// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <RF24_config.h>

// extern "C"
// {
// void blink(void)
//     {
//         pinMode(2,OUTPUT); 
//         int i=0;
//         while(i<5)
//         {
//             digitalWrite(2, HIGH);
//             delay(500);
//             digitalWrite(2, LOW);
//             delay(100);
//             i++;
//         }
//         }
// }

// extern "C"
// {
// void app_main(void)
// {
//     RF24 radio(7, 8); // CE, CSN
//     radio.begin();
//     blink();
// }
// }




// 
//         const byte address[6] = "00001";
//         
//         radio.openWritingPipe(address);
//         radio.setPALevel(RF24_PA_MIN);
//         radio.stopListening();

//         const char text[] = "Hello World";

//         radio.write(&text, sizeof(text));
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
// #include "driver/spi_slave.h"
// #include "driver/gpio.h"

// #define GPIO_CS 15
// #define LED_PIN GPIO_NUM_2  // Change this to the GPIO pin connected to the LED

// void app_main(void)
// {
//     // Configuration for the SPI bus
//     spi_bus_config_t buscfg = {
//         .miso_io_num = -1,
//         .mosi_io_num = -1,
//         .sclk_io_num = -1,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 32
//     };

//     // Configuration for the SPI slave interface
//     spi_slave_interface_config_t slvcfg = {
//         .mode = 0,
//         .spics_io_num = GPIO_CS,
//         .queue_size = 3,
//         .flags = 0
//     };

//     // Initialize SPI slave interface
//     spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

//     gpio_pad_select_gpio(LED_PIN);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     uint16_t potValue;

//     while (1) {
//         spi_slave_transaction_t t;
//         memset(&t, 0, sizeof(t));

//         t.length = 16; // Length in bits (uint16_t)
//         t.rx_buffer = &potValue;

//         // Receive potentiometer value from sender over SPI
//         spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);

//         // Control LED brightness based on potentiometer value
//         int brightness = map(potValue, 0, 1023, 0, 255); // Map pot value to LED brightness (0-255)
//         printf("Potentiometer Value: %d, Brightness: %d\n", potValue, brightness);
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
//         ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

//         vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
//     }
// }


// #include <stdio.h>
// #include <stdint.h>
// #include <stddef.h>
// #include <string.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_log.h"
// #include "driver/spi_slave.h"
// #include "driver/gpio.h"


// #define GPIO_MOSI 13
// #define GPIO_MISO 12
// #define GPIO_SCLK 14
// #define GPIO_CS 15


// //#define RCV_HOST    VSPI_HOST
// #define RCV_HOST    HSPI_HOST
// //#define RCV_HOST    SPI2_HOST


// void app_main(void)
// {

//     int n = 0;
   
//     //Configuration for the SPI bus
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = GPIO_MOSI,
//         .miso_io_num = GPIO_MISO,
//         .sclk_io_num = GPIO_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };


//     //Configuration for the SPI slave interface
//     spi_slave_interface_config_t slvcfg = {
//         .mode = 0,
//         .spics_io_num = GPIO_CS,
//         .queue_size = 3,
//         .flags = 0,
//     };

//     spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO); 
    
//     WORD_ALIGNED_ATTR char recvbuf[129] = "";
//     //memset(recvbuf, 0, 33);
//     spi_slave_transaction_t t;
//     //memset(&t, 0, sizeof(t));

//     while (1) {
        
//         //memset(recvbuf, 0xA5, 129);
        
//         t.length = 128 * 8;
//         t.rx_buffer = &recvbuf;

//         spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

//         printf("\n%d\n",n);
    
//         printf("\nReceived: %s\n", recvbuf);
//         n++;
//     }

// }

// #include <stdio.h>
// #include <stdint.h>
// #include <stddef.h>
// #include <string.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "driver/spi_slave.h"
// #include "driver/gpio.h"

// #define GPIO_MOSI 13
// #define GPIO_SCLK 14
// #define GPIO_CS 15

// //Main application
// void app_main(void)
// {
//     //Configuration for the SPI bus
//     spi_bus_config_t buscfg={
//         .mosi_io_num=GPIO_MOSI,
//         .miso_io_num=-1,
//         .sclk_io_num=GPIO_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 32
//     };

//     //Configuration for the SPI slave interface
//     spi_slave_interface_config_t slvcfg={
//         .mode=0,
//         .spics_io_num=GPIO_CS,
//         .queue_size=3,
//         .flags=0
//     };

//     //Initialize SPI slave interface
//     spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

//     uint16_t recvbuf[8];
//     memset(recvbuf, 0, 8);
//     spi_slave_transaction_t t;
//     memset(&t, 0, sizeof(t));
      

// 	printf("Slave output:\n");
//     while(1) {
        
//         t.length=8*2;
//         t.rx_buffer = recvbuf;
//         spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
        
//         printf("\nReceived: %s \n", recvbuf);
//         vTaskDelay(1000);
//     }
// }
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "math.h"
//#include "driver/spi_master.h"
#include "freertos/timers.h"


#define acc ADC1_CHANNEL_6 //34
#define steer ADC1_CHANNEL_7 //35
#define m1 GPIO_NUM_2
#define m2 GPIO_NUM_4 
#define CH0 LEDC_CHANNEL_0
#define CH1 LEDC_CHANNEL_1
#define PWM_FREQ 5000 // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_13_BIT // PWM resolution
#define encoder  GPIO_NUM_27 //rising edge interupt pin 

volatile long pulse = 0;

void IRAM_ATTR Pulsecounter() 
{
    pulse++;
}

void gpio_interrupt_init() 
{
    
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE; // rising edge
    io_conf.pin_bit_mask = (1ULL << encoder);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    //flag reset
    gpio_install_isr_service(0);

    // Hook ISR handler for the GPIO pin
    gpio_isr_handler_add(encoder, Pulsecounter, NULL);
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
    int x = adc1_get_raw(steer);
    int y;
    x = (x*30)/4095;
    y = 15 - x ;
    return y;
}

void rpm(TimerHandle_t xTimer) 
{
    int rpm = (pulse*60)/11;
    printf("Pulse %ld \t RPM %d", pulse, rpm);
    pulse =0;

}

void app_main() 
{
    gpio_interrupt_init();

    //timer initialized
    TimerHandle_t timer= xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, rpm);
    //start timer with 0 delay 
    xTimerStart(timer, 0);

    
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
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(acc, ADC_ATTEN_DB_11); 
    adc1_config_channel_atten(steer, ADC_ATTEN_DB_11);

    initialize_pwm();

    while(1) {

        int ang = RDsteer();
        vTaskDelay(1);
        if(ang==0)   
        {
            int pot_value = adc1_get_raw(acc);
            uint32_t duty_cycle = (pot_value / 4095.0) * (1 << PWM_RESOLUTION);

            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, duty_cycle);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, duty_cycle);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n", duty_cycle, duty_cycle, ang);
        }
        if(ang<0)// left turn 
        {
            float angle = (-1*ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

            int pot = (adc1_get_raw(acc)/4095.0) * (1 << PWM_RESOLUTION);
            uint32_t w1_pwm = (pot*r3)/rcg;
            uint32_t w2_pwm = (pot*r4)/rcg;
        
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w1_pwm);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w2_pwm);

            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
        }

        if(ang>0)// right turn
        {
            float angle = (1*ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

            int pot = (adc1_get_raw(acc)/4095.0) * (1 << PWM_RESOLUTION);
            uint32_t w1_pwm = (pot*r3)/rcg;
            uint32_t w2_pwm = (pot*r4)/rcg;
        
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w2_pwm);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w1_pwm);

            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
        }
    }
}
*/
/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define ENCODER_PIN1    GPIO_NUM_4   // Replace XX with the GPIO pin connected to encoder signal 1
#define ENCODER_PIN2    GPIO_NUM_39   // Replace XX with the GPIO pin connected to encoder signal 2

#define PULSES_PER_REVOLUTION  11      // Number of pulses per revolution
#define TIME_INTERVAL          1       // Time interval for counting pulses (in seconds)

// Global variables
volatile uint32_t pulse_count = 0;

// Interrupt service routine for handling encoder pulses
void IRAM_ATTR encoder_isr_handler(void* arg) {
    pulse_count++;
}

// Function to initialize GPIO pins for encoder signals
void init_encoder_pins() {
    gpio_config_t io_conf;
    // Configure ENCODER_PIN1 as input
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << ENCODER_PIN1);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);
    // Install ISR service
    gpio_install_isr_service(0);
    // Hook ISR handler for ENCODER_PIN1
    gpio_isr_handler_add(ENCODER_PIN1, encoder_isr_handler, (void*) ENCODER_PIN1);
}

// Function to count pulses within a time interval
void count_pulses(void* arg) {
    vTaskDelay(TIME_INTERVAL * 1000 / portTICK_PERIOD_MS); // Wait for specified time interval
    printf("Pulse count: %lu\n", pulse_count);
    pulse_count = 0; // Reset pulse count
}

// Function to convert pulse count to RPM
float calculate_rpm(uint32_t pulse_count) {
    float rotations = (float)pulse_count / PULSES_PER_REVOLUTION;
    float rpm = (rotations / TIME_INTERVAL) * 60;
    return rpm;
}

void app_main() {
    // Initialize encoder pins
    init_encoder_pins();

    // Create task to count pulses
    xTaskCreate(count_pulses, "count_pulses", 4096, NULL, 5, NULL);

    // Main loop
    while(1) {
        // Calculate and print RPM
        float rpm = calculate_rpm(pulse_count);
        printf("RPM: %.2f\n", rpm);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second
    }
}
*/
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/adc.h"


void initialize_adc() {
    // Configure ADC channels
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
}

void read_adc_values() {
    while(1) 
    {
        uint32_t adc_value1 = adc1_get_raw(ADC1_CHANNEL_0);
        uint32_t adc_value2 = adc1_get_raw(ADC1_CHANNEL_3);
        printf("ADC Value 1: %lu\t ADC Value 2: %lu\n", adc_value1, adc_value2);
    }
}

void app_main() {
    // Initialize ADC
    initialize_adc();
    
    // Create task to read ADC values
    xTaskCreate(read_adc_values, "read_adc_values", 4096, NULL, 5, NULL);
}
*/

/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define CE_PIN   GPIO_NUM_5   // GPIO pin connected to NRF24 module's CE pin
#define CSN_PIN  GPIO_NUM_17  // GPIO pin connected to NRF24 module's CSN pin

static const char *TAG = "NRF24_ESP32";

spi_device_handle_t spi;

void setup_spi() {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1, // MISO not used
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_18, 
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 0,
        .clock_speed_hz = 1000000,  // 1 MHz clock frequency
        .input_delay_ns = 0,
        .spics_io_num = CSN_PIN,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);
}

void send_command(uint8_t command) {
    esp_err_t ret;
    spi_transaction_t t;
    // Zero out the transaction structure
    for (int i = 0; i < sizeof(t); ++i) {
        ((uint8_t *)&t)[i] = 0;
    }
    t.length = 8;           // Command is 8 bits
    t.tx_buffer = &command;
    t.user = (void *)0;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void setup() {
    ESP_LOGI(TAG, "Initializing SPI");
    setup_spi();
}

void loop() {
    // Start listening for incoming data
    gpio_set_level(CE_PIN, 1);

    // Send dummy command to receive data
    send_command(0xFF);

    // Read data from NRF24 module
    uint8_t received_data;
    send_command(0xFF);  // Dummy byte to trigger SPI transfer
    spi_transaction_t t;
    // Zero out the transaction structure
    for (int i = 0; i < sizeof(t); ++i) {
        ((uint8_t *)&t)[i] = 0;
    }
    t.length = 8; // Data is 8 bits
    t.rxlength = 8;
    t.rx_buffer = &received_data;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);

    // Stop listening
    gpio_set_level(CE_PIN, 0);

    // Check if valid data received
    if (received_data != 0xFF) {
        ESP_LOGI(TAG, "Received: %d", received_data);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust as needed
}

void app_main() {
    setup();
    while (1) {
        loop();
    }
}
*/
/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "math.h"
#include "driver/spi_master.h"


#define acc ADC1_CHANNEL_6 //34
#define steer ADC1_CHANNEL_7 //35
#define m1 GPIO_NUM_2
#define m2 GPIO_NUM_4 
#define CH0 LEDC_CHANNEL_0
#define CH1 LEDC_CHANNEL_1
#define PWM_FREQ 5000 // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_13_BIT // PWM resolution

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
    int x = adc1_get_raw(steer);
    int y;
    x = (x*30)/4095;
    y = 15 - x ;
    return y;
}

void app_main() 
{

        // Set GPIO pin 2 (m1) as output and configure as pull-up
    gpio_config_t io_conf_m1;
    io_conf_m1.intr_type = GPIO_INTR_DISABLE;
    io_conf_m1.mode = GPIO_MODE_OUTPUT;
    io_conf_m1.pin_bit_mask = (1ULL << m1);
    io_conf_m1.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf_m1.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_m1);

    // Set GPIO pin 4 (m2) as output and configure as pull-up
    gpio_config_t io_conf_m2;
    io_conf_m2.intr_type = GPIO_INTR_DISABLE;
    io_conf_m2.mode = GPIO_MODE_OUTPUT;
    io_conf_m2.pin_bit_mask = (1ULL << m2);
    io_conf_m2.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf_m2.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_m2);
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(acc, ADC_ATTEN_DB_11); 
    adc1_config_channel_atten(steer, ADC_ATTEN_DB_11);

    initialize_pwm();

    while(1) {

        int ang = RDsteer();

        if(ang==0)   
        {
            int pot_value = adc1_get_raw(acc);
            uint32_t duty_cycle = (pot_value / 4095.0) * (1 << PWM_RESOLUTION);

            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, duty_cycle);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, duty_cycle);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n", duty_cycle, duty_cycle, ang);
            vTaskDelay(100);
        }
        if(ang<0)// left turn 
        {
            float angle = (-1*ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

            int pot = (adc1_get_raw(acc)/4095.0) * (1 << PWM_RESOLUTION);
            uint32_t w1_pwm = (pot*r3)/rcg;
            uint32_t w2_pwm = (pot*r4)/rcg;
        
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w1_pwm);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w2_pwm);

            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
            vTaskDelay(100);
        }

        if(ang>0)// right turn
        {
            float angle = (1*ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);

            int pot = (adc1_get_raw(acc)/4095.0) * (1 << PWM_RESOLUTION);
            uint32_t w1_pwm = (pot*r3)/rcg;
            uint32_t w2_pwm = (pot*r4)/rcg;
        
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH0, w2_pwm);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, CH1, w1_pwm);

            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, CH1);
            printf("w1: %lu, w2: %lu, ang: %d\n",w1_pwm,w2_pwm, ang);
            vTaskDelay(100);
        }  
    }
}
*/
/* }
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#define POTENTIOMETER_PIN 34
#define PWM_PIN 12
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_FREQ 5000 // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_13_BIT // PWM resolution

void initialize_pwm() {
    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = PWM_FREQ; //5khz
    timer_conf.duty_resolution = PWM_RESOLUTION;// 13 bit
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf;
    channel_conf.gpio_num = PWM_PIN; // gpio12
    channel_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_conf.channel = PWM_CHANNEL; //ch0
    channel_conf.intr_type = LEDC_INTR_DISABLE;
    channel_conf.timer_sel = LEDC_TIMER_0;
    channel_conf.duty = 0;
    channel_conf.hpoint = 0;
    ledc_channel_config(&channel_conf);
}

void app_main() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // Configure ADC to read from GPIO pin 34 with 11dB attenuation

    initialize_pwm();

    while(1) {
        int pot_value = adc1_get_raw(ADC1_CHANNEL_6); // Read raw ADC value from potentiometer
        uint32_t duty_cycle = (pot_value / 4095.0) * (1 << PWM_RESOLUTION); // Scale potentiometer value to duty cycle
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, duty_cycle); //13 bit  
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay according to your requirements
    }
}
*/


/*#include <stdio.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"


#define i2 GPIO_NUM_32
#define i4 GPIO_NUM_33
#define acc ADC1_CHANNEL_0 //36
#define ea DAC_CHANNEL_1 //25
#define eb DAC_CHANNEL_2 //26

#define ang 0.0f 

void gpio_init() {
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = ((1ULL<<i2) | (1ULL<<i4));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

void adc_config()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(acc, ADC_ATTEN_DB_11);
}



void app_main()
{
    //gpio
    gpio_init();
    gpio_set_level(i2|i4,0);

    //adc pot val
    adc_config();

    //dac i3 i4
    dac_output_enable(ea);
    dac_output_enable(eb);
    //
    dac_output_voltage(ea, 0);
    dac_output_voltage(eb, 0);

    while(1)
    {
        if(ang==0)
        {
            float w1 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            float w2 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            dac_output_voltage(ea,w1);
            dac_output_voltage(eb,w2);
            printf("%f\n",(adc1_get_raw(acc)/4095.0f)*255.0f);
            
            vTaskDelay(200);
        }
        if(ang<0)// left turn 
        {
            float angle = (-1*ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);
            float w1 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            w1 = (w1*r3)/rcg;
            float w2 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            w2 = (w2*r4)/rcg;
            dac_output_voltage(ea,w1);
            dac_output_voltage(eb,w2);
            printf("%f\n",(adc1_get_raw(acc)/4095.0f)*255.0f);
            vTaskDelay(200);
        }

        if(ang>0)// right turn
        {
            float angle = (ang/180.0f)*M_PI;
            float r3= 20/tan(angle) -7.5;
            float r4= 20/tan(angle) +7.5;
            float rcg= sqrt(((r3+7.5)*(r3+7.5))+100);
            float w1 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            w1 = (w1*r3)/rcg;
            float w2 = (adc1_get_raw(acc)/4095.0f)*255.0f;
            w2 = (w2*r4)/rcg;
            dac_output_voltage(ea,w2);
            dac_output_voltage(eb,w1);
            printf("%f\n",(adc1_get_raw(acc)/4095.0f)*255.0f);
            vTaskDelay(200);
        }                
    }
}



*/




/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac.h"

void app_main()
{
    // Initialize DAC
    dac_output_enable(DAC_CHANNEL_1); // Use DAC Channel 1
    dac_output_voltage(DAC_CHANNEL_1, 255);
    
        //vTaskDelay(100);
    
}
*/

/*#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"

void app_main() {
    // ADC Initialization 
    adc1_config_width(ADC_WIDTH_BIT_12); //4096 resolution
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //11 decible for 3.3 volt
    uint16_t adc_reading = 0;

    char buf[50];

    while (1) 
    {
        adc_reading = adc1_get_raw(ADC1_CHANNEL_0);// gpio 36
        sprintf(buf, "%d", adc_reading);
        printf("val: %s\n", buf);
        vTaskDelay(pdMS_TO_TICKS(1000));// To avoid Watchdog
    }
}

*/

/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac.h"

#define DAC_CHANNEL DAC_CHANNEL_1 //gpio 25
void app_main()
{
    // Initialization
    dac_output_enable(DAC_CHANNEL_1);
    while (1) 
    {
        //Loop
        for (int i = 0; i <= 255; i++) 
        {
            dac_output_voltage(DAC_CHANNEL_1, i);//8 bit DAC
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // Reset to zero volts
        dac_output_voltage(DAC_CHANNEL_1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

*/

/*#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/dac.h"

#define LED_PIN GPIO_NUM_25
#define DAC_CHANNEL DAC_CHANNEL_1

void app_main() {
    // Configure GPIO pin for the LED as an output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // Enable DAC output
    dac_output_enable(DAC_CHANNEL);

    while (1) {
        // Set DAC output to half of its maximum value
        uint8_t half_voltage = 129;  // 8-bit resolution, half of 255
        dac_output_voltage(DAC_CHANNEL, half_voltage);

        // Control LED based on the DAC output
        gpio_set_level(LED_PIN, half_voltage > 128 ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second (adjust as needed)
    }
}
*/

/* #include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "esp_err.h"

#define Analog_PIN GPIO_NUM_25
#define DAC_CHANNEL DAC_CHANNEL_1// gpio 25 


void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_25),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
dac_output_enable(DAC_CHANNEL);
uint8_t dac_value = 127;
    while (1)
    {
        dac_output_voltage(DAC_CHANNEL,dac_value);

        // Turn on the LED
        gpio_set_level(GPIO_NUM_25, dac_value > 128 ? 1 : 0);
        //gpio_set_level(GPIO_NUM_25, dac_value);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Turn off the LED
        //gpio_set_level(LED_PIN, 0);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
*/