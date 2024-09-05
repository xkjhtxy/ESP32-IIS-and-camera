#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>
#define BOARD_WROVER_KIT 1
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "Queue.h"
#include "mWifi.h"
#include "mServer.h"
// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT
//合宙S3引脚
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET 45 //software reset will be performed
#define CAM_PIN_XCLK 39
#define CAM_PIN_SIOD 21
#define CAM_PIN_SIOC 46
#define CAM_PIN_D7 40
#define CAM_PIN_D6 38
#define CAM_PIN_D5 37
#define CAM_PIN_D4 35
#define CAM_PIN_D3 33
#define CAM_PIN_D2 48
#define CAM_PIN_D1 47
#define CAM_PIN_D0 34
#define CAM_PIN_VSYNC 42
#define CAM_PIN_HREF 41
#define CAM_PIN_PCLK 36

//esp-cam引脚
// #define CAM_PIN_PWDN 32
// #define CAM_PIN_RESET -1 //software reset will be performed
// #define CAM_PIN_XCLK 0
// #define CAM_PIN_SIOD 26
// #define CAM_PIN_SIOC 27

// #define CAM_PIN_D7 35
// #define CAM_PIN_D6 34
// #define CAM_PIN_D5 39
// #define CAM_PIN_D4 36
// #define CAM_PIN_D3 21
// #define CAM_PIN_D2 19
// #define CAM_PIN_D1 18
// #define CAM_PIN_D0 5
// #define CAM_PIN_VSYNC 25
// #define CAM_PIN_HREF 23
// #define CAM_PIN_PCLK 22


#endif
void play_i2s_init(void);
static esp_err_t init_camera();

static const char *TAG = "example:take_picture";
camera_fb_t *pic;       //相机数据指针
LinkQueue Queue_wifi_RX;
SemaphoreHandle_t Mutex_wifi_RX;
char Audio_Data[1024];
//int16_t Decode_Audio_Data[512];
size_t bytesOut = 0;
int rst_count = 0;
void WDT_Task(void *pvParameters){
    for(;;){
        if(rst_count++ > 20){
            esp_restart();
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
void app_main()
{
    if(ESP_OK != init_camera()) {
        return;
    }
    //Mutex_wifi_RX = xSemaphoreCreateMutex();
    //InitLinkQueue(&Queue_wifi_RX);
    mWifi_Init(Station);
    //play_i2s_init();
    xTaskCreate(mServer_Init,"tcp_receive_task",4096,NULL,2,NULL);      //创建Wifi接收任务
    xTaskCreate(WDT_Task,"wdt_task",4096,NULL,2,NULL);      //创建Wifi接收任务
    while (1)
    {
        if(xSemaphoreTake(Mutex_wifi_RX,portMAX_DELAY) == pdTRUE){
            if(GetLength(Queue_wifi_RX) != 0){
                memset(Audio_Data,0,1024);
                //memset(Decode_Audio_Data,0,512);
                if(GetHead(Queue_wifi_RX,(uint8_t *)Audio_Data) == 1){
                    err_t ret = i2s_write(I2S_NUM_0, (void *)Audio_Data, 1024, &bytesOut, portMAX_DELAY);
                    if(ret == ESP_OK){
                        if(DeLinkQueue(&Queue_wifi_RX)==1){

                        }
                    }else{
                        printf("IIS发送失败\r\n");
                    }
                }
            }
            xSemaphoreGive( Mutex_wifi_RX );
        }
        vTaskDelay(3 / portTICK_RATE_MS);
    }
}
void play_i2s_init(void)
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = (16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LOWMED,
        .dma_buf_count = 2,
        .dma_buf_len = 1024,
        .bits_per_chan = I2S_BITS_PER_CHAN_16BIT
        };
    i2s_pin_config_t pin_config = {
        .mck_io_num = -1,
        .bck_io_num = 18,   // IIS_SCLK
        .ws_io_num = 16,    // IIS_LCLK
        .data_out_num = 17, // IIS_DSIN
        .data_in_num = -1         // IIS_DOUT
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 10, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .fb_location = CAMERA_FB_IN_DRAM,
};
static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        esp_restart();
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 2);     // -2 to 2
    return ESP_OK;
}
