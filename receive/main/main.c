#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "lvgl/lvgl.h"			// LVGL头文件
#include "lvgl_helpers.h"		// 助手 硬件驱动相关
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "Queue.h"
#include "mWifi.h"
#include "mServer.h"
#include "jpeglib.h"



#define TAG " LittlevGL Demo"
#define LV_TICK_PERIOD_MS 10
#define I2S_WS 21
#define I2S_SD 22
#define I2S_SCK 27
#define I2S_PORT I2S_NUM_0

static void lv_tick_task(void *arg);
void guiTask(void *pvParameter);				// GUI任务

void UI_Init(void);
void I2S_Init(){
	    const i2s_config_t i2s_config = {
        .mode = (I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = (16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 2,
        .dma_buf_len = 1024,
        .use_apll = false
    };
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
	i2s_set_pin(I2S_PORT, &pin_config);
    i2s_start(I2S_PORT);
}

extern int connect_sock;			//发送Socket
size_t bytes_read;
char Audio_Data[1024];		//存储音频数据

LinkQueue Queue_Wifi_Transmit;
SemaphoreHandle_t Mutex_Wifi_Transmit;
// 主函数
void app_main() {
	ESP_LOGW("最开始","%d",esp_get_free_heap_size());
	lvgl_driver_init();
	ili9341_enable_backlight(1);
	mWifi_Init(Station);
    xTaskCreate(mServer_Init,"tcp_receive_task",4096,NULL,2,NULL);      //创建Wifi接收任务
	// // 如果要使用任务创建图形，则需要创建固定核心任务,否则可能会出现诸如内存损坏等问题
	// // 创建一个固定到其中一个核心的FreeRTOS任务，选择核心1
	//xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);		//运行lvgl会消耗 100KB RAM空间，这里为了JPEG解码就把LVGL停了
																				//启用LVGL应将 disp_spi.c 的 // lv_disp_flush_ready(&disp->driver);取消注释
	Mutex_Wifi_Transmit  = xSemaphoreCreateMutex();
    InitLinkQueue( &Queue_Wifi_Transmit);
	//i2s_adc_enable(I2S_NUM);
    ESP_LOGI(TAG,"开始");
	err_t ret;
	I2S_Init();
	ESP_LOGW("之后","%d",esp_get_free_heap_size());
    while(1){
		if(xSemaphoreTake(Mutex_Wifi_Transmit,portMAX_DELAY) == pdTRUE){
			if(GetLength(Queue_Wifi_Transmit) < 20){
				memset(Audio_Data,0,1024);
				ret = i2s_read(I2S_NUM_0, (void *)Audio_Data, 1024, &bytes_read, 100);
				if(ret == ESP_OK){
					if(EnLinkQueue(&Queue_Wifi_Transmit,(unsigned char *)Audio_Data) == 1){

                    }
				}
			}
			xSemaphoreGive(Mutex_Wifi_Transmit);
		}
		vTaskDelay(10 / portTICK_RATE_MS);
    }
	//i2s_adc_disable(I2S_NUM);
}
		//串口输出读取到的数据
		// for(int i= 0 ;i <I2S_READ_LEN;i+=2){
		// 	//ESP_LOGI(TAG,"%d",((Audio_Data[i+1]&0x0F)<<8) | Audio_Data[i]);
		// }
static void lv_tick_task(void *arg){
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}


SemaphoreHandle_t xGuiSemaphore;		// 创建一个GUI信号量
lv_img_dsc_t test = {
	.header.always_zero = 0,
	.header.w = 320,
	.header.h = 240,
	.data_size = 320*240 * LV_COLOR_SIZE / 8,
	.header.cf = LV_IMG_CF_TRUE_COLOR,
	.data = (unsigned char *)0x70000000,
		//.data = (uint8_t *)rgb,
};
void UI_Init(){
	//lv_demo_widgets();
	//jpg_test();
	// lv_obj_t * scr = lv_disp_get_scr_act(NULL);         // 获取当前屏幕
    // lv_obj_t * label1 =  lv_label_create(scr, NULL);    // 在当前活动的屏幕上创建标签
    // lv_label_set_text(label1, "Hello\nworld!");         // 修改标签的文字
    // lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
}
void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();    // 创建GUI信号量
    lv_init();          // 初始化LittlevGL
    lvgl_driver_init(); // 初始化液晶SPI驱动 触摸芯片SPI/IIC驱动

    static lv_color_t buf1[DISP_BUF_SIZE];
#ifndef CONFIG_LVGL_TFT_DISPLAY_MONOCHROME
    static lv_color_t buf2[DISP_BUF_SIZE];
#endif
    static lv_disp_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_IL3820
    /* Actual size in pixel, not bytes and use single buffer */
    size_in_px *= 8;
    lv_disp_buf_init(&disp_buf, buf1, NULL, size_in_px);
#elif defined CONFIG_LVGL_TFT_DISPLAY_MONOCHROME
    lv_disp_buf_init(&disp_buf, buf1, NULL, size_in_px);
#else
    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);
#endif

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

// 如果配置为 单色模式
#ifdef CONFIG_LVGL_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
#endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);


// 如果有配置触摸芯片，配置触摸
#if CONFIG_LVGL_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif


    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // 一个标签演示
    UI_Init();

    while (1) {
		vTaskDelay(10);
		// 尝试锁定信号量，如果成功，请调用lvgl的东西
		if (xSemaphoreTake(xGuiSemaphore, (TickType_t)10) == pdTRUE) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);  // 释放信号量
        }
    }
    vTaskDelete(NULL);      // 删除任务
}


