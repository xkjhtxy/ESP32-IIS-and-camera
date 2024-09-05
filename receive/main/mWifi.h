#ifndef __mWifi_H
#define __mWifi_H
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"


#define CONFIG_ESP_WIFI_SSID "myssid"
#define CONFIG_ESP_WIFI_PASSWORD "mypassword"
#define CONFIG_ESP_WIFI_CHANNEL 1
#define CONFIG_ESP_MAX_STA_CONN 4

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

// #define Station 0x00
// #define SoftAp  0x01

enum{
    Station = 0x00,
    SoftAp,
};
enum{
    CLIENT = 0x00,
    SERVER,
};
void mWifi_Init(uint8_t mode);
void init_nvs();
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


extern uint8_t ip_flag;


#endif

