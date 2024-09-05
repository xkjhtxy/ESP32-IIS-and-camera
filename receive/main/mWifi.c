#include "mWifi.h"
//这个程序里面实现了 STA 和 AP 模式的转换
char SSID1[64] = "CMCC-x56c",PSWR[64] = "13695587503";
char mSSID[64] = "mssid",mPSWR[64] = "mypassword";

char *tag_ap = "WIFI_AP";
uint8_t ip_flag=0;
void mWifi_Init(uint8_t mode){
    init_nvs();

    esp_netif_init();

    esp_event_loop_create_default();
    if(mode == Station){
        esp_netif_create_default_wifi_sta();
    }else if(mode == SoftAp){
        esp_netif_create_default_wifi_ap();
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config_sta = {
        .sta = {
            .ssid = {(uint8_t *)SSID1},
            .password = {(uint8_t *)PSWR},
        },
    };
    sprintf((char*)wifi_config_sta.sta.ssid, SSID1);        //配置wifi名称
    sprintf((char*)wifi_config_sta.sta.password, PSWR);


    wifi_config_t wifi_config_ap = {
        .ap = {
            .ssid = {(uint8_t *)mSSID},
            .ssid_len = strlen(mSSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = {(uint8_t *)mPSWR},
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    sprintf((char*)wifi_config_ap.ap.ssid, mSSID);        //配置wifi名称
    sprintf((char*)wifi_config_ap.ap.password, mPSWR);
    ESP_LOGE("TEST_ESP32","%s   %s",SSID1,PSWR);
    ESP_LOGE("TEST_ESP32","%s   %s",mSSID,mPSWR);
    if(mode == Station){
        esp_wifi_set_config(WIFI_IF_STA , (wifi_config_t *) &wifi_config_sta);
        esp_wifi_set_mode(WIFI_MODE_STA);
    }else if(mode == SoftAp){
        esp_wifi_set_config(WIFI_IF_AP, (wifi_config_t *) &wifi_config_ap);
        esp_wifi_set_mode(WIFI_MODE_AP);
    }
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL);
    esp_wifi_start();
}
/**
 * @brief 用于初始化nvs
 */
void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

/**
 * @brief WiFi 的事件循环Handler
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        //printf("wifi断开连接\r\n");
    }
    if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("TEST_ESP32", "Got IP: " IPSTR,  IP2STR(&event->ip_info.ip));
        ip_flag = 1;
    }

    //ap模式下的
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    ESP_LOGI(tag_ap, "station "MACSTR" join, AID=%d",
                MAC2STR(event->mac), event->aid);
    ip_flag = 1;
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(tag_ap, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}


