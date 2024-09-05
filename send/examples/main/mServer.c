#include "mServer.h"
#include "Queue.h"
#include "esp_camera.h"
char IP_address[64] = "192.168.1.147";
int port = 8080;
TaskHandle_t Tcp_Transmit_handle = NULL;        //任务句柄
int connect_sock;        //给Client创建Socket使用 以及给Server bind 使用  给发送接收WIFI信息使用
int listen_sock;        //给Server模式创建Socket使用
static char *TAG="TCP_Server";
int tcp_server_connect=0;
extern LinkQueue Queue_wifi_RX;
extern SemaphoreHandle_t Mutex_wifi_RX;
extern camera_fb_t *pic;
extern int rst_count;
void Tcp_Transmit(void *pvParameters){
    ESP_LOGI(TAG,"TCP发送任务开始");
    err_t ret;
    for(;;){
        pic = esp_camera_fb_get();
        if(!pic)
        {
            ESP_LOGE(TAG, "Camera Init Failed");
        }
        else{
            rst_count = 0;
            ret =  send(connect_sock, pic->buf,pic->len, 0);
            //ESP_LOGI(TAG, "len = %d",pic->len);
            esp_camera_fb_return(pic);
        }
        vTaskDelay(3 / portTICK_RATE_MS);
    }

}
void Tcp_Receive(){
    xTaskCreate(Tcp_Transmit,"tcp_transmit_task",4096,NULL,2,&Tcp_Transmit_handle);  //创建WIFI发送任务
    ESP_LOGI(TAG,"TCP接收任务开始");
    int len;
    unsigned char rx_buffer[1024];
    while(1)
    {
        memset(rx_buffer, 0, sizeof(rx_buffer));    // 清空缓存
        len = recv(connect_sock, rx_buffer, 1024, 0);  // 阻塞读取接收数据
        if(len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            break;
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
            break;
        }
        else
        {
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
            if(xSemaphoreTake(Mutex_wifi_RX,portMAX_DELAY)==pdTRUE){
                if(GetLength(Queue_wifi_RX)<20){
                    if(EnLinkQueue(&Queue_wifi_RX,rx_buffer)==1){

                    }
                }
                xSemaphoreGive( Mutex_wifi_RX );
            }
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}
void mServer_Init(void *pvParameters){
    int addr_family = 0;
    int ip_protocol = 0;
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    while(ip_flag==0){
        printf("等待获取IP....\r\n");
        vTaskDelay(500);
    }
    printf("server = %d\nip = %s\nport = %d",0,IP_address,port);
    listen_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
    if(listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        //close(listen_sock);
    }
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if(err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        close(listen_sock);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", port);
    err = listen(listen_sock, 1);
    if(err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
    }

    while(1)
    {
        ESP_LOGI(TAG, "Socket listening");
        struct sockaddr_in6 source_addr;
        uint16_t addr_len = sizeof(source_addr);
        connect_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if(connect_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            close(listen_sock);
        }
        else{
            tcp_server_connect =1;
            ESP_LOGI("SERVICE", "连接成功");
            Tcp_Receive();
            ESP_LOGE(TAG,"tcp接收任务终止");
        }
        tcp_server_connect =0;
        vTaskDelete(Tcp_Transmit_handle);
        ESP_LOGE(TAG,"tcp发送任务终止");
        shutdown(connect_sock, 0);
        close(connect_sock);
    }
}
