#include "mServer.h"
#include "Queue.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "jpeglib.h"
#include "ili9341.h"

//RAM 空间不够，每解析 JPEG_Decode_Line 行后显示
#define JPEG_Decode_Line 10

int jpegDecode(void);

struct jpeg_decompress_struct cinfo;
struct jpeg_error_mgr jerr;
char data[320*10*3];		//320*240的一半
char jpeg_data[10240];
int jpg_point=0,jpeg_zt = 0;
char ap[64] = {1},server[64] = {1},IP_address[64] = "192.168.1.206";
int port = 8080;
TaskHandle_t Tcp_Transmit_handle = NULL;        //任务句柄
int connect_sock;        //给Client创建Socket使用 以及给Server bind 使用  给发送接收WIFI信息使用
int listen_sock;        //给Server模式创建Socket使用
extern char spi_ok;
static char *TAG="TCP_Server";
int tcp_server_connect=0;
char Wifi_Send_Data[1024];
extern LinkQueue Queue_Wifi_Transmit;
extern SemaphoreHandle_t Mutex_Wifi_Transmit;
void Tcp_Transmit(void *pvParameters){
	err_t ret;
    ESP_LOGI(TAG,"TCP发送任务开始");
    for(;;){
		if(xSemaphoreTake(Mutex_Wifi_Transmit,portMAX_DELAY) == pdTRUE){
			if(GetLength(Queue_Wifi_Transmit) != 0){
				if(GetHead(Queue_Wifi_Transmit,(uint8_t *)Wifi_Send_Data) == 1){
					ret = send(connect_sock,Wifi_Send_Data,1024,0);
					if(ret == ESP_OK){
						if(DeLinkQueue(&Queue_Wifi_Transmit)==1){

                        }
					}
				}
			}
			xSemaphoreGive(Mutex_Wifi_Transmit);
		}
        vTaskDelay(10 / portTICK_RATE_MS);
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
			for(int i =0;i<len;i++){
				jpeg_data[jpg_point] = rx_buffer[i];
				jpg_point = (jpg_point + 1)%10240;
				if(jpg_point == 2 || jpeg_zt == 1){
					//ESP_LOGW(TAG,"%d %d",jpeg_data[0],jpeg_data[1]);
					if(jpeg_data[0] == 0xff && jpeg_data[1] == 0xd8){
						jpeg_zt = 1;
						if(jpeg_data[jpg_point - 2] == 0xff && jpeg_data[jpg_point - 1] == 0xd9){
							jpegDecode();
							for(int j=0;j<=jpg_point;j++){
								jpeg_data[j] = 0;
							}
							jpg_point = 0;
							jpeg_zt = 0;
						}
					}else{
						jpg_point = 0;
					}
				}
			}
		}
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}
void mServer_Init(void *pvParameters){
    //int *pserver = (int *)pvParameters;
    int addr_family = 0;
    int ip_protocol = 0;
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    while(ip_flag==0){
        //printf("等待获取IP....\r\n");
        vTaskDelay(500);
    }
    ESP_LOGE(TAG,"server = %d\nip = %s\nport = %d",1,IP_address,port);
	connect_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
	if(connect_sock < 0)
	{
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		close(connect_sock);
	}
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(IP_address);
    while(1)
    {
		connect_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
		if(connect_sock < 0)
		{
			ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
			close(connect_sock);
		}
		while(connect(connect_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0){
			ESP_LOGE("连接IP","失败");
			vTaskDelay(500);
			close(connect_sock);
			connect_sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
			if(connect_sock < 0)
			{
				ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
				close(connect_sock);
			}
		}
		ESP_LOGI("CLIENT", "连接成功");
		Tcp_Receive();
		ESP_LOGE(TAG,"tcp接收任务终止");
		vTaskDelete(Tcp_Transmit_handle);
		ESP_LOGE(TAG,"tcp发送任务终止");
		close(connect_sock);
    }
}

int jpegDecode(void)
{
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, (void *)jpeg_data, sizeof(jpeg_data));
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);
    // printf("height%d\n",cinfo.output_height);
    // printf("width%d\n",cinfo.output_width);
    // printf("scanline%d\n",cinfo.output_scanline);
    // printf("width%d\n",cinfo.output_width);
	JSAMPARRAY out_buffer = (JSAMPARRAY)malloc(sizeof(JSAMPROW));
	out_buffer[0] = (JSAMPROW)malloc(sizeof(JSAMPLE) * 320 * 3);
	int j =0;
	while (cinfo.output_scanline < cinfo.output_height)
	{
		jpeg_read_scanlines(&cinfo, out_buffer, 1);				//空间不够，边解析边显示
		memcpy(&data[j], out_buffer[0], cinfo.image_width * 3);
		j += cinfo.image_width * 3;
		if((cinfo.output_scanline+1)%JPEG_Decode_Line==0){
			int k = 0;
			j = 0;
			for(int i = 0;i<JPEG_Decode_Line*320*3;i+=3){
				// char b = data[i]&0x1f;
				// char g = data[i+1]&0x3f;
				// char r = data[i+2]&0x1f;
				// data[j++] = r<<3 | g>>3;
				// data[j++] = g<<3 | b;
				uint16_t rgb =(((data[i+0] >> 3) & 0x1F) << 11)//b
								|(((data[i+1] >> 2) & 0x3F) << 5)//g
								|(((data[i+2] >> 3) & 0x1F) << 0);//r
				data[k++] = (unsigned char)((rgb>>8)&0xff);
				data[k++] = (unsigned char)((rgb)&0xff);
			}
			lcd_flush(0,((cinfo.output_scanline+1)/JPEG_Decode_Line-1)*JPEG_Decode_Line,320-1,(cinfo.output_scanline+1)-1,(uint8_t *)data);
			//ESP_LOGW("解码之后","%d %d %d %d",0,((cinfo.output_scanline+1)/10-1)*10,320-1,(cinfo.output_scanline+1)-1);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}
	//ESP_LOGW("解码之后","%d",esp_get_free_heap_size());
	free(out_buffer[0]);
	free(out_buffer);
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    //printf("\nfinsh\n");
    return 0;
}
