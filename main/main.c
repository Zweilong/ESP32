/*
@ link : http://wit-motion.cn

@ Function:
1. Power on automatic detection sensor
2. Read acceleration, angular velocity, angle and magnetic field data

     ESP32              		JY901s
       +5        ----        	 VCC
       GPIO5	 ----		 	 SDA
       GPIO4	 ----		 	 SCL
       GND       ----       	 GND
------------------------------------
*/

#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include <sys/socket.h>     // for socket(), sendto(), sockaddr
#include <netinet/in.h>     // for sockaddr_in, htons(), IPPROTO_IP
#include <arpa/inet.h>      // for inet_addr()
#include <errno.h>          // for errno
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define UDP_TARGET_IP   "192.168.137.1"  // TODO: 改成上位机 IP
#define UDP_TARGET_PORT 12345
#define UDP_LISTEN_PORT 9999

static const char *TAG = "main";
bool collet_data_start_flag = false;
// ——— WiFi/网络部分 ———
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

#define BUF_SIZE 1024

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
float fAcc[3], fGyro[3], fAngle[3];
float fVelocity[3] = {0.0f, 0.0f, 0.0f};
static volatile char s_cDataUpdate = 0;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static void CmdProcess(char);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void CopeCmdData(unsigned char ucData);

static void wifi_init_sta();

static int i2c_master_port = 0;

static void wifi_event_handler(void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected. Retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP address.");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void wifi_init_sta(void)
{
    esp_netif_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst1, inst2;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &wifi_event_handler, NULL, &inst1));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &wifi_event_handler, NULL, &inst2));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "zwlccc99",
            .password = "bibuliaoye",
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // 可选项
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // 等待连接
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP");
    } else {
        ESP_LOGE(TAG, "Failed to connect to AP");
    }
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 5,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 4,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static int32_t WitIICRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    int ret;
    int i;

    if(uiLen == 0)
    	return 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ucReg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr | 0x01, ACK_CHECK_EN);
    for(i=0; i<uiLen; i++)
    {
    	if(i == uiLen-1)	// last pack
    		i2c_master_read_byte(cmd, p_ucVal+i, NACK_VAL);
    	else
    		i2c_master_read_byte(cmd, p_ucVal+i, ACK_VAL);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static int32_t WitIICWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
{
    int ret;
    int i;

    if(uiLen == 0)
    	return 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ucAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ucReg, ACK_CHECK_EN);
    for(i=0; i<uiLen; i++)
    {
    	if(i == uiLen-1)	// last pack
    		i2c_master_read_byte(cmd, p_ucVal+i, NACK_VAL);
    	else
    		i2c_master_read_byte(cmd, p_ucVal+i, ACK_VAL);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void Usart0_task(void *pvParameters)
{
	unsigned char c;

	while(1)
	{
		if(scanf("%c", &c) != -1)
		{
			CopeCmdData(c);
		}
		else
		{
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	}
}

static void readSensorDate_Task(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		WitReadReg(AX, 12);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

static void save_data_to_file_Task()
{
	FILE* f_clean = fopen("/spiffs/data.txt", "w");
    if (f_clean) {
        fclose(f_clean);
        printf("Data file cleared on boot.\n");
    } else {
        printf("Failed to clear data file.\n");
    }

	while(1)
	{
		FILE* f = fopen("/spiffs/data.txt", "a");
		if(f){
			fprintf(f, "acc: %.3f, %.3f, %.3f\n", fAcc[0], fAcc[1], fAcc[2]);
			fclose(f);
		}
		else {
            printf("Failed to open file for writing\n");
        }
		vTaskDelay(pdMS_TO_TICKS(100));
		
	}
}

// ——— UDP 发送任务 ———
static void udp_send_Task(void *pvParameters)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_TARGET_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_TARGET_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", UDP_TARGET_IP, UDP_TARGET_PORT);

    char buf[512];
    while (1) {
        // 构造要发送的数据 — 这里示例发送加速度
        int len = snprintf(buf, sizeof(buf),
                           "ACC:%.3f,%.3f,%.3f\r\n",
                           fAcc[0], fAcc[1], fAcc[2]);
        if (len > 0) {
            int err = sendto(sock, buf, len, 0,
                             (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300)); // 每 100 ms 发送一次
    }
}

// ——— UDP 接收任务 ———
static void udp_receive_Task(void *pvParameters)
{
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("UDP_RX", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in listen_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_LISTEN_PORT),
    };

    if (bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE("UDP_RX", "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("UDP_RX", "Listening on UDP port %d", UDP_LISTEN_PORT);

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            rx_buffer[len] = '\0';  // Null-terminate received data
            ESP_LOGI("UDP_RX", "Received: %s", rx_buffer);
            rx_buffer[strcspn(rx_buffer, "\r\n")] = 0;
        
		if (strcmp(rx_buffer, "start") == 0) {
		collet_data_start_flag = true;
        ESP_LOGI("UDP_RX", "Received start command, set flag to true");
		}

        }
    }

    // 从不应该到这里，但如果退出循环就清理
    close(sock);
    vTaskDelete(NULL);
}
void app_main(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());
	wifi_init_sta();
	
	esp_vfs_spiffs_conf_t conf = {
	.base_path = "/spiffs",
	.partition_label = NULL,
	.max_files = 5,
	.format_if_mount_failed = true
	};

	esp_vfs_spiffs_register(&conf);

	int i;
	static int64_t lastTime = 0;

	// xTaskCreate(Usart0_task, "Usart0_task", 4096, NULL, 5, NULL);
	xTaskCreate(readSensorDate_Task, "readSensorDate_Task", 4096, NULL, 5, NULL);
	xTaskCreate(udp_send_Task, "udp_send_Task", 8192, NULL, 5, NULL);
	xTaskCreate(udp_receive_Task, "udp_receive_Task", 4096, NULL, 5, NULL);
	// xTaskCreate(save_data_to_file, "save_data_to_file_Task", 4096, NULL, 5, NULL);
	i2c_master_init();

	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	WitI2cFuncRegister(WitIICWrite, WitIICRead);
	WitDelayMsRegister(Delayms);
	printf("\r\n********************** wit-motion IIC example  ************************\r\n");
	AutoScanSensor();
	ESP_LOGI(TAG, "Tasks created.");
	while (1)
	{
            // printf("start data collection...\n");
            vTaskDelay(pdMS_TO_TICKS(300)); //延时10ms
            if(s_cDataUpdate)
            {
                // int64_t currentTime = esp_timer_get_time(); // 获取当前时间（微秒）
                // float dt = (currentTime - lastTime) / 1000000.0f; // 转换为秒
                // lastTime = currentTime;
                for(i = 0; i < 3; i++)
                {
                    fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
                    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
                    fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
                }
                // /*计算速度*/ 
                if(collet_data_start_flag){
                    Vel_data_caculate();
                }
                /*打印数据*/
                if(s_cDataUpdate & ACC_UPDATE)
                {
                    printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
                    printf("vel:%.3f %.3f %.3f\r\n", fVelocity[0], fVelocity[1], fVelocity[2]);
                    s_cDataUpdate &= ~ACC_UPDATE;
                }
                if(s_cDataUpdate & GYRO_UPDATE)
                {
                    printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
                    s_cDataUpdate &= ~GYRO_UPDATE;
                }
                if(s_cDataUpdate & ANGLE_UPDATE)
                {
                    printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
                    s_cDataUpdate &= ~ANGLE_UPDATE;
                }
                if(s_cDataUpdate & MAG_UPDATE)
                {
                    printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
                    s_cDataUpdate &= ~MAG_UPDATE;
                }
            }
	}
}


void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;

	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1]=='\r'||s_ucData[1]=='\n') && (s_ucData[2]=='\r' || s_ucData[2]=='\n'))
		{
			CmdProcess(s_ucData[0]);
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;

		}
	}
}
static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(char s_cCmd)
{
	switch(s_cCmd)
	{
		case 'a':
			if(WitStartAccCali() != WIT_HAL_OK)
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':
			if(WitStartMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK)
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK)
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else
				uart_set_baudrate(UART_NUM_1, 115200);
			break;
		case 'b':
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else
				uart_set_baudrate(UART_NUM_1, 9600);
			break;
		case 'h':
			ShowHelp();
			break;
	}
}

static void Delayms(uint16_t usMs)
{
	vTaskDelay(usMs / portTICK_PERIOD_MS);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

// static void AutoScanSensor(void)
// {
// 	int i, iRetry;
// 	printf("Auto scan sensor...\r\n");
// 	for(i = 0; i < 0x7F; i++)
// 	{
// 		WitInit(WIT_PROTOCOL_I2C, i);
// 		iRetry = 2;
// 		do
// 		{
// 			s_cDataUpdate = 0;
// 			WitReadReg(AX, 3);
// 			Delayms(10);
// 			if(s_cDataUpdate != 0)
// 			{
// 				printf("find 0x%02X addr sensor\r\n", i);
// 				ShowHelp();
// 				return ;
// 			}
// 			iRetry--;
// 		}while(iRetry);
// 		printf("i = %d\n", i);
// 	}
// 	printf("can not find sensor\r\n");
// 	printf("please check your connection\r\n");
// }
static void AutoScanSensor(void)
{
    int iRetry;

    printf("Try sensor at 0x50...\r\n");
    WitInit(WIT_PROTOCOL_I2C, 0x50);

    iRetry = 5;   // 多试几次，避免初始化延迟
    while (iRetry--)
    {
        s_cDataUpdate = 0;
        WitReadReg(AX, 3);
        Delayms(10);

        if (s_cDataUpdate != 0) {
            printf(">>> Found sensor at 0x50\r\n");
            ShowHelp();
            return;
        }
    }

    printf("!!! Can not find sensor at 0x50\r\n");
    printf("!!! Please check your connection\r\n");
}

static void Vel_data_caculate()
{
    

}
