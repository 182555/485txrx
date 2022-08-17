/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_log.h"
// #include "driver/uart.h"
// #include "string.h"
// #include "driver/gpio.h"
// #include "freertos/queue.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "stdlib.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "stdio.h"
#include "driver/gpio.h"
#include "driver/uart.h"

static const int RX_BUF_SIZE = 1024 * 2;

#define TXD_PIN (GPIO_NUM_14)
#define RXD_PIN (GPIO_NUM_27)
#define RTS_PIN (GPIO_NUM_15)

#define UART_NUM_0 (0) /*!< UART port 0 */
#define UART_NUM_1 (1) /*!< UART port 1 */

// QueueHandle_t uart_queue;
// QueueHandle_t udp_send_queue;
// QueueHandle_t uart_tx_queue;

// typedef enum {
//     PC,
//     robot,
//     process
// } queue_sender;

// typedef struct
// {
//     /* data */
//     uint8_t msg_array[10];
//     uint8_t udp_recv_array[14];
//     // uint8_t  process_array[34];
//     queue_sender q_sender;
//     int sender_int;       // 0: robot_rx; 1: PC_UDP_rev; 2: Rtn_button press monitor task.
//     uint8_t process_flag; // Bit0: rtn_pressed;

// } queue_msg;

// typedef struct
// {
//     /* data */
//     uint8_t  udp_send_array[34];
//     //queue_sender q_sender;

// } udp_send_msg;

// int StrToInt(char s[4])
// {
//     int i, m, temp = 0, n;
//     m = strlen(s); //十六进制是按字符串传进来的，所以要获得字符串长度
//     for (i = 0; i < m; i++)
//     {
//         if (s[i] >= 'A' && s[i] <= 'F') //十六进制还要判断字符是不是在A-F或者a-f之间
//         {
//             n = s[i] - 'A' + 10;
//         }
//         else if (s[i] >= 'a' && s[i] <= 'f')
//         {
//             n = s[i] - 'a' + 10;
//         }
//         else
//         {
//             n = s[i] - '0';
//         }
//         temp = temp * 16 + n;
//     }
//     return temp;
// }
int htoi(char* str)
{
	int n = 0;
	if (str == NULL) return -1;
	if (*str == '0' && (*(str + 1) == 'x' || *(str + 1) == 'X'))
	{
		str += 2;
	}
	while (1)
	{
		if (*str >= '0' && *str <= '9')
		{
			n = 16 * n + (*str - '0');
		}
		else if (*str >= 'A' && *str <= 'F')//十六进制还要判断字符是不是在A-F或者a-f之间
		{
			n = 16 * n + (*str - 'A' + 10);
		}
		else if (*str >= 'a' && *str <= 'f')
		{
			n = 16 * n + (*str - 'a' + 10);
		}
		else
		{
			break;
		}
		str++;
	}
	return n;
}


void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,

    };
    //  uint8_t * tx_buf = (uint8_t *)malloc(4);     //申请6字节内存
    //   { *tx_buf     = 0xAA;
    //     *(tx_buf+1) = 0x07;
    //     *(tx_buf+2) = 0x02;
    //     *(tx_buf+3) = 0x00;}
    //     return(tx_buf);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// int sendData(const char *logName, const char *data)
// {
// //         const int len = strlen(data);
//     const uint8_t txBytes = uart_write_bytes(UART_NUM_1, data, 14);
// //     //     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }
int i = 1;
static void tx_task(void *arg)
{
    // uint8_t Tx_buf[] = {0xFE, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00,0XFF,0xCF, 0xFC, 0xCC, 0xFF};
    uint8_t Tx_buf[] = {0xFE, 0x01, 0x07, 0x00, 0x01, 0x00, 0x01, 0XFF, 0xCF, 0xFC, 0xCC, 0xFF};
    // uint8_t Tx_buf[] = {F,0,0,7,0,0,0,0,0,0,0,0,F,F,C,F,F,C,C,C,F,F};
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    int txBytes = 0;

    // (void) txBytes;
    // {

    // }
    // queue_msg uart_tx_struct;
    while (1)
    {

        txBytes = uart_write_bytes(UART_NUM_1, Tx_buf, 12);
        vTaskDelay(100000 / portTICK_PERIOD_MS);

        // if (xQueueReceive(uart_tx_queue, &uart_tx_struct, (TickType_t)10) == pdPASS)
        // sendData(TX_TASK_TAG,&Tx_buf);
        // {
        // printf("down");
        // memcpy(Tx_buf, uart_tx_struct.msg_array, 14);
        //    i=1;
        // }
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, 12, 1);
        if (rxBytes > 0)
        {
            // if(i==1)
            // {

            data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes,  data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            //  vTaskDelay(2000 / portTICK_PERIOD_MS);
            // i=0;
            // }
            char s1[4];
            memcpy(s1,data+4,4);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, s1, 4, ESP_LOG_INFO);
            char s1_data[9];
            // sprintf(s1_data,"%s",s1);
            sprintf(s1_data,"%02x%02x%02x%02x",s1[0],s1[1],s1[2],s1[3]);
            // int i;
            // for (i=0;i<6;i++) 
            // sprintf(s1_data+i*2,"%02X",(unsigned char)s1);

	        int s2_data = 0;
            s2_data=htoi(s1_data);
            // ESP_LOGI(RX_TASK_TAG, "%s ", s1_data);
            ESP_LOGI(RX_TASK_TAG, "%d ", s2_data);
	       
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, s1_data,4, ESP_LOG_INFO);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, s1,4, ESP_LOG_INFO);

            //  StrToInt( s1[4]);
        }
        //  vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    free(data);
}

void app_main(void)
{

    // //创建队列queue
    // uart_queue = xQueueCreate(10, sizeof(queue_msg));
    // udp_send_queue = xQueueCreate(10, sizeof(udp_send_msg));
    // uart_tx_queue = xQueueCreate(10, sizeof(queue_msg));
    // StrToInt();
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
