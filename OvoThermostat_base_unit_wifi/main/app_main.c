#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#define SSR_OUTPUT_1 GPIO_NUM_26
#define SSR_OUTPUT_2 GPIO_NUM_25
#define SSR_OUTPUT_3 GPIO_NUM_33
#define SSR_OUTPUT_4 GPIO_NUM_32
#define SSR_OUTPUT_5 GPIO_NUM_15
#define SSR_OUTPUT_6 GPIO_NUM_14
#define SSR_OUTPUT_7 GPIO_NUM_27

#define SSR_IO_OUTPUT_PIN_SEL ((1ULL<<SSR_OUTPUT_1) | (1ULL<<SSR_OUTPUT_2) | (1ULL<<SSR_OUTPUT_3) | (1ULL<<SSR_OUTPUT_4) | (1ULL<<SSR_OUTPUT_5) | (1ULL<<SSR_OUTPUT_6) | (1ULL<<SSR_OUTPUT_7))

#define TCI_TXD  (GPIO_NUM_17)
#define TCI_RXD  (GPIO_NUM_16)
#define TCI_RTS  (UART_PIN_NO_CHANGE)
#define TCI_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

/**< All sensor data buffer size */
#define SENSOR_DATA_BUF_SIZE        2//15

static const char *TAG = "MQTT_EXAMPLE";
static const char *TAG_UART1 = "UART1_LOG";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static uint8_t connect_state = 0;

void ssr_io_config (void)
{
	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = SSR_IO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
			
			connect_state = 1;
			
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			connect_state = 0;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
    }
    return ESP_OK;
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

esp_mqtt_client_handle_t client;

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    //esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 

int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 

// Converts a floating point number to string. 
void ftoa(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    }
} 

static void tci_task()
{
	float sensor_data[SENSOR_DATA_BUF_SIZE];
	char res[20];
	
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TCI_TXD, TCI_RXD, TCI_RTS, TCI_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
		vTaskDelay( 100 / portTICK_PERIOD_MS );
        // Read data from the UART
		memset(data, 0, BUF_SIZE);
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        
		ESP_LOGI(TAG_UART1, "uart1 len = %d", len);
		
		if ((connect_state != 0) && (len>=(SENSOR_DATA_BUF_SIZE*sizeof(float)))) //len==12, three float sensor values
		{
			memcpy(sensor_data, data, sizeof(sensor_data));
			
			//ESP_LOGI(TAG_UART1, "---sensor data---");
			//for (int sensor_index=0; sensor_index<SENSOR_DATA_BUF_SIZE; sensor_index++)
			//{
			//	ESP_LOGI(TAG_UART1, "[%f]", sensor_data[sensor_index]);
			//}
			
			int msg_id;
			
			// Create string of all sensor data
			
			for(int i=0; i<SENSOR_DATA_BUF_SIZE; i++)
			{
				memset(res, 0, sizeof(res));
				ftoa(sensor_data[i], res, 2);
				msg_id = esp_mqtt_client_publish(client, "/topic/qos1/sensor_data", (char*)res, 0, 1, 0);
			}
			ESP_LOGI(TAG_UART1, "sent publish successful, msg_id=%d", msg_id);
			
			//if(data[2] == (uint8_t)'t')
			//{
			//	memset(res, 0, sizeof(res));
			//	ftoa(sensor_data[0], res, 2);
			//	msg_id = esp_mqtt_client_publish(client, "/topic/qos1/temp", (char*)res, 0, 1, 0);
			//	ESP_LOGI(TAG_UART1, "sent publish successful, msg_id=%d", msg_id);
			//}
		}
    }
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    nvs_flash_init();
    wifi_init();
    mqtt_app_start();
	ssr_io_config();
	gpio_set_level(SSR_OUTPUT_1, 0);
	gpio_set_level(SSR_OUTPUT_2, 0);
	gpio_set_level(SSR_OUTPUT_3, 0);
	gpio_set_level(SSR_OUTPUT_4, 0);
	gpio_set_level(SSR_OUTPUT_5, 0);
	gpio_set_level(SSR_OUTPUT_6, 0);
	gpio_set_level(SSR_OUTPUT_7, 0);
	
	xTaskCreate(tci_task, "uart_tci_task", 2048, NULL, 10, NULL);
}
