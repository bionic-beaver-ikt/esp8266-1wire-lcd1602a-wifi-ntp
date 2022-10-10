/* Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This example shows how to scan for available set of APs.
*/

#include "stdlib.h"
#include <time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/apps/sntp.h"
#include "lwip/netdb.h"
#include "esp_vfs_fat.h"
#include "driver/hw_timer.h"

   //#include "sdmmc_cmd.h" //SDCARD!!!

#define DEFAULT_SCAN_LIST_SIZE 20
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define EXAMPLE_ESP_WIFI_SSID "3-Ogorodnaya-55"
#define EXAMPLE_ESP_WIFI_PASS "@REN@-$0b@k@"
#define PORT 3333
#define KEEPALIVE_IDLE 60
#define KEEPALIVE_INTERVAL 10
#define KEEPALIVE_COUNT 10
//.ssid = "3-Ogorodnaya-55",
//.password = "@REN@-$0b@k@",

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define ONEW_GPIO 5 //NEED TO CHANGE!!!
#define LED 2
#define RS 11
#define E 7
//#define D4 9
#define D4 9
#define D5 10
#define D6 8
#define D7 6

  //#define MOUNT_POINT "/sdcard" //SDCARD!!!
  //#define PIN_NUM_MISO 16 //SDCARD!!!
  //#define PIN_NUM_MOSI 14 //SDCARD!!!
  //#define PIN_NUM_CLK  12 //SDCARD!!!
  //#define PIN_NUM_CS   13 //SDCARD!!!

/*#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#define SPI_DMA_CHAN    host.slot
#else
#define SPI_DMA_CHAN    1
#endif*/

static const char *TAG = "scan";
//static const char *TAG = "wifi station";
static int s_retry_num = 0;
char rx_buffer[128];
int max1=0;
int max2=0;
int sign_max=0;
int min1=0;
int min2=0;
int sign_min=0;
int sign=0;

uint8_t last_temp = 0;

uint8_t digit;
uint8_t decimal;
int dec1;
uint8_t temperature[2];

int written=0;
int debug=0;

SemaphoreHandle_t xSemaphore;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	ESP_LOGI(TAG, "EVENT 1!!!");
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

    time_t now;
    struct tm timeinfo;

static void sntp_example_task()
{
    //time_t now;
    //struct tm timeinfo;
	char strftime_buf[64];

	time(&now);
	localtime_r(&now, &timeinfo);

	ESP_LOGI(TAG, "timeinfo.tm_year = %d", timeinfo.tm_year);

    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
    }

    setenv("TZ", "<+08>-8", 1);
    tzset();

	time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year < (2016 - 1900)) ESP_LOGE(TAG, "The current date/time error");
    else {
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
		if (timeinfo.tm_sec%10==0)
			ESP_LOGW(TAG, "Date: %02d-%02d-%04d. Time: %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		else ESP_LOGI(TAG, "Date: %02d-%02d-%04d. Time: %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    //ESP_LOGI(TAG, "The current date/time in Irkutsk is: %s", strftime_buf);
    }
    /*while (1) {
        // update 'now' variable with current time
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year < (2016 - 1900)) {
            ESP_LOGE(TAG, "The current date/time error");
        } else {
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
			if (timeinfo.tm_sec%10==0)
				ESP_LOGW(TAG, "Date: %02d-%02d-%04d. Time: %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
			else ESP_LOGI(TAG, "Date: %02d-%02d-%04d. Time: %02d:%02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            //ESP_LOGI(TAG, "The current date/time in Irkutsk is: %s", strftime_buf);
        }

        //ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
        vTaskDelay(1000 / portTICK_RATE_MS);
    }*/
}

static void _us_delay(uint32_t time_us)
{
    //ets_delay_us(time_us);
        os_delay_us(time_us);
}

static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}

static void wifi_scan(void)
{
	s_wifi_event_group = xEventGroupCreate();
    //ESP_ERROR_CHECK(esp_netif_init());
	//esp_netif_create_default_wifi_sta();
	tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        print_auth_mode(ap_info[i].authmode);
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
    }
	ESP_ERROR_CHECK(esp_wifi_stop() );
	//esp_event_handler_t instance_any_id;
    //esp_event_handler_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL)); //,&instance_any_id
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL)); //,&instance_got_ip
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
			.threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
	//ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,pdFALSE,pdFALSE,portMAX_DELAY);
	if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler)); // instance_got_ip
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler)); // instance_any_id
    vEventGroupDelete(s_wifi_event_group);
}

/*static void do_retransmit(const int sock)
{
    int len;
    //char rx_buffer[128];
    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}*/

static void lcd_init()
{
//	gpio_set_level(VDD, 0);
	gpio_set_level(RS, 0);
	gpio_set_level(E, 0);
	gpio_set_level(D4, 0);
	gpio_set_level(D5, 0);
	gpio_set_level(D6, 0);
	gpio_set_level(D7, 0);

	//gpio_set_level(VDD, 1);
	_us_delay(20000);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(D4, 1);
	gpio_set_level(D5, 1);
	gpio_set_level(E, 0);
	_us_delay(4200);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(E, 0);
	_us_delay(150);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(E, 0);
	_us_delay(1000);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(D4, 0);
	gpio_set_level(E, 0);
	_us_delay(1000);
}

static void command (unsigned char data)
{
	gpio_set_level(RS, 0);
	gpio_set_level(E, 1);
	_us_delay(150);
	gpio_set_level(D4, 0);
	gpio_set_level(D5, 0);
	gpio_set_level(D6, 0);
	gpio_set_level(D7, 0);

	gpio_set_level(D7, (data&(1<<7))>>7);
	gpio_set_level(D6, (data&(1<<6))>>6);
	gpio_set_level(D5, (data&(1<<5))>>5);
	gpio_set_level(D4, (data&(1<<4))>>4);

	gpio_set_level(E, 0);
	_us_delay(150);
	gpio_set_level(E, 1);
	_us_delay(150);
	gpio_set_level(D4, 0);
	gpio_set_level(D5, 0);
	gpio_set_level(D6, 0);
	gpio_set_level(D7, 0);

	gpio_set_level(D7, (data&(1<<3))>>3);
	gpio_set_level(D6, (data&(1<<2))>>2);
	gpio_set_level(D5, (data&(1<<1))>>1);
	gpio_set_level(D4, data&1);
	gpio_set_level(E, 0);
	_us_delay(2000);
}

static void lcd_send (unsigned char data)
{
	gpio_set_level(RS, 1);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(D4, 0);
	gpio_set_level(D5, 0);
	gpio_set_level(D6, 0);
	gpio_set_level(D7, 0);

	gpio_set_level(D7, (data&(1<<7))>>7);
	gpio_set_level(D6, (data&(1<<6))>>6);
	gpio_set_level(D5, (data&(1<<5))>>5);
	gpio_set_level(D4, (data&(1<<4))>>4);

	gpio_set_level(E, 0);
	_us_delay(50);
	gpio_set_level(E, 1);
	_us_delay(50);
	gpio_set_level(D4, 0);
	gpio_set_level(D5, 0);
	gpio_set_level(D6, 0);
	gpio_set_level(D7, 0);

	gpio_set_level(D7, (data&(1<<3))>>3);
	gpio_set_level(D6, (data&(1<<2))>>2);
	gpio_set_level(D5, (data&(1<<1))>>1);
	gpio_set_level(D4, data&1);
	gpio_set_level(E, 0);
	_us_delay(1000);
}

static void set_position (unsigned char stroka, unsigned char symb)
{
	command((0x40*stroka+symb)|0b10000000);
}

static bool owb_status_reset()
{
    gpio_set_direction(ONEW_GPIO, GPIO_MODE_OUTPUT);
    //_us_delay(0);
    gpio_set_level(ONEW_GPIO, 0);  // Drive DQ low
    _us_delay(480);
    gpio_set_direction(ONEW_GPIO, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(ONEW_GPIO, 1);  // Reset the output level for the next output
    _us_delay(70);
    int level1 = gpio_get_level(ONEW_GPIO);
    _us_delay(410);   // Complete the reset sequence recovery
    int level2 = gpio_get_level(ONEW_GPIO);
	bool present = false;
    if ((level1 == 0) && (level2 == 1)) present = true;   // Sample for presence pulse from slave
    if (debug) ESP_LOGI(TAG, "reset: level1 0x%x, level2 0x%x, present %d", level1, level2, present);
    return present;
}

static void _write_bit(int bit)
{
    int delay1 = bit ? 6 : 60;
    int delay2 = bit ? 64 : 10;
    gpio_set_direction(ONEW_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ONEW_GPIO, 0);  // Drive DQ low
    _us_delay(delay1);
    gpio_set_level(ONEW_GPIO, 1);  // Release the bus
    _us_delay(delay2);
}

static int _read_bit()
{
    int result = 0;
    gpio_set_direction(ONEW_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ONEW_GPIO, 0);  // Drive DQ low
    //_us_delay(1);
    _us_delay(6);
    gpio_set_direction(ONEW_GPIO, GPIO_MODE_INPUT); // Release the bus
    gpio_set_level(ONEW_GPIO, 1);  // Reset the output level for the next output
    _us_delay(9);
    int level = gpio_get_level(ONEW_GPIO);
    _us_delay(55);   // Complete the timeslot and 10us recovery
    result = level & 0x01;
    //_us_delay(1);
    return result;
}

static void _write_bits(uint8_t data)
{
    //ESP_LOGI(TAG, "write 0x%02x", data);
    for (int i = 0; i < 8; ++i)
    {
        _write_bit(data & 0x01);
        data >>= 1;
    }
    //return true;
}

/*static void _read_bits(uint8_t *out, int number_of_bits_to_read)
{
    uint8_t result = 0;
    for (int i = 0; i < number_of_bits_to_read; ++i)
    {
        result >>= 1;
        if (_read_bit())
        {
            result |= 0x80;
        }
    }
    //ESP_LOGI(TAG, "read 0x%02x", result);
    *out = result;

    //return OWB_STATUS_OK;
}*/

uint8_t _read_byte()
{
    uint8_t result = 0;
    for (int i = 0; i < 8; ++i)
    {
        result >>= 1;
        if (_read_bit())
        {
            result |= 0x80;
        }
    }
    //ESP_LOGI(TAG, "read 0x%02x", result);
    return result;

    //return OWB_STATUS_OK;
}

int reverse(int data)
{
    int temp=0;
    switch (data&0x0f)
    {
        case 0: temp|=0; break;
        case 1: temp|=8; break;
        case 2: temp|=4; break;
        case 3: temp|=12; break;
        case 4: temp|=2; break;
        case 5: temp|=10; break;
        case 6: temp|=6; break;
        case 7: temp|=14; break;
        case 8: temp|=1; break;
        case 9: temp|=9; break;
        case 10: temp|=5; break;
        case 11: temp|=13; break;
        case 12: temp|=3; break;
        case 13: temp|=11; break;
        case 14: temp|=7; break;
        case 15: temp|=15; break;
    }
    temp<<=4;
    switch ((data&0xf0)>>4)
    {
        case 0: temp|=0; break;
        case 1: temp|=8; break;
        case 2: temp|=4; break;
        case 3: temp|=12; break;
        case 4: temp|=2; break;
        case 5: temp|=10; break;
        case 6: temp|=6; break;
        case 7: temp|=14; break;
        case 8: temp|=1; break;
        case 9: temp|=9; break;
        case 10: temp|=5; break;
        case 11: temp|=13; break;
        case 12: temp|=3; break;
        case 13: temp|=11; break;
        case 14: temp|=7; break;
        case 15: temp|=15; break;
    }
    return temp;
}

int crc_code8(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7)
{
	bool xor_status=0;
    const int num_of_bytes=7;
    uint8_t data[7]={a1,a2,a3,a4,a5,a6,a7};
    for (int i=0;i<num_of_bytes;i++) data[i]=reverse(data[i]);
    for (int a=0;a<num_of_bytes;a++)
    {
        for(int b=1;b<9;b++)
        {
            if ((data[a]&0x80)==0x80) xor_status=true; else xor_status=false;
            data[a]<<=1;
            if (a!=num_of_bytes-1) if (((data[a+1]<<(b-1))&0x80)==0x80) data[a]|=0x01;
            if (xor_status) data[a]^=0x31;
        }
        if (a!=num_of_bytes) data[a+1]=data[a];
    }
	return reverse ( data[num_of_bytes-1]);
}

int crc_code9(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
{
	bool xor_status=0;
    const int num_of_bytes=8;
    uint8_t data[8]={a1,a2,a3,a4,a5,a6,a7,a8};
    for (int i=0;i<num_of_bytes;i++) data[i]=reverse(data[i]);
    for (int a=0;a<num_of_bytes;a++)
    {
        for(int b=1;b<9;b++)
        {
            if ((data[a]&0x80)==0x80) xor_status=true; else xor_status=false;
            data[a]<<=1;
            if (a!=num_of_bytes-1) if (((data[a+1]<<(b-1))&0x80)==0x80) data[a]|=0x01;
            if (xor_status) data[a]^=0x31;
        }
        if (a!=num_of_bytes) data[a+1]=data[a];
    }
	return reverse ( data[num_of_bytes-1]);
}

int crc_code9_full(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8, uint8_t a9)
{
	bool xor_status=0;
    const int num_of_bytes=9;
    uint8_t data[9]={a1,a2,a3,a4,a5,a6,a7,a8,a9};
    for (int i=0;i<num_of_bytes;i++) data[i]=reverse(data[i]);
    for (int a=0;a<num_of_bytes;a++)
    {
        for(int b=1;b<9;b++)
        {
            if ((data[a]&0x80)==0x80) xor_status=true; else xor_status=false;
            data[a]<<=1;
            if (a!=num_of_bytes-1) if (((data[a+1]<<(b-1))&0x80)==0x80) data[a]|=0x01;
            if (xor_status) data[a]^=0x31;
        }
        if (a!=num_of_bytes) data[a+1]=data[a];
    }
	return reverse ( data[num_of_bytes-1]);
}

void read_temp()
{
	do	{
	uint8_t buffer[5]={0};
	int buffer2[5]={0};
	for (int i=0; i<4; i++)
	{
		int crc_test = 13;
		uint8_t arr[9];
		do	{
			while (!owb_status_reset()) {ESP_LOGI(TAG, "NO-1");}
			_write_bits(0xCC);
			_write_bits(0x44);
			if (debug)
			{
				int cycle=0;
				while(!_read_bit()) {cycle++;}
				ESP_LOGI(TAG, "Cycles: %u", cycle);
			}
			else while(!_read_bit()) {};
			while (!owb_status_reset()) {ESP_LOGI(TAG, "NO-2");}
			_write_bits(0xCC);
			_write_bits(0xBE);
			for (int j=0; j<9; j++)
			{
				arr[j] = _read_byte();
			}
			crc_test = crc_code9_full(arr[0],arr[1],arr[2],arr[3],arr[4],arr[5],arr[6],arr[7],arr[8]);
			if ((debug) & (crc_test!=0)) ESP_LOGI(TAG, "CRC Error!");

			if (crc_test!=0)
			{
                ESP_LOGI(TAG, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x - 0x%02x", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], crc_test);
			}
			if (debug)
			{
				ESP_LOGI(TAG, "TEMP:");
				for (int k=0; k<9; k++)
				{
					ESP_LOGI(TAG, "0x%02x", arr[k]);
				}
				ESP_LOGI(TAG, "CRC: 0x%02x", crc_test);
			}
		}
		//while (crc_test!=0);
		while ((crc_test!=0)|(arr[2]!=0x4B)|(arr[3]!=0x46)|(arr[4]!=0x7F)|(arr[5]!=0xFF)|(arr[6]!=0x0C)|(arr[7]!=0x10));
		temperature[0]=arr[0];
		temperature[1]=arr[1];
		digit=temperature[0]>>4;
		digit|=(temperature[1]&0x7)<<4;
		decimal=temperature[0]&0xf;
		dec1=decimal*625;
		sign = 1;

//		if (debug) ESP_LOGI(TAG, "%+d.%04u C", digit, dec1);
		if ((temperature[1]&0xF8) == 0xF8)
		{
			digit=127-digit;
			dec1= (10000-dec1)%10000;
			if (dec1==0) digit++;
			sign = 0;
		}
		buffer[i+1]= digit;
		buffer2[i+1]= dec1;
		if ((digit<2)|(digit>50)) ESP_LOGI(TAG, "%+d.%04u C 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x - 0x%02x", digit, dec1, arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], crc_test);
		//ESP_LOGI(TAG, "%+d.%04u C - (%d) %d", digit, dec1, i, buffer[i]);
	}
	if(((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[1] < 5 ) & ((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[1] > -5 ))
	{
		digit = buffer[1];
		dec1 = buffer2[1];
		ESP_LOGI(TAG, "1: %d %d %d %d", buffer[1], buffer[2], buffer[3], buffer[4]);
	}
	else if(((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[2] < 5 ) & ((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[2] > -5 ))
	{
		digit = buffer[2];
		dec1 = buffer2[2];
		ESP_LOGI(TAG, "2: %d %d %d %d", buffer[1], buffer[2], buffer[3], buffer[4]);
	}
	else if(((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[3] < 5 ) & ((buffer[1]+buffer[2]+buffer[3]+buffer[4])/4 - buffer[3] > -5 ))
	{
		digit = buffer[3];
		dec1 = buffer2[3];
		ESP_LOGI(TAG, "3: %d %d %d %d", buffer[1], buffer[2], buffer[3], buffer[4]);
	}
	else
	{
		digit = 85;
		ESP_LOGI(TAG, "4: %d, %d, %d, %d = %d", buffer[1], buffer[2], buffer[3], buffer[4], (buffer[1]+buffer[2]+buffer[3]+buffer[4])/4);
	}
	}
	while (digit == 85);
	ESP_LOGI(TAG, "%+d.%04u C", digit, dec1);
}

/*static void adc_task()
{
    int x;
    uint16_t adc_data[100];
	//uint16_t adc_data;
    if (ESP_OK == adc_read(&adc_data)) ESP_LOGI(TAG, "adc read: %d\r\n", adc_data);

    ESP_LOGI(TAG, "adc read fast:\r\n");
    if (ESP_OK == adc_read_fast(adc_data, 100)) {
        for (x = 0; x < 100; x++) {
            printf("%d\n", adc_data[x]);
        }
    }
        //vTaskDelay(1000 / portTICK_RATE_MS);
}*/

static void lcd_intro()
{
	//ESP_LOGI(TAG, "intro");
	/*gpio_pad_select_gpio(13);
	gpio_pad_select_gpio(14);
	gpio_pad_select_gpio(27);
	gpio_pad_select_gpio(26);
	gpio_pad_select_gpio(25);
	gpio_pad_select_gpio(33);
	gpio_pad_select_gpio(32);*/
	//gpio_set_direction(LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RS, GPIO_MODE_OUTPUT);
	gpio_set_direction(E, GPIO_MODE_OUTPUT);
	gpio_set_direction(D4, GPIO_MODE_OUTPUT);
	gpio_set_direction(D5, GPIO_MODE_OUTPUT);
	gpio_set_direction(D6, GPIO_MODE_OUTPUT);
	gpio_set_direction(D7, GPIO_MODE_OUTPUT);
	//gpio_set_direction(32, GPIO_MODE_OUTPUT);

	//gpio_set_level(LED, 0);

	lcd_init();
	//ESP_LOGI(TAG, "init");
	command(0b00101000); //4-bit, 2-lines
	//ESP_LOGI(TAG, "init1");
	command(0b00001100); //display_on, cursor_off, blink_off
	//command(0b00001111); //display_on, cursor_on, blink_on
	//ESP_LOGI(TAG, "init2");
	command(0b00000110); //cursor_right, no_display_shift
	command(0b00000001);
	_us_delay(200);
	//ESP_LOGI(TAG, "init3");
	command(0b01000000);
	//ESP_LOGI(TAG, "init4");
	lcd_send(0b00000100);
	lcd_send(0b00001110);
	lcd_send(0b00010101);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);

	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00000100);
	lcd_send(0b00010101);
	lcd_send(0b00001110);
	lcd_send(0b00000100);

	lcd_send(0b00000110);
	lcd_send(0b00001001);
	lcd_send(0b00001001);
	lcd_send(0b00000110);
	lcd_send(0b00000000);
	lcd_send(0b00000000);
	lcd_send(0b00000000);
	lcd_send(0b00000000);

	lcd_send(0b00001111);
	lcd_send(0b00000111);
	lcd_send(0b00000111);
	lcd_send(0b00000011);
	lcd_send(0b00000011);
	lcd_send(0b00000001);
	lcd_send(0b00000001);
	lcd_send(0b00000000);

	lcd_send(0b00010001);
	lcd_send(0b00001110);
	lcd_send(0b00010001);
	lcd_send(0b00010101);
	lcd_send(0b00010001);
	lcd_send(0b00001110);
	lcd_send(0b00001010);
	lcd_send(0b00011011);

	set_position(0,0);
	lcd_send('W');_us_delay(50000);
	lcd_send('a');_us_delay(50000);
	lcd_send('k');_us_delay(50000);
	lcd_send('e');_us_delay(50000);
	lcd_send(' ');_us_delay(50000);
	lcd_send('u');_us_delay(50000);
	lcd_send('p');_us_delay(50000);
	lcd_send(',');_us_delay(50000);
	lcd_send(' ');_us_delay(50000);
	lcd_send('N');_us_delay(50000);
	lcd_send('e');_us_delay(50000);
	lcd_send('o');_us_delay(50000);
	lcd_send('!');_us_delay(50000);
	lcd_send(' ');_us_delay(50000);
	lcd_send(4);_us_delay(50000);

	set_position(1,0);
	lcd_send('Y');_us_delay(50000);
	lcd_send('o');_us_delay(50000);
	lcd_send('u');_us_delay(50000);
	lcd_send(' ');_us_delay(50000);
	lcd_send('o');_us_delay(50000);
	lcd_send('b');_us_delay(50000);
	lcd_send('o');_us_delay(50000);
	lcd_send('s');_us_delay(50000);
	lcd_send('r');_us_delay(50000);
	lcd_send('a');_us_delay(50000);
	lcd_send('l');_us_delay(50000);
	lcd_send('s');_us_delay(50000);
	lcd_send('y');_us_delay(50000);
	lcd_send('a');_us_delay(50000);
	lcd_send('!');_us_delay(50000);
	_us_delay(500000);
}

static void show_temp()
{
	if (owb_status_reset() == true)
	{
		/*int crc_test=13;
		uint8_t arr[8];
		_write_bits(0x33);
		for (int i=0; i<7; i++) arr[i] = _read_byte();

		crc_test = crc_code9(arr[0],arr[1],arr[2],arr[3],arr[4],arr[5],arr[6],arr[7]);
		if (debug)
		{
			ESP_LOGI(TAG, "ID:");
			for (int j=0; j<7; j++) ESP_LOGI(TAG, "0x%02x", arr[j]);
			ESP_LOGI(TAG, "ID CRC: 0x%02x", crc_test);
		} */
			read_temp();
			if ((dec1%100/10)>5) dec1+=100;
			if ((dec1%1000/100)>5) dec1+=1000;

			if ((sign == 1)&(sign_min==1))
			{
				if (digit<=min1)
				{
					min1=digit;
					min2=dec1;
				}
				if ((digit==min1)&(dec1<min2)) min2=dec1;
			}
			else if ((sign == 0)&(sign_min==0))
			{
				if (digit>=min1)
				{
					min1=digit;
					min2=dec1;
				}
				if ((digit==min1)&(dec1>min2)) min2=dec1;
			}
			else if ((sign==0)&(sign_min==1))
			{
				sign_min=0;
				min1=digit;
				min2=dec1;
			}

			if ((sign == 1)&(sign_max==1))
			{
				if (digit>=max1)
				{
					max1=digit;
					max2=dec1;
				}
				if ((digit==max1)&(dec1>max2)) max2=dec1;
			}
			else if ((sign == 0)&(sign_max==0))
			{
				if (digit<=max1)
				{
					max1=digit;
					max2=dec1;
				}
				if ((digit==max1)&(dec1<max2)) max2=dec1;
			}
			else if ((sign==1)&(sign_max==0))
			{
				sign_max=1;
				max1=digit;
				max2=dec1;
			}
			command(0b00000001);
			_us_delay(200);
			set_position(0,0);
			if (sign==1) lcd_send('+'); else lcd_send('-');
			if ((digit%100/10)!=0) lcd_send(48+digit%100/10);
			lcd_send(48+digit%100%10);
			lcd_send('.');
			lcd_send(48+dec1%10000/1000);
			lcd_send(2);
			lcd_send('C');

			set_position(1,8);
			lcd_send(1);
			if (sign_min==1) lcd_send('+'); else lcd_send('-');
			if ((min1%100/10)!=0) lcd_send(48+min1%100/10);
			lcd_send(48+min1%100%10);
			lcd_send('.');
			lcd_send(48+min2%10000/1000);
			lcd_send(2);
			lcd_send('C');
			set_position(0,8);
			lcd_send(0);
			if (sign_max==1) lcd_send('+'); else lcd_send('-');
			if ((max1%100/10)!=0) lcd_send(48+max1%100/10);
			lcd_send(48+max1%100%10);
			lcd_send('.');
			lcd_send(48+max2%10000/1000);
			lcd_send(2);
			lcd_send('C');
			//char rx_buffer[len]={" "};
			char temp[3];
			char temp4[4];
			char temp2[2];
			time(&now);
			localtime_r(&now, &timeinfo);

			strcpy(rx_buffer, "Date: ");
			itoa(timeinfo.tm_year+1900,temp4,10);
			strcat(rx_buffer, temp4);
			strcat(rx_buffer, "-");
			if (timeinfo.tm_mon+1<10) strcat(rx_buffer, "0");
			itoa(timeinfo.tm_mon+1,temp2,10);
			strcat(rx_buffer, temp2);
			strcat(rx_buffer, "-");
			if (timeinfo.tm_mday<10) strcat(rx_buffer, "0");
			itoa(timeinfo.tm_mday,temp2,10);
			strcat(rx_buffer, temp2);

			strcat(rx_buffer, " Time: ");
			if (timeinfo.tm_hour<10) strcat(rx_buffer, "0");
			itoa(timeinfo.tm_hour,temp2,10);
			strcat(rx_buffer, temp2);
			strcat(rx_buffer, ":");
			if (timeinfo.tm_min<10) strcat(rx_buffer, "0");
			itoa(timeinfo.tm_min,temp2,10);
			strcat(rx_buffer, temp2);
			strcat(rx_buffer, ":");
			if (timeinfo.tm_sec<10) strcat(rx_buffer, "0");
			itoa(timeinfo.tm_sec,temp2,10);
			strcat(rx_buffer, temp2);

			strcat(rx_buffer, " Temp: ");
			if (sign==1) strcat(rx_buffer, "+"); else strcat(rx_buffer, "-");
			itoa(digit,temp,10);
			if (digit<10) strcat(rx_buffer, "0");
			strcat(rx_buffer, temp);
			strcat(rx_buffer, ".");
			itoa(dec1%10000/1000,temp,10);
			strcat(rx_buffer, temp);
			strcat(rx_buffer, "C");

			strcat(rx_buffer, " Min temp: ");
			if (sign_min==1) strcat(rx_buffer, "+"); else strcat(rx_buffer, "-");
			itoa(min1,temp,10);
			if (min1<10) strcat(rx_buffer, "0");
			strcat(rx_buffer, temp);
			strcat(rx_buffer, ".");
			itoa(min2%10000/1000,temp,10);
			strcat(rx_buffer, temp);
			strcat(rx_buffer, "C");

			strcat(rx_buffer, " Max temp: ");
			if (sign_max==1) strcat(rx_buffer, "+"); else strcat(rx_buffer, "-");
			itoa(max1,temp,10);
			if (max1<10) strcat(rx_buffer, "0");
			strcat(rx_buffer, temp);
			strcat(rx_buffer, ".");
			itoa(max2%10000/1000,temp,10);
			strcat(rx_buffer, temp);
			strcat(rx_buffer, "C");

			uint16_t adc_data;
			if (ESP_OK == adc_read(&adc_data))
			{
				//ESP_LOGI(TAG, "adc read: %d\r\n", adc_data);
				itoa(adc_data,temp4,10);
				strcat(rx_buffer, " Voltage: ");
				if (adc_data < 10) strcat(rx_buffer, "00");
				else if (adc_data < 100) strcat(rx_buffer, "0");
				if (adc_data >= 1000) itoa(adc_data%1000,temp4,10);
				strcat(rx_buffer, temp4);
				strcat(rx_buffer, "mV");
			}
			//strcat(rx_buffer, "\n\r");
			strcat(rx_buffer, "\r\0");
//			adc_task();
			//strcat(rx_buffer, "C");
			xSemaphoreGive(xSemaphore);
		}
		else ESP_LOGI(TAG, "no");
}

static void do_send(const int sock)
{
	//int written=0;
	int to_write=0;
    int len=128;
	//strcpy(rx_buffer, "test_test\n\r");
    //char rx_buffer[128];
	//show_temp();
	rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
       // ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
	to_write = len;
    written = send(sock, rx_buffer, to_write, 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;

	while(1)
	{
		struct sockaddr_storage destAddr;
		struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&destAddr;
		dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
		dest_addr_ip4->sin_family = AF_INET;
		dest_addr_ip4->sin_port = htons(PORT);
		ip_protocol = IPPROTO_IP;

		int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
		if (sock < 0) {
			ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
			goto CLEAN_UP;
		}

		ESP_LOGI(TAG, "Socket created");

		int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
		if (err != 0) {
			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
			ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
			close(sock);
			goto CLEAN_UP;
		}
		ESP_LOGI(TAG, "Socket bound, port %d", PORT);

		err = listen(sock, 1);
		if (err != 0) {
			ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
			close(sock);
            goto CLEAN_UP;
		}
		ESP_LOGI(TAG, "Socket listening");

		struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
		socklen_t addr_len = sizeof(source_addr);
		int sock1 = accept(sock, (struct sockaddr *)&source_addr, &addr_len);
		if (sock1 < 0) {
			ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
			break;
		}

		setsockopt(sock1, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
		setsockopt(sock1, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
		setsockopt(sock1, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
		setsockopt(sock1, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
		if (source_addr.ss_family == PF_INET) {
			inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
		}
		ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

		do
		{
			xSemaphoreTake(xSemaphore, ( TickType_t ) 10000);
			do_send(sock1);
//			ESP_LOGI(TAG, "xSemaphoreTake");
			//show_temp();
			//do_send(sock);
			//ESP_LOGI(TAG, "1");
//			vTaskDelay(500);
			//_us_delay(5000000);
		}
		while (written>=0);

		ESP_LOGI(TAG, "After do_retransmit");
		shutdown(sock, 0);
		shutdown(sock1, 0);
		ESP_LOGI(TAG, "After shutdown");
		close(sock);
		close(sock1);
		ESP_LOGI(TAG, "After close");
	}
	CLEAN_UP:
    vTaskDelete(NULL);
}

void app_main(void)
{
        /*timer_config_t config;
	config.divider = 80; // тактирование счетчика 1 мкс
	config.counter_dir = TIMER_COUNT_UP; // прямой счет
	config.counter_en = TIMER_START; // счетчик работает
	config.alarm_en = TIMER_ALARM_DIS; // событие перезагрузка запрещено
	config.auto_reload = TIMER_AUTORELOAD_DIS ; // аппаратная перезагрузка запрещена
	timer_init(TIMER_GROUP_0, TIMER_0 , &config);*/

    //esp_err_t ret = nvs_flash_init();
	//ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

	esp_err_t ret2;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret2 = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret2 = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret2 != ESP_OK) {
        if (ret2 == ESP_FAIL) ESP_LOGE(TAG, "Failed to mount filesystem.");
		else ESP_LOGE(TAG, "Failed to initialize the card (%s).", esp_err_to_name(ret2));
        return;
    }

    sdmmc_card_print_info(stdout, card);

    const char *file_temp = MOUNT_POINT"/temp.txt";

    ESP_LOGI(TAG, "Opening file %s", file_temp1);
    FILE *f = fopen(file_temp, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
	else ESP_LOGE(TAG, "File opened");
	//fprintf(f, "SD-CARD: %s\n", card->cid.name);

    //ESP_LOGI(TAG, "File written");*/

wifi_scan();
sntp_example_task();

	adc_config_t adc_config;

    // Depend on menuconfig->Component config->PHY->vdd33_const value
    // When measuring system voltage(ADC_READ_VDD_MODE), vdd33_const must be set to 255.
    adc_config.mode = ADC_READ_TOUT_MODE;
    adc_config.clk_div = 8; // ADC sample collection clock = 80MHz/clk_div = 10MHz
    ESP_ERROR_CHECK(adc_init(&adc_config));
	//xTaskCreate(sntp_example_task, "sntp_example_task", 2048, NULL, 10, NULL);

	lcd_intro();
	while (owb_status_reset() != true)
	{
	_us_delay(2000000);
	}
	//read_temp();
	read_temp();
	sign_min=sign;
	min1=digit;
	min2=dec1;
	sign_max=sign;
	max1=digit;
	max2=dec1;

	xSemaphore = xSemaphoreCreateBinary();
	if( xSemaphore == NULL ) ESP_LOGI(TAG, "Cannot create semaphore"); else ESP_LOGI(TAG, "Semaphore created"); 

	xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);

	while(1)
	{
//		if (!debug) ESP_LOGI(TAG, "before show_temp");
		show_temp();
//		if (!debug) ESP_LOGI(TAG, "after show_temp");
		//fprintf(f, "Temp: %s\n", rx_buffer);
		//fprintf(f, "%s;", rx_buffer);
		vTaskDelay(100);
	}
	ESP_LOGI(TAG, "After xTask");
	//tcp_server_task((void*)AF_INET);
	//fclose(f);
}
