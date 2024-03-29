#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_log.h>
#include "tcpip_adapter.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static const char *TAG = "esp82xx_test";


//I don't think this is used anymore.
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGI( TAG, "ip: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
        ESP_LOGI( TAG, "netmask: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.netmask));
        ESP_LOGI( TAG, "gw: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.gw));
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

#if 0
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "host",
			.password = "password",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
#else
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t wifi_config = {
		.ap = {
			.ssid = "esp32",
			.password = "password",
			.channel = 0,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = 0,
			.max_connection = 4,
			.beacon_interval = 100
		}
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
#endif

    ESP_ERROR_CHECK( esp_wifi_start() );
}


void app_main()
{
    nvs_flash_init();
	initialise_wifi();

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI( TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI( TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI( TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


//	while(1)
//	{
 //       taskYIELD();
//	}
//	SetupESP82xx();

#if 0
	while(1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
#endif

}
