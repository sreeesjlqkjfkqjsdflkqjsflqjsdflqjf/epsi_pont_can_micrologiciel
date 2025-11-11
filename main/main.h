
#include <stdio.h>
#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/idf_additions.h"
#include "lwip/inet.h"
#include "nvs_flash.h"

#include "dns_server.h"
#include "esp_http_server.h"

#include "cJSON.h"
#include "esp_chip_info.h"

#include "battery_data.h"
#include "led.h"
#include "m41t81s.h"
#include "mcp251863.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN
extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");
static const char *TAG = "example";
struct tm now;
struct battery_detailed_data bat_1 = {
    .address = 0x1,
    .state_of_health = 255,
    .terminal_current = 30E3,
    .internal_voltage = 60E3,
    .remaining_capacity = 60E3,
    .state_of_charge = 250,
    .charging_voltage = 60E3,
    .charging_current = 60E3,
};
struct battery_summary bat_2 = {
    .address = 0x1,
    .battery_voltage = 100,
    .battery_current = 200,
    .battery_temperature = 100,
};
