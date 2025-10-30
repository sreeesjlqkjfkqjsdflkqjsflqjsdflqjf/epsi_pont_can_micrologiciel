#include "led.h"
#include "esp_err.h"
#include <stdio.h>

esp_err_t LED_init(LED_t *led) {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = 1ULL << led->pin_number;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  return gpio_set_level(led->pin_number, led->state);
}

esp_err_t LED_toggle(LED_t *led) {
  led->state = 1 ^ led->state;
  return gpio_set_level(led->pin_number, led->state);
}

esp_err_t LED_on(LED_t *led) {
  led->state = 1;
  return gpio_set_level(led->pin_number, led->state);
}

esp_err_t LED_off(LED_t *led) {
  led->state = 0;
  return gpio_set_level(led->pin_number, led->state);
}

LED_t led_verte = {.pin_number = PIN_LED_1};
LED_t led_rouge = {.pin_number = PIN_LED_2};
