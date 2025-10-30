#include "driver/gpio.h"

#define PIN_LED_1 5
#define PIN_LED_2 6

typedef struct {
  int pin_number;
  bool state;
} LED_t;
esp_err_t LED_init(LED_t *led);
esp_err_t LED_toggle(LED_t *led);
esp_err_t LED_on(LED_t *led);
esp_err_t LED_off(LED_t *led);

extern LED_t led_verte;
extern LED_t led_rouge;
