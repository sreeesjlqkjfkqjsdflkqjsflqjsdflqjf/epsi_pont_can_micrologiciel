#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <time.h>
// i2c
#define RTC_ADDR 0x68
#define I2C_MASTER_SCL_IO 2       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 1       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
/*Register address definitions*/
#define rtc_pseconds_reg 0x00
#define rtc_seconds_reg 0x01
#define rtc_minutes_reg 0x02
#define rtc_hours_reg 0x03
#define rtc_dow_reg 0x04
#define rtc_date_reg 0x05
#define rtc_month_reg 0x06
#define rtc_year_reg 0x07

extern struct tm flash_time;

void m41t81s_init();
void m41t81s_reset();
esp_err_t m41t81s_getTime(struct tm *now);
esp_err_t m41t81s_setTime(struct tm *now);
