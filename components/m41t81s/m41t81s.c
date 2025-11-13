#include "m41t81s.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdio.h>

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t rtc_handle;
struct tm flash_time = {.tm_sec = 0,
                        .tm_min = 1,
                        .tm_hour = 2,
                        .tm_mday = 3,
                        .tm_mon = 4,
                        .tm_year = 5,
                        .tm_wday = 6};
static uint8_t m41t81s_getSeconds();
static uint8_t m41t81s_getMinutes();
static uint8_t m41t81s_getHours();
static uint8_t m41t81s_getDate();
static uint8_t m41t81s_getDayOfWeek();
static uint8_t m41t81s_getMonth();
static uint8_t m41t81s_getYear();

uint8_t bcdToDec(uint8_t bcd) { return (bcd & 0x0F) + ((bcd >> 4) * 10); }

uint8_t decToBCD(uint8_t dec) { return ((dec / 10) << 4) + (dec % 10); }

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle) {
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = RTC_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}
void m41t81s_init() { i2c_master_init(&bus_handle, &rtc_handle); }

// lecture des registres
static esp_err_t rtc_register_read(uint8_t reg_addr, uint8_t *data,
                                   size_t len) {
  return i2c_master_transmit_receive(rtc_handle, &reg_addr, 1, data, len,
                                     I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t rtc_register_write_byte(i2c_master_dev_handle_t rtc_handle,
                                         uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_transmit(rtc_handle, write_buf, sizeof(write_buf),
                             I2C_MASTER_TIMEOUT_MS);
}
static esp_err_t rtc_write_bytes(uint8_t *reg_data) {
  return i2c_master_transmit(rtc_handle, reg_data, sizeof(reg_data),
                             I2C_MASTER_TIMEOUT_MS);
}
void m41t81s_reset() {
  // rtc_register_write_byte(rtc_handle, 0x0C, 0);
  // rtc_register_write_byte(rtc_handle, rtc_seconds_reg, 0);
  // rtc_register_write_byte(rtc_handle, rtc_minutes_reg, 0);
  // rtc_register_write_byte(rtc_handle, rtc_hours_reg, 0);
  // rtc_register_write_byte(rtc_handle, rtc_dow_reg, 0b1);
  // rtc_register_write_byte(rtc_handle, rtc_date_reg, 0b00010011);
  // rtc_register_write_byte(rtc_handle, rtc_month_reg, 0b00010010);
  // rtc_register_write_byte(rtc_handle, rtc_year_reg, 0b00010011);
  m41t81s_setTime(&flash_time);
}

esp_err_t m41t81s_getTime(struct tm *now) {
  // bonne chance avec les valeurs magiques :)
  uint8_t time_registers[7];
  esp_err_t ret = rtc_register_read(rtc_seconds_reg, time_registers, 7);
  now->tm_sec = bcdToDec(time_registers[0] & 0x7F);
  now->tm_min = bcdToDec(time_registers[1] & 0x7F); /* Minutes. [0-59]      */
  now->tm_hour = bcdToDec(time_registers[2] & 0x3F);
  now->tm_wday = bcdToDec(time_registers[3] & 0x07);
  now->tm_mday = bcdToDec(time_registers[4] & 0x3F);
  now->tm_mon = bcdToDec(time_registers[5] & 0x1F);
  now->tm_year = bcdToDec(time_registers[6]);
  return ret;
}

esp_err_t m41t81s_setTime(struct tm *now) {

  uint8_t time_registers[8] = {rtc_month_reg,
                               decToBCD(now->tm_sec),
                               decToBCD(now->tm_min),
                               decToBCD(now->tm_hour),
                               decToBCD(now->tm_wday) & 0x07,
                               decToBCD(now->tm_mday) & 0x3F,
                               decToBCD(now->tm_mon) & 0x1F,
                               decToBCD(now->tm_year)};
  return rtc_write_bytes(time_registers);
};
// disabled because likely useless
// static uint8_t m41t81s_getPartSeconds() {
//   uint8_t psecs;
//   ESP_ERROR_CHECK(rtc_register_read(rtc_pseconds_reg, &psecs, 1));
//   return (psecs & 0x0F) + (((psecs >> 4) & 0x0F) * 10);
// }

static uint8_t m41t81s_getSeconds() {
  uint8_t sec;
  ESP_ERROR_CHECK(rtc_register_read(rtc_seconds_reg, &sec, 1));
  return bcdToDec(sec & 0x7F);
}

static uint8_t m41t81s_getMinutes() {
  uint8_t min;
  ESP_ERROR_CHECK(rtc_register_read(rtc_minutes_reg, &min, 1));
  return bcdToDec(min & 0x7F);
}
uint8_t m41t81s_getHours() {
  uint8_t hr;
  ESP_ERROR_CHECK(rtc_register_read(rtc_hours_reg, &hr, 1));
  return bcdToDec(hr & 0x3F);
}

uint8_t m41t81s_getDayOfWeek() {
  uint8_t day;
  ESP_ERROR_CHECK(rtc_register_read(rtc_dow_reg, &day, 1));
  return bcdToDec(day & 0x07);
}

uint8_t m41t81s_getDate() {
  uint8_t date;
  ESP_ERROR_CHECK(rtc_register_read(rtc_date_reg, &date, 1));
  return bcdToDec(date & 0x3F);
}

uint8_t m41t81s_getMonth() {
  uint8_t month;
  ESP_ERROR_CHECK(rtc_register_read(rtc_month_reg, &month, 1));
  return bcdToDec(month & 0x1F);
}

uint8_t m41t81s_getYear() {
  uint8_t year;
  ESP_ERROR_CHECK(rtc_register_read(rtc_year_reg, &year, 1));
  return bcdToDec(year);
}
/*
void rtc_setSeconds(uint8_t secs) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_seconds_reg);
  Wire.write(decToBCD(secs));
  Wire.endTransmission();
}

void rtc_setMinutes(uint8_t mins) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_minutes_reg);
  Wire.write(decToBCD(mins));
  Wire.endTransmission();
}

void rtc_setHours(uint8_t hrs) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_hours_reg);
  Wire.write(decToBCD(hrs));
  Wire.endTransmission();
}

void rtc_setDayOfWeek(uint8_t dow) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_dow_reg);
  Wire.write(decToBCD(dow));
  Wire.endTransmission();
}

void rtc_setDate(uint8_t date) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_date_reg);
  Wire.write(decToBCD(date));
  Wire.endTransmission();
}

void rtc_setMonth(uint8_t month) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_month_reg);
  Wire.write(decToBCD(month) & 0x1F);
  Wire.endTransmission();
}

void rtc_setYear(uint8_t year) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(rtc_year_reg);
  Wire.write(decToBCD(year));
  Wire.endTransmission();
}

M41T81SClass rtc;
*/
