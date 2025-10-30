#include "mcp251863.h"
#include "MCP251XFD.h"
#include "esp_timer.h"
#include "hal/spi_types.h"
#include <stdint.h>
#include <stdio.h>

/*
bool SPI_Init_EPSI(void) {
  esp_err_t ret;
  // Configuration for the SPI bus
  spi_bus_config_t bus_cfg = {
      .miso_io_num = PIN_NMEA_MISO,
      .mosi_io_num = PIN_NMEA_MOSI,
      .sclk_io_num = PIN_NMEA_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0 // no limit
  };

  // Define MCP2515 SPI device configuration
  spi_device_interface_config_t dev_cfg = {.mode = 0,                  // (0,0)
                                           .clock_speed_hz = 10000000, // 4 mhz
                                           .spics_io_num = PIN_NMEA_CS,
                                           .queue_size = 1024};

  // Initialize SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  printf("bus init passée!\n\r");

  // Add MCP2515 SPI device to the bus
  ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &MCP2515_Object->spi);
  ESP_ERROR_CHECK(ret);

  return true;
}*/

uint32_t getcurrentms() { return esp_timer_get_time() / 1000; }

eERRORRESULT SPI_Init_EPSI(void *pIntDev, uint8_t chipSelect,
                           const uint32_t sckFreq) {

  esp_err_t ret;
  // Configuration for the SPI bus
  spi_bus_config_t bus_cfg = {
      .miso_io_num = CAN_EPSI_MISO,
      .mosi_io_num = CAN_EPSI_MOSI,
      .sclk_io_num = CAN_EPSI_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0 // no limit
  };

  // Define MCP2515 SPI device configuration
  spi_device_interface_config_t dev_cfg = {.mode = 0,                 // (0,0)
                                           .clock_speed_hz = sckFreq, // 4 mhz
                                           .spics_io_num = chipSelect,
                                           .queue_size = 1024};

  // Initialize SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  printf("epsi bus init passée!\n\r");

  // Add MCP2515 SPI device to the bus
  ret =
      spi_bus_add_device(SPI2_HOST, &dev_cfg, (spi_device_handle_t *)&pIntDev);
  ESP_ERROR_CHECK(ret);

  return true;

  return 0;
}
eERRORRESULT SPI_Transfer_EPSI(void *pIntDev, uint8_t chipSelect,
                               uint8_t *txData, uint8_t *rxData, size_t size) {
  spi_transaction_t trans = {};

  trans.length = size;
  trans.rx_buffer = rxData;
  trans.tx_buffer = txData;

  esp_err_t ret = spi_device_transmit(pIntDev, &trans);
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }
  return 0;
}
MCP251XFD MCP251XFD_EPSI = {
    .UserDriverData = NULL,
    //--- Driver configuration ---
    .DriverConfig = MCP251XFD_DRIVER_NORMAL_USE,
    //--- IO configuration ---
    .GPIOsOutLevel = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_LOW,
    //--- Interface driver call functions ---
    .SPI_ChipSelect =
        CAN_EPSI_CS, // Here the chip select of the EXT1 interface is 1
    .InterfaceDevice = NULL,
    // Here this point to the address memory of the peripheral SPI0
    .fnSPI_Init = SPI_Init_EPSI,
    .fnSPI_Transfer = SPI_Transfer_EPSI,
    //--- Time call function ---
    .fnGetCurrentms = getcurrentms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16 = NULL,
    //--- Interface clocks ---
    .SPIClockSpeed = 17000000, // 17MHz
};

MCP251XFD_BitTimeStats MCP2517FD_EPSI_BTStats;
uint32_t SYSCLK_EPSI;
MCP251XFD_Config MCP2517FD_EPSI_Config = {
    //--- Controller clocks ---
    .XtalFreq = 40000000, // CLKIN is a 40MHz quartz
    // CLKIN is not a crystal
    .OscFreq = 0,
    .SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN,
    .ClkoPinConfig = MCP251XFD_CLKO_SOF,
    .SYSCLK_Result = &SYSCLK_EPSI,
    //--- CAN configuration ---
    .NominalBitrate = 500000, // Nominal Bitrate to 250-500kbs
    .DataBitrate = MCP251XFD_NO_CANFD,
    .BitTimeStats = &MCP2517FD_EPSI_BTStats,
    .Bandwidth = MCP251XFD_NO_DELAY,
    .ControlFlags = 0, // voir doc page 55

    //--- GPIOs and Interrupts pins ---
    .GPIO0PinMode = MCP251XFD_PIN_AS_INT0_TX,
    .GPIO1PinMode = MCP251XFD_PIN_AS_INT1_RX,
    .INTsOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
    .TXCANOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
    //--- Interrupts ---
    .SysInterruptFlags = MCP251XFD_INT_NO_EVENT,
};
