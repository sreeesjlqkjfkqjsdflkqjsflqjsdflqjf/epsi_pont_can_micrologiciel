#include "ErrorsDef.h"
#include "MCP251XFD.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "hal/spi_types.h"
#define CAN_NMEA_CS 10
#define CAN_NMEA_MISO 11
#define CAN_NMEA_MOSI 13
#define CAN_NMEA_CLK 12
#define CAN_NMEA_INT 38

#define CAN_EPSI_CS 21
#define CAN_EPSI_MISO 9
#define CAN_EPSI_MOSI 46
#define CAN_EPSI_CLK 45
#define CAN_EPSI_INT 39

extern MCP251XFD MCP251XFD_EPSI;

extern MCP251XFD_BitTimeStats MCP2517FD_EPSI_BTStats;
extern uint32_t SYSCLK_EPSI;
extern MCP251XFD_Config MCP2517FD_EPSI_Config;
extern eERRORRESULT ConfigureMCP251XFDDeviceOnCAN_EPSI(void);
