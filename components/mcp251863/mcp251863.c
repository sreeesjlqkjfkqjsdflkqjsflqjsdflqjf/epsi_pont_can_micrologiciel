#include "mcp251863.h"
#include "MCP251XFD.h"
#include "driver/spi_master.h"
#include "esp_log.h"
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
  spi_device_interface_config_t dev_cfg = {
      .mode = 0,                 // (0,0)
      .clock_speed_hz = sckFreq, // 4 mhz
      .spics_io_num = chipSelect,
      .queue_size = 1024,
      // tbu désactivation des commandes / addresses car spi command unique
      // .command_bits = 4,
      // .address_bits = 12,

  };

  // Initialize SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  // Add mcp251863 SPI device to the bus
  ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, (spi_device_handle_t *)pIntDev);
  ESP_ERROR_CHECK(ret);

  // return true;
  return 0;
}

eERRORRESULT SPI_Init_NMEA(void *pIntDev, uint8_t chipSelect,
                           const uint32_t sckFreq) {

  esp_err_t ret;
  // Configuration for the SPI bus
  spi_bus_config_t bus_cfg = {
      .miso_io_num = CAN_NMEA_MISO,
      .mosi_io_num = CAN_NMEA_MOSI,
      .sclk_io_num = CAN_NMEA_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0 // no limit
  };

  // Define MCP2515 SPI device configuration
  spi_device_interface_config_t dev_cfg = {
      .mode = 0,                 // (0,0)
      .clock_speed_hz = sckFreq, // 4 mhz
      .spics_io_num = chipSelect,
      .queue_size = 1024,
      // tbu désactivation des commandes / addresses car spi command unique
      // .command_bits = 4,
      // .address_bits = 12,

  };

  // Initialize SPI bus
  ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  // Add mcp251863 SPI device to the bus
  ret = spi_bus_add_device(SPI3_HOST, &dev_cfg, (spi_device_handle_t *)pIntDev);
  ESP_ERROR_CHECK(ret);

  // return true;
  return 0;
}
eERRORRESULT SPI_Transfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData,
                          uint8_t *rxData, size_t size) {
  // uint8_t Buffer[MCP251XFD_TRANS_BUF_SIZE];
  // spi_transaction_t trans = {
  //     .cmd = txData[0] >> 4,
  //     .addr = (uint16_t)(txData[0] & 0b00001111) << 8 | (uint16_t)txData[1],
  //     .length = (size - 2) * 8,
  //     .rx_buffer = Buffer,
  //     .tx_buffer = trans.cmd == 2 ? txData + 2 : NULL,
  // };
  // printf("spi_transfer_called : cmd %u , addr %llu\n", trans.cmd,
  // trans.addr); if (trans.tx_buffer != NULL) {
  //   for (int i = 0; i < size; i++) {
  //     printf("spi_device_transmit tx_buffer : %u\n", txData[i]);
  //   }
  // }
  // esp_err_t ret =
  //     spi_device_polling_transmit(*(spi_device_handle_t *)pIntDev, &trans);

  // if (rxData != NULL) {
  //   for (int i = 0; i < size; i++) {
  //     printf("spi_device_transmit rx_buffer : %u\n", Buffer[i]);
  //   }
  //   for (int i = 0; i < size - 2; i++) {
  //     rxData[i] = Buffer[i];
  //   }
  // }
  // if (ret != ESP_OK) {
  //   printf("spi_device_transmit failed\n");
  // }
  // printf("SPI_Transfer_EPSI size : %u\n", size);
  spi_transaction_t trans = {
      .length = size * 8,
      .rx_buffer = rxData,
      .tx_buffer = txData,
  };
  // if (trans.tx_buffer != NULL) {
  //   for (int i = 0; i < size; i++) {
  //     printf("spi_device_transmit tx_buffer : %u\n", txData[i]);
  //   }
  // }
  esp_err_t ret =
      spi_device_polling_transmit(*(spi_device_handle_t *)pIntDev, &trans);

  // if (rxData != NULL) {
  //   for (int i = 0; i < size; i++) {
  //     printf("spi_device_transmit rx_buffer : %u\n", rxData[i]);
  //   }
  // }
  if (ret != ESP_OK) {
    printf("spi_device_transmit failed\n");
  }
  return 0;
}
spi_device_handle_t can_epsi_dev_handle;
MCP251XFD MCP251XFD_EPSI = {
    .UserDriverData = NULL,
    //--- Driver configuration ---
    .DriverConfig = MCP251XFD_DRIVER_NORMAL_USE,
    //--- IO configuration ---
    .GPIOsOutLevel = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_LOW,
    //--- Interface driver call functions ---
    .SPI_ChipSelect =
        CAN_EPSI_CS, // Here the chip select of the EXT1 interface is 1
    .InterfaceDevice = &can_epsi_dev_handle,
    // Here this point to the address memory of the peripheral SPI0
    .fnSPI_Init = SPI_Init_EPSI,
    .fnSPI_Transfer = SPI_Transfer,
    //--- Time call function ---
    .fnGetCurrentms = getcurrentms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16 = NULL,
    //--- Interface clocks ---
    .SPIClockSpeed = 2E6, // 17MHz
};

spi_device_handle_t can_nmea_dev_handle;
MCP251XFD MCP251XFD_NMEA = {
    .UserDriverData = NULL,
    //--- Driver configuration ---
    .DriverConfig = MCP251XFD_DRIVER_NORMAL_USE,
    //--- IO configuration ---
    .GPIOsOutLevel = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_LOW,
    //--- Interface driver call functions ---
    .SPI_ChipSelect =
        CAN_EPSI_CS, // Here the chip select of the EXT1 interface is 1
    .InterfaceDevice = &can_epsi_dev_handle,
    // Here this point to the address memory of the peripheral SPI0
    .fnSPI_Init = SPI_Init_NMEA,
    .fnSPI_Transfer = SPI_Transfer,
    //--- Time call function ---
    .fnGetCurrentms = getcurrentms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16 = NULL,
    //--- Interface clocks ---
    .SPIClockSpeed = 2E6, // 17MHz
};
MCP251XFD_BitTimeStats MCP2517FD_EPSI_BTStats;
uint32_t SYSCLK_EPSI;
MCP251XFD_Config MCP2517FD_EPSI_Config = {
    //--- Controller clocks ---
    .XtalFreq = 40000000, // CLKIN is a 40MHz quartz
    .OscFreq = 0,
    .SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN,
    .ClkoPinConfig = MCP251XFD_CLKO_SOF,
    .SYSCLK_Result = &SYSCLK_EPSI,
    //--- CAN configuration ---
    .NominalBitrate = 250E3, // Nominal Bitrate to 250-500kbs
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
MCP251XFD_RAMInfos EPSI_TEF_RAMInfos;
MCP251XFD_RAMInfos EPSI_TXQ_RAMInfos;
MCP251XFD_RAMInfos EPSI_FIFOs_RAMInfos[MCP2517FD_EPSI_FIFO_COUNT - 2];
MCP251XFD_FIFO MCP2517FD_EPSI_FIFOlist[MCP2517FD_EPSI_FIFO_COUNT] = {
    {
        .Name = MCP251XFD_TEF,
        .Size = MCP251XFD_FIFO_10_MESSAGE_DEEP,
        .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_OBJ,
        .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT +
                          MCP251XFD_FIFO_EVENT_FIFO_NOT_EMPTY_INT,
        .RAMInfos = &EPSI_TEF_RAMInfos,
    },
    {
        .Name = MCP251XFD_TXQ,
        .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP,
        .Payload = MCP251XFD_PAYLOAD_64BYTE,
        .Attempts = MCP251XFD_THREE_ATTEMPTS,
        .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16,
        .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
        .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT +
                          MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
        .RAMInfos = &EPSI_TXQ_RAMInfos,
    },
    {
        .Name = MCP251XFD_FIFO1,
        .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP,
        .Payload = MCP251XFD_PAYLOAD_64BYTE,
        .Direction = MCP251XFD_RECEIVE_FIFO,
        .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
        .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT +
                          MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
        .RAMInfos = &EPSI_FIFOs_RAMInfos[0],
    }, // SID: 0x000..0x1FF ; No EID
    {
        .Name = MCP251XFD_FIFO2,
        .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP,
        .Payload = MCP251XFD_PAYLOAD_64BYTE,
        .Direction = MCP251XFD_TRANSMIT_FIFO,
        .Attempts = MCP251XFD_THREE_ATTEMPTS,
        .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16,
        .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
        .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT +
                          MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
        .RAMInfos = &EPSI_FIFOs_RAMInfos[1],
    },
};
MCP251XFD_Filter MCP2517FD_EPSI_FilterList[MCP2517FD_EPSI_FILTER_COUNT] = {
    {
        .Filter = MCP251XFD_FILTER0,
        .EnableFilter = true,
        .Match = MCP251XFD_MATCH_ONLY_EID,
        .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES,
        .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES,
        .PointTo = MCP251XFD_FIFO1,
    }, // 0x000..0x1FF
       // { .Filter = MCP251XFD_FILTER1, .EnableFilter = true, .Match =
    // MCP251XFD_MATCH_ONLY_SID, .AcceptanceID = 0x200, .AcceptanceMask = 0x600,
    // .PointTo = MCP251XFD_FIFO2, }, // 0x200..0x3FF { .Filter =
    // MCP251XFD_FILTER2, .EnableFilter = true, .Match =
    // MCP251XFD_MATCH_ONLY_SID, .AcceptanceID = 0x400, .AcceptanceMask = 0x600,
    // .PointTo = MCP251XFD_FIFO3, }, // 0x400..0x5FF { .Filter =
    // MCP251XFD_FILTER3, .EnableFilter = true, .Match =
    // MCP251XFD_MATCH_ONLY_SID, .AcceptanceID = 0x600, .AcceptanceMask = 0x600,
    // .PointTo = MCP251XFD_FIFO4, }, // 0x600..0x7FF
};
//=============================================================================
// Configure the MCP251863 device on EPSI
//=============================================================================
eERRORRESULT ConfigureMCP251XFDDeviceOnCAN_EPSI(void) {
  //--- Initialize Int pins or GPIOs ---
  //--- Configure module on EPSI ---
  eERRORRESULT ErrorEPSI = ERR__NO_DEVICE_DETECTED;
  ErrorEPSI = Init_MCP251XFD(&MCP251XFD_EPSI, &MCP2517FD_EPSI_Config);
  if (ErrorEPSI != ERR_OK) {
    return ErrorEPSI;
  }
#define TIMESTAMP_TICK_us (25)
#define TIMESTAMP_TICK(sysclk) (((sysclk) / 1E6) * TIMESTAMP_TICK_us)
  ErrorEPSI = MCP251XFD_ConfigureTimeStamp(&MCP251XFD_EPSI, false,
                                           MCP251XFD_TS_CAN20_SOF_CANFD_SOF,
                                           TIMESTAMP_TICK(SYSCLK_EPSI), false);
  if (ErrorEPSI != ERR_OK) {
    return ErrorEPSI;
  }
  ErrorEPSI = MCP251XFD_ConfigureFIFOList(
      &MCP251XFD_EPSI, &MCP2517FD_EPSI_FIFOlist[0], MCP2517FD_EPSI_FIFO_COUNT);
  if (ErrorEPSI != ERR_OK)
    return ErrorEPSI;
  ErrorEPSI = MCP251XFD_ConfigureFilterList(
      &MCP251XFD_EPSI, MCP251XFD_D_NET_FILTER_DISABLE,
      &MCP2517FD_EPSI_FilterList[0], MCP2517FD_EPSI_FILTER_COUNT);
  if (ErrorEPSI != ERR_OK)
    return ErrorEPSI;

  // ErrorEPSI = MCP251XFD_StartCAN20(&MCP251XFD_EPSI);
  ErrorEPSI = MCP251XFD_StartCAN20(&MCP251XFD_EPSI);
  return ErrorEPSI;
}
//=============================================================================
// Transmit a hardcoded message on EPSI
//=============================================================================
eERRORRESULT TransmitMessageToEPSI(void) {
  eERRORRESULT ErrorEPSI = ERR_OK;
  eMCP251XFD_FIFOstatus FIFOstatus = 0;
  ErrorEPSI = MCP251XFD_GetFIFOStatus(&MCP251XFD_EPSI, MCP251XFD_FIFO2,
                                      &FIFOstatus); // First get FIFO2 status
  if (ErrorEPSI != ERR_OK)
    return ErrorEPSI;
  if ((FIFOstatus & MCP251XFD_TX_FIFO_NOT_FULL) > 0)
  // Second check FIFO not full
  {
    uint8_t payload[4] = {0xde, 0xad, 0xbe, 0xef};
    MCP251XFD_CANMessage TansmitMessage;
    TansmitMessage.MessageID = 0xc0ffee;
    TansmitMessage.MessageSEQ = 0;
    TansmitMessage.ControlFlags = MCP251XFD_CAN20_FRAME;
    TansmitMessage.DLC = 4;
    TansmitMessage.PayloadData = payload;
    //  Send message and flush
    ErrorEPSI = MCP251XFD_TransmitMessageToFIFO(
        &MCP251XFD_EPSI, &TansmitMessage, MCP251XFD_FIFO2, true);
  }
  // printf("ouais, c'est mort : %u\n", FIFOstatus);
  return ErrorEPSI;
}
//=============================================================================
// Receive a message from EPSI
//=============================================================================
eERRORRESULT ReceiveMessageFromEPSI(MCP251XFD_CANMessage *ReceivedMessage) {
  eERRORRESULT ErrorEPSI = ERR_OK;
  eMCP251XFD_FIFOstatus FIFOstatus = 0;
  ErrorEPSI = MCP251XFD_GetFIFOStatus(&MCP251XFD_EPSI, MCP251XFD_FIFO1,
                                      &FIFOstatus); // First get FIFO1 status
  printf("FIFO_status : %u\n", FIFOstatus);
  if (ErrorEPSI != ERR_OK)
    return ErrorEPSI;
  if ((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) >
      0) // Second check FIFO not empty
  {
    uint32_t MessageTimeStamp = 0;
    uint8_t RxPayloadData[64];
    // In this example, the FIFO1 have 64 bytes of payload
    // MCP251XFD_CANMessage ReceivedMessage;
    // ReceivedMessage.PayloadData =
    //     &RxPayloadData[0]; // Add receive payload data pointer to the message
    // structure
    // that will be received
    ErrorEPSI = MCP251XFD_ReceiveMessageFromFIFO(
        &MCP251XFD_EPSI, ReceivedMessage, MCP251XFD_PAYLOAD_64BYTE,
        &MessageTimeStamp, MCP251XFD_FIFO1);
    printf("MessageId ");
    if (ErrorEPSI == ERR_OK) {
      printf("MessageID : %lu\n MessageSEQ : %lu PayloadData : \n",
             ReceivedMessage->MessageID, ReceivedMessage->MessageSEQ);
      for (int i = 0; i < ReceivedMessage->DLC; i++) {
        printf("%i, ", ReceivedMessage->PayloadData[i]);
      }
      printf("\n");
    }
  }
  return ErrorEPSI;
}
