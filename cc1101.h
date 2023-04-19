#pragma once

#include <Arduino.h>
#include <SPI.h>

#define PIN_UNUSED 0xff

#define CC1101_SPI_MAX_FREQ    6500000    /* 6.5 MHz */
#define CC1101_SPI_DATA_ORDER  MSBFIRST
#define CC1101_SPI_DATA_MODE   SPI_MODE0  /* clk low, leading edge */

#define CC1101_WRITE           0x00
#define CC1101_READ            0x80
#define CC1101_BURST           0x40

#define CC1101_PARTNUM         0x00
#define CC1101_VERSION         0x14

/* Command strobes */
#define CC1101_SRES            0x30

/* Registers */

/* Status registers */
#define CC1101_REG_PARTNUM     0x30
#define CC1101_REG_VERSION     0x31

enum Status {
  STATUS_OK = 0,

  STATUS_ERROR_CHIP_NOT_FOUND
};

class CC1101 {
 public:
  CC1101(byte cs, byte gd0 = PIN_UNUSED, byte gd2 = PIN_UNUSED)
    : cs(cs),
      gd0(gd0),
      gd2(gd2),
      spiSettings(CC1101_SPI_MAX_FREQ,
                  CC1101_SPI_DATA_ORDER,
                  CC1101_SPI_DATA_MODE) {}
  Status begin();

  byte getChipPartNumber();
  byte getChipVersion();

  byte readReg(byte addr);

 private:
  void chipSelect();
  void waitReady();
  void chipDeselect();

  void hardReset();

  byte cs, gd0, gd2;
  SPISettings spiSettings;
};

