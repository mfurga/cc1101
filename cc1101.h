#pragma once

#include <Arduino.h>
#include <SPI.h>

#define PIN_UNUSED 0xff

#define CC1101_SPI_MAX_FREQ    6500000    /* 6.5 MHz */
#define CC1101_SPI_DATA_ORDER  MSBFIRST
#define CC1101_SPI_DATA_MODE   SPI_MODE0  /* clk low, leading edge */

#define CC1101_CRYSTAL_FREQ    26         /* 26 MHz */


#define CC1101_WRITE           0x00
#define CC1101_READ            0x80
#define CC1101_BURST           0x40

#define CC1101_PARTNUM         0x00
#define CC1101_VERSION         0x14

/* Command strobes */
#define CC1101_CMD_RES         0x30  /* Reset chip */
#define CC1101_CMD_RX          0x34  /* Enable RX */
#define CC1101_CMD_TX          0x35  /* Enable TX */
#define CC1101_CMD_IDLE        0x36  /* Enable IDLE */
#define CC1101_CMD_FRX         0x3a  /* Flush the RX FIFO buffer */
#define CC1101_CMD_FTX         0x3b  /* Flush the TX FIFO buffer */
#define CC1101_CMD_NOP         0x3d  /* No operation */

/* Registers */
#define CC1101_REG_PKTLEN      0x06
#define CC1101_REG_PKTCTRL1    0x07
#define CC1101_REG_PKTCTRL0    0x08  /* Packet Automation Control */

#define CC1101_REG_FREQ2       0x0d
#define CC1101_REG_FREQ1       0x0e
#define CC1101_REG_MDMCFG4     0x10
#define CC1101_REG_MDMCFG3     0x11
#define CC1101_REG_MDMCFG2     0x12  /* Modem Configuration */
#define CC1101_REG_FREQ0       0x0f

#define CC1101_REG_MCSM2       0x16
#define CC1101_REG_MCSM1       0x17
#define CC1101_REG_MCSM0       0x18
#define CC1101_REG_FREND0      0x22  /* Front End TX Configuration */

#define CC1101_REG_PATABLE     0x3e
#define CC1101_REG_FIFO        0x3f

/* Status registers */
#define CC1101_REG_PARTNUM        0x30
#define CC1101_REG_VERSION        0x31
#define CC1101_REG_TXBYTES        0x3a
#define CC1101_REG_RXBYTES        0x3b
#define CC1101_REG_RCCTRL0_STATUS 0x3d

/* Default registers values */

/*
  7:6 - Not used.
  5:4 - FS_AUTOCAL[1:0] - Automatically calibrate when going to RX or TX
                          or back to IDLE
    00 - Never (manually calibrate using SCAL strobe)
    01 - When going from IDLE to RX or TX (or FSTXON)
    10 - When going from RX or TX back to IDLE automatically
    11 - Every 4th time when going from RX or TX to IDLE automatically
*/
#define CC1101_DEFVAL_MCSM0    0b00010100

/* Disable CRC, fixed packet length mode */
#define CC1101_DEFVAL_PKTCTRL0 0b00000000
#define CC1101_DEFVAL_PKTCTRL1 0b00000100

#define CC1101_DEFVAL_MDMCFG2  0b00110000

#define CC1101_DEFVAL_FREND0   0b00010001

#define CC1101_DEFVAL_PKTLEN   16

enum Status {
  STATUS_OK = 0,

  STATUS_INVALID_PARAM,
  STATUS_ERROR_CHIP_NOT_FOUND
};

enum CC1101_State {
  STATE_IDLE = 0,         /* IDLE state */
  STATE_RX,               /* Receive mode */
  STATE_TX,               /* Transmit mode */
  STATE_FSTXON,           /* Fast TX ready */
  STATE_CALIBRATE,        /* Frequency synthesizer calibration is running */
  STATE_SETTLING,         /* PLL is settling */
  STATE_RXFIFO_OVERFLOW,  /* RX FIFO has overflowed */
  STATE_TXFIFO_UNDERFLOW, /* TX FIFO has underflowed */
};

enum CC1101_Modulation {
  MOD_2FSK    = 0,
  MOD_GFSK    = 1,
  MOD_ASK_OOK = 3,
  MOD_4FSK    = 4,
  MOD_MSK     = 7
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

  void setModulation(CC1101_Modulation mod);
  Status setFrequency(double freq);
  Status setDataRate(double drate);

  void transmit(byte data);
 private:
  void chipSelect();
  void waitReady();
  void chipDeselect();

  byte readReg(byte addr);
  void readRegBurst(byte addr, byte *buff, size_t size);

  void writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo);
  void writeReg(uint8_t addr, uint8_t data);
  void writeRegBurst(uint8_t addr, uint8_t *data, size_t size);

  void sendCmd(byte addr);

  void setRegs();
  void hardReset();
  void flushRxBuffer();
  void flushTxBuffer();

  void saveStatus(byte status);

  byte cs, gd0, gd2;
  SPISettings spiSettings;

  CC1101_State currentState = STATE_IDLE;
  CC1101_Modulation mod = MOD_2FSK;
  double freq;
  double drate;
};

