#pragma once

#include <Arduino.h>
#include <SPI.h>

#define PIN_UNUSED                0xff

#define CC1101_SPI_MAX_FREQ       6500000    /* 6.5 MHz */
#define CC1101_SPI_DATA_ORDER     MSBFIRST
#define CC1101_SPI_DATA_MODE      SPI_MODE0  /* clk low, leading edge */

#define CC1101_FIFO_SIZE          64    /* 64 B */
#define CC1101_CRYSTAL_FREQ       26    /* 26 MHz */
#define CC1101_PKT_MAX_SIZE       61    /* 61 B, 1B for address, 2B for status */

#define CC1101_WRITE              0x00
#define CC1101_READ               0x80
#define CC1101_BURST              0x40

#define CC1101_PARTNUM            0x00
#define CC1101_VERSION            0x14

/* Command strobes */
#define CC1101_CMD_RES            0x30  /* Reset chip */
#define CC1101_CMD_RX             0x34  /* Enable RX */
#define CC1101_CMD_TX             0x35  /* Enable TX */
#define CC1101_CMD_IDLE           0x36  /* Enable IDLE */
#define CC1101_CMD_FRX            0x3a  /* Flush the RX FIFO buffer */
#define CC1101_CMD_FTX            0x3b  /* Flush the TX FIFO buffer */
#define CC1101_CMD_NOP            0x3d  /* No operation */

/* Registers */
#define CC1101_REG_SYNC1          0x04  /* Sync Word, High Byte */
#define CC1101_REG_SYNC0          0x05  /* Sync Word, Low Byte */
#define CC1101_REG_PKTLEN         0x06
#define CC1101_REG_PKTCTRL1       0x07
#define CC1101_REG_PKTCTRL0       0x08  /* Packet Automation Control */

#define CC1101_REG_FREQ2          0x0d
#define CC1101_REG_FREQ1          0x0e
#define CC1101_REG_MDMCFG4        0x10
#define CC1101_REG_MDMCFG3        0x11
#define CC1101_REG_MDMCFG2        0x12  /* Modem Configuration */
#define CC1101_REG_MDMCFG1        0x13
#define CC1101_REG_FREQ0          0x0f

#define CC1101_REG_MCSM2          0x16
#define CC1101_REG_MCSM1          0x17
#define CC1101_REG_MCSM0          0x18
#define CC1101_REG_FREND0         0x22  /* Front End TX Configuration */

#define CC1101_REG_PATABLE        0x3e
#define CC1101_REG_FIFO           0x3f

/* Status registers */
#define CC1101_REG_PARTNUM        0x30
#define CC1101_REG_VERSION        0x31
#define CC1101_REG_TXBYTES        0x3a
#define CC1101_REG_RXBYTES        0x3b
#define CC1101_REG_RCCTRL0_STATUS 0x3d

namespace CC1101 {

enum Status {
  STATUS_OK = 0,

  STATUS_LENGTH_TOO_BIG,
  STATUS_INVALID_PARAM,
  STATUS_ERROR_CHIP_NOT_FOUND
};

enum State {
  STATE_IDLE             = 0,  /* IDLE state */
  STATE_RX               = 1,  /* Receive mode */
  STATE_TX               = 2,  /* Transmit mode */
  STATE_FSTXON           = 3,  /* Fast TX ready */
  STATE_CALIBRATE        = 4,  /* Freq synthesizer calibration is running */
  STATE_SETTLING         = 5,  /* PLL is settling */
  STATE_RXFIFO_OVERFLOW  = 6,  /* RX FIFO has overflowed */
  STATE_TXFIFO_UNDERFLOW = 7,  /* TX FIFO has underflowed */
};

enum Modulation {
  MOD_2FSK    = 0,
  MOD_GFSK    = 1,
  MOD_ASK_OOK = 3,
  MOD_4FSK    = 4,
  MOD_MSK     = 7
};

enum SyncMode {
  SYNC_MODE_NO_PREAMBLE    = 0,  /* No preamble/sync */
  SYNC_MODE_15_16          = 1,  /* 15/16 sync word bits detected */
  SYNC_MODE_16_16          = 2,  /* 16/16 sync word bits detected */
  SYNC_MODE_30_32          = 3,  /* 30/32 sync word bits detected */
  SYNC_MODE_NO_PREAMBLE_CS = 4,  /* No preamble/sync, CS above threshold */
  SYNC_MODE_15_16_CS       = 5,  /* 15/16 + carrier-sense above threshold */
  SYNC_MODE_16_16_CS       = 6,  /* 16/16 + carrier-sense above threshold */
  SYNC_MODE_30_32_CS       = 7,  /* 30/32 + carrier-sense above threshold */
};

enum PacketLengthMode {
  PKT_LEN_MODE_FIXED    = 0,  /* Length configured in PKTLEN register */
  PKT_LEN_MODE_VARIABLE = 1,  /* Packet length put in the first byte */
  // TODO: PKT_LEN_MODE_INFINITE = 2,  /* Infinite packet length mode */
};

enum AddressFilteringMode {
  ADDR_FILTER_MODE_NONE = 0,          /* No address check */
  ADDR_FILTER_MODE_CHECK = 1,         /* Address check, no broadcast */
  ADDR_FILTER_MODE_CHECK_BC_0 = 2,    /* Address check, 0 broadcast */
  ADDR_FILTER_MODE_CHECK_BC_0_255 = 3 /* Address check, 0 and 255 broadcast */
};

class Radio {
 public:
  Radio(byte cs, byte gd0 = PIN_UNUSED, byte gd2 = PIN_UNUSED)
    : cs(cs),
      gd0(gd0),
      gd2(gd2),
      spiSettings(CC1101_SPI_MAX_FREQ,
                  CC1101_SPI_DATA_ORDER,
                  CC1101_SPI_DATA_MODE) {}

  Status begin(Modulation mod = MOD_ASK_OOK,
               double freq = 433.5,
               double drate = 4.0);

  uint8_t getChipPartNumber();
  uint8_t getChipVersion();

  void setModulation(Modulation mod);
  Status setFrequency(double freq);
  Status setDataRate(double drate);
  Status setRxBandwidth(double bw);

  void setOutputPower(int8_t power);

  /* Enable CRC calculation in TX and CRC check in RX. */
  void setCrc(bool enable);
  void setAddressFilteringMode(AddressFilteringMode mode);
  void setPacketLengthMode(PacketLengthMode mode, uint8_t length = 255);
  void setSyncMode(SyncMode mode);
  Status setPreambleLength(uint8_t length);
  void setSyncWord(uint8_t syncHi, uint8_t syncLo);

  Status transmit(uint8_t *data, size_t length, uint8_t addr = 0);
  Status receive(uint8_t *data, size_t length, uint8_t addr = 0);

 private:
  void chipSelect();
  void waitReady();
  void chipDeselect();

  uint8_t readRegField(uint8_t addr, uint8_t hi, uint8_t lo);
  uint8_t readReg(uint8_t addr);
  void readRegBurst(uint8_t addr, uint8_t *buff, size_t size);

  void writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo);
  void writeReg(uint8_t addr, uint8_t data);
  void writeRegBurst(uint8_t addr, uint8_t *data, size_t size);

  void sendCmd(byte addr);

  void setRegs();
  void hardReset();
  void flushRxBuffer();
  void flushTxBuffer();

  State getState();
  void saveStatus(byte status);

  uint8_t cs, gd0, gd2;
  SPISettings spiSettings;

  State currentState = STATE_IDLE;
  Modulation mod = MOD_2FSK;
  PacketLengthMode pktLenMode = PKT_LEN_MODE_FIXED;
  AddressFilteringMode addrFilterMode = ADDR_FILTER_MODE_NONE;

  double freq = 433.5;
  double drate = 4.0;
  int8_t power = 0;
};

}

