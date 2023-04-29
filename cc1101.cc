#include "cc1101.h"

using namespace CC1101;

Status Radio::begin(Modulation mod, double freq, double drate) {
  Status status;

  pinMode(cs, OUTPUT);
  chipDeselect();
  SPI.begin();

  hardReset();
  delay(10);

  uint8_t partnum = getChipPartNumber();
  uint8_t version = getChipVersion();
  if (partnum != CC1101_PARTNUM || version != CC1101_VERSION) {
    return STATUS_ERROR_CHIP_NOT_FOUND;
  }

  setRegs();

  setModulation(mod);

  if ((status = setFrequency(freq)) != STATUS_OK) {
    return status;
  }

  if ((status = setDataRate(drate)) != STATUS_OK) {
    return status;
  }

  setOutputPower(0);

  sendCmd(CC1101_CMD_IDLE);
  flushRxBuffer();
  flushTxBuffer();

  return STATUS_OK;
}

void Radio::setRegs() {
  /* Automatically calibrate when going from IDLE to RX or TX. */
  writeRegField(CC1101_REG_MCSM0, 1, 5, 4);

  /* Disable data whitening. */
  writeRegField(CC1101_REG_PKTCTRL0, 0, 6, 6);

  /*
    TODO: Enable append status. Two status bytes will be appended to the payload
    of the packet. The status bytes contain RSSI and LQI values, as well as
    CRC OK.
  */
  writeRegField(CC1101_REG_PKTCTRL1, 0, 2, 2);
}

void Radio::setModulation(Modulation mod) {
  this->mod = mod;
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mod, 6, 4);

  setOutputPower(this->power);
}

Status Radio::setFrequency(double freq) {
  if (!((freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0))) {
    return STATUS_INVALID_PARAM;
  }

  this->freq = freq;
  sendCmd(CC1101_CMD_IDLE);

  uint32_t f = ((freq * 65536.0) / CC1101_CRYSTAL_FREQ);
  writeReg(CC1101_REG_FREQ0, f & 0xff);
  writeReg(CC1101_REG_FREQ1, (f >> 8) & 0xff);
  writeReg(CC1101_REG_FREQ2, (f >> 16) & 0xff);

  setOutputPower(this->power);

  return STATUS_OK;
}

Status Radio::setDataRate(double drate) {

  static const double range[][2] = {
    [MOD_2FSK]    = {  0.6, 500.0 },  /* 0.6 - 500 kBaud */
    [MOD_GFSK]    = {  0.6, 250.0 },
    [2]           = {  0.0, 0.0   },  /* gap */
    [MOD_ASK_OOK] = {  0.6, 250.0 },
    [MOD_4FSK]    = {  0.6, 300.0 },
    [5]           = {  0.0, 0.0   },  /* gap */
    [6]           = {  0.0, 0.0   },  /* gap */
    [MOD_MSK]     = { 26.0, 500.0 }
  };

  if (drate < range[mod][0] || drate > range[mod][1]) {
    return STATUS_INVALID_PARAM;
  }

  this->drate = drate;

  uint32_t xosc = CC1101_CRYSTAL_FREQ * 1000;
  uint8_t e = log2((drate * (1 << 20)) / xosc);
  uint32_t m = round(drate * ((1 << 28) / (xosc * (1 << e))) - 256);

  if (m == 256) {
    m = 0;
    e++;
  }

  writeRegField(CC1101_REG_MDMCFG4, e, 3, 0);
  writeReg(CC1101_REG_MDMCFG3, (uint8_t)m);

  return STATUS_OK;
}

Status Radio::setRxBandwidth(double bw) {
  /*
    CC1101 supports the following channel filter bandwidths [kHz]:
    (assuming a 26 MHz crystal).

  \ E  0     1     2     3
  M +----------------------
  0 | 812 | 406 | 203 | 102
  1 | 650 | 335 | 162 |  81
  2 | 541 | 270 | 135 |  68
  3 | 464 | 232 | 116 |  58

  */

  int bwMin = (CC1101_CRYSTAL_FREQ * 1000) / (8 * (4 + 3) * (1 << 3));
  int bwMax = (CC1101_CRYSTAL_FREQ * 1000) / (8 * (4 + 0) * (1 << 0));

  if (bw < bwMin || bw > bwMax) {
    return STATUS_INVALID_PARAM;
  }

  uint8_t bestE = 0, bestM = 0;
  double diff = bwMax;

  for (uint8_t e = 0; e <= 3; e++) {
    for (uint8_t m = 0; m <= 3; m++) {
      double t = (double)(CC1101_CRYSTAL_FREQ * 1000) / (8 * (4 + m) * (1 << e));
      if (fabs(bw - t) < diff) {
        diff = fabs(bw - t);
        bestE = e;
        bestM = m;
      }
    }
  }

  writeRegField(CC1101_REG_MDMCFG4, bestE, 7, 6);
  writeRegField(CC1101_REG_MDMCFG4, bestM, 5, 4);

  return STATUS_OK;
}

void Radio::setOutputPower(int8_t power) {

  static const uint8_t powers[][8] = {
    [0 /* 315 Mhz */ ] = { 0x12, 0x0d, 0x1c, 0x34, 0x51, 0x85, 0xcb, 0xc2 },
    [1 /* 433 Mhz */ ] = { 0x12, 0x0e, 0x1d, 0x34, 0x60, 0x84, 0xc8, 0xc0 },
    [2 /* 868 Mhz */ ] = { 0x03, 0x0f, 0x1e, 0x27, 0x50, 0x81, 0xcb, 0xc2 },
    [3 /* 915 MHz */ ] = { 0x03, 0x0e, 0x1e, 0x27, 0x8e, 0xcd, 0xc7, 0xc0 }
  };

  uint8_t powerIdx, freqIdx;

  if (freq <= 348.0) {
    freqIdx = 0;
  } else if (freq <= 464.0) {
    freqIdx = 1;
  } else if (freq <= 855.0) {
    freqIdx = 2;
  } else {
    freqIdx = 3;
  }

  if (power <= -30) {
    powerIdx = 0;
  } else if (power <= -20) {
    powerIdx = 1;
  } else if (power <= -15) {
    powerIdx = 2;
  } else if (power <= -10) {
    powerIdx = 3;
  } else if (power <= 0) {
    powerIdx = 4;
  } else if (power <= 5) {
    powerIdx = 5;
  } else if (power <= 7) {
    powerIdx = 6;
  } else {
    powerIdx = 7;
  }

  this->power = power;

  if (mod == MOD_ASK_OOK) {
    /* No shaping. Use only the first 2 entries in the power table. */
    uint8_t data[2] = { 0x00, powers[freqIdx][powerIdx] };
    writeRegBurst(CC1101_REG_PATABLE, data, sizeof(data));
    writeRegField(CC1101_REG_FREND0, 1, 2, 0);  /* PA_POWER = 1 */
  } else {
    writeReg(CC1101_REG_PATABLE, powers[freqIdx][powerIdx]);
    writeRegField(CC1101_REG_FREND0, 0, 2, 0);  /* PA_POWER = 0 */
  }
}

Status Radio::setPreambleLength(uint8_t length) {
  uint8_t data;

  switch (length) {
    case 16:
      data = 0;
    break;
    case 24:
      data = 1;
    break;
    case 36:
      data = 2;
    break;
    case 48:
      data = 3;
    break;
    case 64:
      data = 4;
    break;
    case 96:
      data = 5;
    break;
    case 128:
      data = 6;
    break;
    case 192:
      data = 7;
    break;
    default:
      return STATUS_INVALID_PARAM;
  }

  writeRegField(CC1101_REG_MDMCFG1, data, 6, 4);
  return STATUS_OK;
}

void Radio::setSyncWord(uint8_t syncHi, uint8_t syncLo) {
  writeReg(CC1101_REG_SYNC1, syncHi);
  writeReg(CC1101_REG_SYNC0, syncLo);
}

void Radio::setSyncMode(SyncMode mode) {
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mode, 2, 0);
}

void Radio::setPacketLengthMode(PacketLengthMode mode, uint8_t length) {
  writeRegField(CC1101_REG_PKTCTRL0, (uint8_t)mode, 1, 0);
  if (mode == PKT_LEN_MODE_FIXED) {
    writeReg(CC1101_REG_PKTLEN, length);
  }
}

void Radio::setAddressFilteringMode(AddressFilteringMode mode) {
  writeRegField(CC1101_REG_PKTCTRL1, (uint8_t)mode, 1, 0);
}

void Radio::setCrc(bool enable) {
  writeRegField(CC1101_REG_PKTCTRL0, (uint8_t)enable, 2, 2);
}


Status Radio::transmit(uint8_t *data, size_t length, uint8_t addr) {
  uint8_t bytesSent = 0;

  if (length > 255) {
    return STATUS_LENGTH_TOO_BIG;
  }

  sendCmd(CC1101_CMD_IDLE);
  flushTxBuffer();

  switch (pktLenMode) {
    case PKT_LEN_MODE_FIXED:
      writeReg(CC1101_REG_PKTLEN, (uint8_t)length);
    break;
    case PKT_LEN_MODE_VARIABLE:
      writeReg(CC1101_REG_FIFO, (uint8_t)length);
      bytesSent++;
    break;
  }

  if (addrFilterMode != ADDR_FILTER_MODE_NONE) {
    writeReg(CC1101_REG_FIFO, addr);
    bytesSent++;
  }

  uint8_t l = min((uint8_t)length, CC1101_FIFO_SIZE - bytesSent);
  writeRegBurst(CC1101_REG_FIFO, data, l);
  bytesSent += l;

  sendCmd(CC1101_CMD_TX);

  while (bytesSent < length) {
    uint8_t bytesInFifo = readRegField(CC1101_REG_TXBYTES, 6, 0);

    if (bytesInFifo < CC1101_FIFO_SIZE) {
      uint8_t bytesToWrite = min((uint8_t)length - bytesSent,
                                 CC1101_FIFO_SIZE - bytesInFifo);
      writeRegBurst(CC1101_REG_FIFO, data + bytesSent, bytesToWrite);
      bytesSent += bytesToWrite;
    }
  }

  while (getState() != STATE_IDLE) {
    delayMicroseconds(100);
  }

  return STATUS_OK;
}

Status Radio::receive(uint8_t *data, size_t length, uint8_t addr) {
  if (length > 255) {
    return STATUS_LENGTH_TOO_BIG;
  }

  sendCmd(CC1101_CMD_IDLE);
  flushRxBuffer();

  sendCmd(CC1101_CMD_RX);
  while (getState() != STATE_RX) {
    delayMicroseconds(800);
  }

  uint8_t pktLength = 0;
  switch (pktLenMode) {
    case PKT_LEN_MODE_FIXED:
      Serial.println("FIXED");
      pktLength = readReg(CC1101_REG_PKTLEN);
    break;
    case PKT_LEN_MODE_VARIABLE:
      Serial.println("VARIABLE");
      /* Indicates the maximum packet length allowed. */
      writeReg(CC1101_REG_PKTLEN, (uint8_t)length);

      /* Wait for length byte. */
      while (pktLength == 0) {
        pktLength = readReg(CC1101_REG_FIFO);
      }
    break;
  }

  if (pktLength > length) {
    Serial.println("ERROR LEN TO SMALL");
    // TODO: Error.
    for (;;);
  }

  if (addrFilterMode == ADDR_FILTER_MODE_CHECK) {
    Serial.println("ADDR CHECK");
    (void)readReg(CC1101_REG_FIFO);
  }

  uint8_t bytesInFifo = readRegField(CC1101_REG_RXBYTES, 6, 0);
  uint8_t bytesRead = 0;

  Serial.printf("pktLength: %d\r\n", pktLength);

  /*
    For packet lengths less than 64 bytes it is recommended to wait until
    the complete packet has been received before reading it out of the RX FIFO.
  */
  if (pktLength <= CC1101_PKT_MAX_SIZE) {
    while (bytesInFifo < pktLength) {
      delayMicroseconds(15 * 8);
      bytesInFifo = readRegField(CC1101_REG_RXBYTES, 6, 0);
    }
  }

  // FIXME
  while (bytesRead < pktLength) {
    while (bytesInFifo == 0) {
      delayMicroseconds(15);
      bytesInFifo = readRegField(CC1101_REG_RXBYTES, 6, 0);
    }

    uint8_t bytesToRead = min(pktLength - bytesRead, bytesInFifo);
    readRegBurst(CC1101_REG_FIFO, data + bytesRead, bytesToRead);
    bytesRead += bytesToRead;
  }

  while (getState() != STATE_IDLE) {
    delayMicroseconds(100);
  }

  Serial.printf("state: %d\r\n", currentState);

  return STATUS_OK;
}

State Radio::getState() {
  sendCmd(CC1101_CMD_NOP);
  return currentState;
}

void Radio::saveStatus(byte status) {
  currentState = (State)((status >> 4) & 0b111);
}

void Radio::hardReset() {
  SPI.beginTransaction(spiSettings);

  chipDeselect();
  delayMicroseconds(5);
  chipSelect();
  delayMicroseconds(5);
  chipDeselect();
  delayMicroseconds(40);

  chipSelect();
  waitReady();

  SPI.transfer(CC1101_CMD_RES);

  waitReady();
  chipDeselect();

  SPI.endTransaction();
}

void Radio::flushRxBuffer() {
  if (currentState != STATE_IDLE && currentState != STATE_RXFIFO_OVERFLOW) {
    return;
  }
  sendCmd(CC1101_CMD_FRX);
}

void Radio::flushTxBuffer() {
  if (currentState != STATE_IDLE && currentState != STATE_TXFIFO_UNDERFLOW) {
    return;
  }
  sendCmd(CC1101_CMD_FTX);
}

uint8_t Radio::getChipPartNumber() {
  return readReg(CC1101_REG_PARTNUM);
}

uint8_t Radio::getChipVersion() {
  return readReg(CC1101_REG_VERSION);
}

uint8_t Radio::readRegField(uint8_t addr, uint8_t hi, uint8_t lo) {
  return (readReg(addr) >> lo) & ((1 << (hi - lo + 1)) - 1);
}

uint8_t Radio::readReg(uint8_t addr) {
  byte header = CC1101_READ | (addr & 0b111111);

  if (addr >= CC1101_REG_PARTNUM && addr <= CC1101_REG_RCCTRL0_STATUS) {
    /* Status registers - access with the burst bit on. */
    header |= CC1101_BURST;
  }

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));
  byte data = SPI.transfer(0x00);

  chipDeselect();
  SPI.endTransaction();
  return data;
}

void Radio::readRegBurst(uint8_t addr, uint8_t *buff, size_t size) {
  byte header = CC1101_READ | CC1101_BURST | (addr & 0b111111);

  if (addr >= CC1101_REG_PARTNUM && addr <= CC1101_REG_RCCTRL0_STATUS) {
    /* Status registers cannot be accessed with the burst option. */
    return;
  }

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));
  for (size_t i = 0; i < size; i++) {
    buff[i] = SPI.transfer(0x00);
  }

  chipDeselect();
  SPI.endTransaction();
}

void Radio::writeRegField(uint8_t addr, uint8_t data, uint8_t hi,
                           uint8_t lo) {
  data <<= lo;
  uint8_t current = readReg(addr);
  uint8_t mask = ((1 << (hi - lo + 1)) - 1) << lo;
  data = (current & ~mask) | (data & mask);
  writeReg(addr, data);
}

void Radio::writeReg(uint8_t addr, uint8_t data) {
  if (addr >= CC1101_REG_PARTNUM && addr <= CC1101_REG_RCCTRL0_STATUS) {
    /* Status registers are read-only. */
    return;
  }

  byte header = CC1101_WRITE | (addr & 0b111111);

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));
  saveStatus(SPI.transfer(data));

  chipDeselect();
  SPI.endTransaction();
}

void Radio::writeRegBurst(uint8_t addr, uint8_t *data, size_t size) {
  if (addr >= CC1101_REG_PARTNUM && addr <= CC1101_REG_RCCTRL0_STATUS) {
    /* Status registers are read-only. */
    return;
  }

  byte header = CC1101_WRITE | CC1101_BURST | (addr & 0b111111);

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));
  for (size_t i = 0; i < size; i++) {
    saveStatus(SPI.transfer(data[i]));
  }

  chipDeselect();
  SPI.endTransaction();
}

void Radio::sendCmd(byte addr) {
  byte header = CC1101_WRITE | (addr & 0b111111);

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));

  chipDeselect();
  SPI.endTransaction();
}

void Radio::chipSelect() {
  digitalWrite(cs, LOW);
}

void Radio::chipDeselect() {
  digitalWrite(cs, HIGH);
}

void Radio::waitReady() {
  while (digitalRead(MISO))
    ;
}

