#include "cc1101.h"

#if defined(ESP32) && \
    !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32C3)
#include <driver/gpio.h>
#endif

#define log2(x) (log(x) / log(2))

using namespace CC1101;

Status Radio::begin(Modulation mod, double freq, double drate) {
  Status status;

  pinMode(cs, OUTPUT);

  if (gd0 != PIN_UNUSED) {
    pinMode(gd0, INPUT);
  }

  if (gd2 != PIN_UNUSED) {
    pinMode(gd2, INPUT);
  }

  chipDeselect();
  #ifdef ESP32
  SPI.begin(clk, miso, mosi, cs);
  #else
  SPI.begin();
  #endif

  hardReset();
  delay(10);

  uint8_t partnum = getChipPartNumber();
  uint8_t version = getChipVersion();
  if (partnum != CC1101_PARTNUM ||
      (version != CC1101_VERSION && version != CC1101_VERSION_LEGACY)) {
    return STATUS_CHIP_NOT_FOUND;
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

  setState(STATE_IDLE);
  flushRxBuffer();
  flushTxBuffer();

  return STATUS_OK;
}

void Radio::setRegs() {
  /* Automatically calibrate when going from IDLE to RX or TX. */
  writeRegField(CC1101_REG_MCSM0, 1, 5, 4);

  /* Return to IDLE after a packet is received (MCSM1.RXOFF_MODE = IDLE) */
  writeRegField(CC1101_REG_MCSM1, 0, 3, 2);

  /* Append the 2 status bytes (RSSI + LQI/CRC_OK) to every packet and never
     auto-flush the RX FIFO on CRC error; Both are assumptions receive() relies on. */
  writeRegField(CC1101_REG_PKTCTRL1, 1, 2, 2);  /* APPEND_STATUS = 1 */
  writeRegField(CC1101_REG_PKTCTRL1, 0, 3, 3);  /* CRC_AUTOFLUSH = 0 */

  /* Disable data whitening. */
  setDataWhitening(false);
}

void Radio::setModulation(Modulation mod) {
  this->mod = mod;
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mod, 6, 4);

  if (mod == MOD_ASK_OOK) {
    writeReg(CC1101_REG_AGCCTRL2, 0x07);  /* MAGN_TARGET (DN022: 0x03-0x07) */
    writeReg(CC1101_REG_AGCCTRL1, 0x00);  /* AGC_LNA_PRIORITY = 0           */
    writeReg(CC1101_REG_AGCCTRL0, 0x91);  /* 8 dB ASK decision boundary     */
  } else {
    writeReg(CC1101_REG_AGCCTRL2, 0x03);  /* reset defaults                 */
    writeReg(CC1101_REG_AGCCTRL1, 0x40);
    writeReg(CC1101_REG_AGCCTRL0, 0x91);
  }

  setOutputPower(this->power);

  if (mod == MOD_MSK || mod == MOD_4FSK) {
    setManchester(false);
  }
}

Status Radio::setFrequency(double freq) {
  if (!((freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0))) {
    return STATUS_INVALID_PARAM;
  }

  this->freq = freq;
  setState(STATE_IDLE);

  uint32_t f = ((freq * 65536.0) / CC1101_CRYSTAL_FREQ);
  writeReg(CC1101_REG_FREQ0, f & 0xff);
  writeReg(CC1101_REG_FREQ1, (f >> 8) & 0xff);
  writeReg(CC1101_REG_FREQ2, (f >> 16) & 0xff);

  setOutputPower(this->power);

  return STATUS_OK;
}

Status Radio::setFrequencyDeviation(double dev) {
  double xosc = CC1101_CRYSTAL_FREQ * 1000;

  double devMin = (xosc / ((uint32_t)1 << 17)) * (8 + 0) * 1;
  double devMax = (xosc / ((uint32_t)1 << 17)) * (8 + 7) * (1 << 7);

  if (dev < devMin || dev > devMax) {
    return STATUS_INVALID_PARAM;
  }

  uint8_t bestE = 0, bestM = 0;
  double diff = devMax;

  for (uint8_t e = 0; e <= 7; e++) {
    for (uint8_t m = 0; m <= 7; m++) {
      double t = (xosc / (double)((uint32_t)1 << 17)) * (8 + m) * (double)((uint32_t)1 << e);
      if (fabs(dev - t) < diff) {
        diff = fabs(dev - t);
        bestE = e;
        bestM = m;
      }
    }
  }

  writeRegField(CC1101_REG_DEVIATN, bestM, 2, 0);
  writeRegField(CC1101_REG_DEVIATN, bestE, 6, 4);

  return STATUS_OK;
}

void Radio::setChannel(uint8_t ch) {
  writeReg(CC1101_REG_CHANNR, ch);
}

Status Radio::setChannelSpacing(double sp) {
  double xosc = CC1101_CRYSTAL_FREQ * 1000;

  double spMin = (xosc / (double)((uint32_t)1 << 18)) * (256. + 0.) * 1.;
  double spMax = (xosc / (double)((uint32_t)1 << 18)) * (256. + 255.) * 8.;

  if (sp < spMin || sp > spMax) {
    return STATUS_INVALID_PARAM;
  }

  uint8_t bestE = 0, bestM = 0;
  double diff = spMax;

  for (uint8_t e = 0; e <= 3; e++) {
    for (uint16_t m = 0; m <= 255; m++) {
      double t = (xosc / (double)((uint32_t)1 << 18)) * (256. + m) * (double)((uint32_t)1 << e);
      if (fabs(sp - t) < diff) {
        diff = fabs(sp - t);
        bestE = e;
        bestM = m;
      }
    }
  }

  writeReg(CC1101_REG_MDMCFG0, bestM);
  writeRegField(CC1101_REG_MDMCFG1, bestE, 1, 0);

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
  uint8_t e = log2((drate * (double)((uint32_t)1 << 20)) / xosc);
  uint32_t m = round(drate * ((double)((uint32_t)1 << (28 - e)) / xosc) - 256.);

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

  double bwMin = (double)(CC1101_CRYSTAL_FREQ * 1000) / (8 * (4 + 3) * (1 << 3));
  double bwMax = (double)(CC1101_CRYSTAL_FREQ * 1000) / (8 * (4 + 0) * (1 << 0));

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
  } else if (freq <= 891.5) {
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
    case 32:
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

void Radio::setSyncWord(uint16_t sync) {
  writeReg(CC1101_REG_SYNC1, sync >> 8);
  writeReg(CC1101_REG_SYNC0, sync & 0xff);
}

void Radio::setSyncMode(SyncMode mode) {
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mode, 2, 0);
}

void Radio::setPacketLengthMode(PacketLengthMode mode, uint8_t length) {
  this->pktLenMode = mode;
  this->pktLen = length;

  writeRegField(CC1101_REG_PKTCTRL0, (uint8_t)mode, 1, 0);

  switch (mode) {
    case PKT_LEN_MODE_FIXED:
      writeReg(CC1101_REG_PKTLEN, length);
    break;
    case PKT_LEN_MODE_VARIABLE:
      /* Indicates the maximum packet length allowed. */
      writeReg(CC1101_REG_PKTLEN, length);
    break;
  }
}

void Radio::setAddressFilteringMode(AddressFilteringMode mode) {
  this->addrFilterMode = mode;

  writeRegField(CC1101_REG_PKTCTRL1, (uint8_t)mode, 1, 0);
}

void Radio::setCrc(bool enable) {
  writeRegField(CC1101_REG_PKTCTRL0, (uint8_t)enable, 2, 2);
}

void Radio::setDataWhitening(bool enable) {
  writeRegField(CC1101_REG_PKTCTRL0, (uint8_t)enable, 6, 6);
}

Status Radio::setManchester(bool enable) {
  if (enable && (this->mod == MOD_MSK || this->mod == MOD_4FSK || this->fec)) {
    return STATUS_BAD_STATE;
  }

  this->manchester = enable;
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)enable, 3, 3);
  return STATUS_OK;
}

Status Radio::setFEC(bool enable) {
  if (enable && (this->pktLenMode != PKT_LEN_MODE_FIXED || this->manchester)) {
    return STATUS_BAD_STATE;
  }

  this->fec = enable;
  writeRegField(CC1101_REG_MDMCFG1, (uint8_t)enable, 7, 7);
  return STATUS_OK;
}

int8_t Radio::getRSSI() {
  return ((int8_t)this->rssi / 2) - 74;
}

uint8_t Radio::getLQI() {
  return this->lqi;
}

Status Radio::abortTransmit() {
  bool underflow = txFifoUnderflowed();
  setState(STATE_IDLE);
  flushTxBuffer();
  return underflow ? STATUS_TXFIFO_UNDERFLOW : STATUS_TIMEOUT;
}

Status Radio::transmit(uint8_t *data, size_t length, uint8_t addr) {
  size_t curPktLen = length;

  if (addrFilterMode != ADDR_FILTER_MODE_NONE) {
    curPktLen++;
  }

  if (curPktLen > 255) {
    return STATUS_LENGTH_TOO_BIG;
  }

  if (pktLenMode == PKT_LEN_MODE_FIXED) {
    if (curPktLen < this->pktLen) {
      return STATUS_LENGTH_TOO_SMALL;
    }
    if (curPktLen > this->pktLen) {
      return STATUS_LENGTH_TOO_BIG;
    }
  }

  setState(STATE_IDLE);
  flushTxBuffer();

  uint8_t headerBytes = 0;

  if (pktLenMode == PKT_LEN_MODE_VARIABLE) {
    writeReg(CC1101_REG_FIFO, (uint8_t)curPktLen);
    headerBytes++;
  }

  if (addrFilterMode != ADDR_FILTER_MODE_NONE) {
    writeReg(CC1101_REG_FIFO, addr);
    headerBytes++;
  }

  /*
    Fill the FIFO with the first chunk of payload (after the header bytes) and
    start transmitting, then keep topping it up as the chip drains it.
  */
  uint8_t firstChunk = min((uint8_t)length, (uint8_t)(CC1101_FIFO_SIZE - headerBytes));
  writeRegBurst(CC1101_REG_FIFO, data, firstChunk);
  size_t dataWritten = firstChunk;

  setState(STATE_TX);

  while (dataWritten < length) {
    uint8_t spaceInFifo = waitForSpaceInFifo(1);
    if (spaceInFifo == 0) {
      return abortTransmit();
    }

    uint8_t bytesToWrite = min((uint8_t)(length - dataWritten), spaceInFifo);
    writeRegBurst(CC1101_REG_FIFO, data + dataWritten, bytesToWrite);
    dataWritten += bytesToWrite;
  }

  /*
    The whole packet is in the FIFO; wait for the chip to transmit it and
    return to IDLE on its own (MCSM1.TXOFF_MODE = IDLE).
  */
  unsigned long start = millis();
  while (getState() != STATE_IDLE) {
    if (txFifoUnderflowed() || millis() - start > CC1101_XMIT_TIMEOUT_MS) {
      return abortTransmit();
    }
    delayMicroseconds(50);
    yield();
  }

  return STATUS_OK;
}

Status Radio::abortReceive() {
  bool overflow = rxFifoOverflowed();
  setState(STATE_IDLE);
  flushRxBuffer();
  return overflow ? STATUS_RXFIFO_OVERFLOW : STATUS_TIMEOUT;
}

Status Radio::receive(uint8_t *data, size_t length, size_t *read, uint8_t addr) {
  if (read != nullptr) {
    *read = 0;
  }

  if (length > 255) {
    return STATUS_LENGTH_TOO_BIG;
  }

  writeReg(CC1101_REG_ADDR, addr);

  setState(STATE_IDLE);
  flushRxBuffer();
  setState(STATE_RX);

  uint8_t headerBytes = 0;
  uint8_t dataLength = this->pktLen;

  if (pktLenMode == PKT_LEN_MODE_VARIABLE) {
    if (waitForBytesInFifo() == 0) {
      return abortReceive();
    }
    dataLength = readReg(CC1101_REG_FIFO);
    headerBytes++;
  }

  if (addrFilterMode != ADDR_FILTER_MODE_NONE && dataLength > 0) {
    if (waitForBytesInFifo() == 0) {
      return abortReceive();
    }
    (void)readReg(CC1101_REG_FIFO);
    headerBytes++;
    dataLength--;
  }

  if (dataLength > length) {
    setState(STATE_IDLE);
    flushRxBuffer();
    return STATUS_LENGTH_TOO_SMALL;
  }

  /*
    For packet lengths less than 64 bytes it is recommended to wait until
    the complete packet has been received before reading it out of the RX FIFO.
    Include the 2 appended status bytes (RSSI + CRC_OK|LQI) in the count.
  */
  uint16_t fullPacket = (uint16_t)dataLength + 2;
  if (fullPacket <= (CC1101_FIFO_SIZE - headerBytes)) {
    if (waitForBytesInFifo((uint8_t)fullPacket) == 0) {
      return abortReceive();
    }
  }

  uint8_t dataRead = 0;
  while (dataRead < dataLength) {
    uint8_t remaining = dataLength - dataRead;

    uint8_t bytesInFifo = waitForBytesInFifo(2);
    if (bytesInFifo == 0) {
      return abortReceive();
    }

    /*
      Per the datasheet the RX FIFO must never be emptied before the last byte
      of the packet has been received, otherwise the last read byte may be
      duplicated. Keep one byte back until the whole packet (payload + the 2
      appended status bytes) is in the FIFO.
    */
    bool fullPacketInFifo = (uint16_t)bytesInFifo >= (uint16_t)remaining + 2;
    uint8_t available = fullPacketInFifo ? bytesInFifo : (uint8_t)(bytesInFifo - 1);
    uint8_t bytesToRead = min(remaining, available);

    readRegBurst(CC1101_REG_FIFO, data + dataRead, bytesToRead);
    dataRead += bytesToRead;
  }

  if (waitForBytesInFifo(2) == 0) {
    return abortReceive();
  }

  this->rssi = readReg(CC1101_REG_FIFO);
  uint8_t v = readReg(CC1101_REG_FIFO);
  this->lqi = v & 0x7f;
  bool crc_ok = (v >> 7) & 1;

  setState(STATE_IDLE);
  flushRxBuffer();

  if (read != nullptr) {
    *read = dataLength;
  }

  return crc_ok ? STATUS_OK : STATUS_CRC_MISMATCH;
}

/*
  Read the NUM_*BYTES field of a FIFO byte-count status register repeatedly until
  the same value is returned twice, per datasheet errata (SWRZ020E).
*/
uint8_t Radio::readFifoByteCount(uint8_t addr) {
  uint8_t a = readRegField(addr, 6, 0);
  uint8_t b;
  for (uint8_t i = 0; i < CC1101_FIFO_BYTES_MAX_READS; i++) {
    b = a;
    a = readRegField(addr, 6, 0);
    if (a == b) {
      break;
    }
  }
  return a;
}

/*
  Reliable RX FIFO overflow check. RXBYTES.RXFIFO_OVERFLOW is a single-bit
  field, which the SPI read-synchronization errata (SWRZ020E) lists as immune
  to corruption - unlike the status-byte STATE field captured on every SPI transfer.
*/
bool Radio::rxFifoOverflowed() {
  return readRegField(CC1101_REG_RXBYTES, 7, 7) != 0;
}

uint8_t Radio::waitForBytesInFifo(uint8_t minBytes) {
  unsigned long start = millis();
  while (true) {
    if (rxFifoOverflowed()) {
      return 0;
    }
    uint8_t bytesInFifo = readFifoByteCount(CC1101_REG_RXBYTES);
    if (bytesInFifo >= minBytes) {
      return bytesInFifo;
    }
    if (millis() - start > CC1101_RECV_TIMEOUT_MS) {
      return 0;
    }
    delayMicroseconds(15);
    yield();
  }
}

/* Same as rxFifoOverflowed() */
bool Radio::txFifoUnderflowed() {
  return readRegField(CC1101_REG_TXBYTES, 7, 7) != 0;
}

uint8_t Radio::waitForSpaceInFifo(uint8_t minSpace) {
  unsigned long start = millis();
  while (true) {
    if (txFifoUnderflowed()) {
      return 0;
    }
    uint8_t bytesInFifo = readFifoByteCount(CC1101_REG_TXBYTES);
    uint8_t spaceInFifo =
        (bytesInFifo >= CC1101_FIFO_SIZE) ? 0 : (CC1101_FIFO_SIZE - bytesInFifo);
    if (spaceInFifo >= minSpace) {
      return spaceInFifo;
    }
    if (millis() - start > CC1101_XMIT_TIMEOUT_MS) {
      return 0;
    }
    delayMicroseconds(15);
    yield();
  }
}

void Radio::receiveCallback(void (*func)(void)) {
  /*
    Associated to the RX FIFO: Asserts when RX FIFO is filled at or above
    the RX FIFO threshold or the end of packet is reached. De-asserts when
    the RX FIFO is empty.
  */
  writeRegField(CC1101_REG_IOCFG0, 1, 5, 0);

  // TODO: Move to other method.
  flushRxBuffer();
  setState(STATE_RX);

  recvCallback = true;
  attachInterrupt(digitalPinToInterrupt(gd0), func, RISING);
}

State Radio::getState() {
  sendCmd(CC1101_CMD_NOP);
  return currentState;
}

void Radio::setState(State state) {
  switch (state) {
    case STATE_IDLE:
      sendCmd(CC1101_CMD_IDLE);
    break;
    case STATE_TX:
      sendCmd(CC1101_CMD_TX);
    break;
    case STATE_RX:
      sendCmd(CC1101_CMD_RX);
    break;
    default:
      /* Not supported. */
      return;
  }

  while (getState() != state) {
    delayMicroseconds(100);
    yield();
  }
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
  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  // ESP32C3/S3 does not allow a pin to be polled whilst it is attached to the
  // SPI peripheral. This will hang forever.
  // Fortunately, the CC1101 datasheet (pp29-30) states that MISO immediately
  // goes low on CS unless in a low power mode. As this library does not (yet)
  // support low power modes, we can safely return immediately.
  return;
  #elif defined(ESP32)
  // On other ESP32 variants MISO is routed to the SPI peripheral through the
  // GPIO matrix, so the Arduino digitalRead() HAL logs "IO N is not set as
  // GPIO" warnings.
  uint8_t pin = (miso == PIN_UNUSED) ? MISO : miso;
  while (gpio_get_level((gpio_num_t)pin))
    ;
  #else
  while (digitalRead(MISO))
    ;
  #endif
}
