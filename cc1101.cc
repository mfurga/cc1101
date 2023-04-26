#include "cc1101.h"

Status CC1101::begin() {
  pinMode(cs, OUTPUT);
  chipDeselect();

  SPI.begin();

  hardReset();
  delay(10);

  byte partnum = getChipPartNumber();
  byte version = getChipVersion();

  if (partnum != CC1101_PARTNUM || version != CC1101_VERSION) {
    return STATUS_ERROR_CHIP_NOT_FOUND;
  }

  setRegs();

  sendCmd(CC1101_CMD_IDLE);
  flushRxBuffer();
  flushTxBuffer();

  return STATUS_OK;
}

void CC1101::setRegs() {
  writeReg(CC1101_REG_MCSM0, CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_REG_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_REG_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
  writeReg(CC1101_REG_FREND0, CC1101_DEFVAL_FREND0);
  writeReg(CC1101_REG_PKTLEN, CC1101_DEFVAL_PKTLEN);
}

void CC1101::setModulation(CC1101_Modulation mod) {
  this->mod = mod;
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mod, 6, 4);
}

Status CC1101::setFrequency(double freq) {
  if (!((freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0))) {
    return STATUS_INVALID_PARAM;
  }

  this->freq = freq;
  sendCmd(CC1101_CMD_IDLE);

  uint32_t f = ((freq * 65536.0) / 26.0);
  writeReg(CC1101_REG_FREQ0, f & 0xff);
  writeReg(CC1101_REG_FREQ1, (f >> 8) & 0xff);
  writeReg(CC1101_REG_FREQ2, (f >> 16) & 0xff);

  Serial.printf("FREQ0=%02x\r\n", readReg(CC1101_REG_FREQ0));
  Serial.printf("FREQ1=%02x\r\n", readReg(CC1101_REG_FREQ1));
  Serial.printf("FREQ2=%02x\r\n", readReg(CC1101_REG_FREQ2));

  return STATUS_OK;
}

Status CC1101::setDataRate(double drate) {

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

void CC1101::transmit(byte data) {
  byte txFifoBytes;

  sendCmd(CC1101_CMD_IDLE);
  flushTxBuffer();

  writeReg(0x00, 0x06);

  byte paValues[2] = { 0x00, 0xc0 };
  writeRegBurst(CC1101_REG_PATABLE, paValues, 2);

  txFifoBytes = readReg(CC1101_REG_TXBYTES) & ~(1 << 7);

  byte sent[16];
  for (int i = 0; i < 16; i++) {
    sent[i] = data;
  }

  writeRegBurst(CC1101_REG_FIFO, sent, 16);
  sendCmd(CC1101_CMD_TX);

  Serial.printf("State before: %d\r\n", currentState);

  do {
    txFifoBytes = readReg(CC1101_REG_TXBYTES) & ~(1 << 7);
    //delayMicroseconds(300);
  } while (txFifoBytes > 0);


  Serial.printf("State after: %d\r\n", currentState);

  sendCmd(CC1101_CMD_IDLE);
  flushTxBuffer();
}

void CC1101::saveStatus(byte status) {
  currentState = (CC1101_State)((status >> 4) & 0b111);
}

void CC1101::hardReset() {
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

void CC1101::flushRxBuffer() {
  if (currentState != STATE_IDLE && currentState != STATE_RXFIFO_OVERFLOW) {
    return;
  }
  sendCmd(CC1101_CMD_FRX);
}

void CC1101::flushTxBuffer() {
  if (currentState != STATE_IDLE && currentState != STATE_TXFIFO_UNDERFLOW) {
    return;
  }
  sendCmd(CC1101_CMD_FTX);
}

byte CC1101::getChipPartNumber() {
  return readReg(CC1101_REG_PARTNUM);
}

byte CC1101::getChipVersion() {
  return readReg(CC1101_REG_VERSION);
}

byte CC1101::readReg(byte addr) {
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

void CC1101::readRegBurst(byte addr, byte *buff, size_t size) {
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

void CC1101::writeRegField(uint8_t addr, uint8_t data, uint8_t hi,
                           uint8_t lo) {
  data <<= lo;
  uint8_t current = readReg(addr);
  uint8_t mask = ((1 << (hi - lo + 1)) - 1) << lo;
  data = (current & ~mask) | (data & mask);
  writeReg(addr, data);
}

void CC1101::writeReg(uint8_t addr, uint8_t data) {
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

void CC1101::writeRegBurst(uint8_t addr, uint8_t *data, size_t size) {
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

void CC1101::sendCmd(byte addr) {
  byte header = CC1101_WRITE | (addr & 0b111111);

  Serial.printf("SPI CMD: %02x\r\n", header);

  SPI.beginTransaction(spiSettings);
  chipSelect();
  waitReady();

  saveStatus(SPI.transfer(header));

  chipDeselect();
  SPI.endTransaction();
}

void CC1101::chipSelect() {
  digitalWrite(cs, LOW);
}

void CC1101::chipDeselect() {
  digitalWrite(cs, HIGH);
}

void CC1101::waitReady() {
  while (digitalRead(MISO))
    ;
}

