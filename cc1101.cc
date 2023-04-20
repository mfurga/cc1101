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

  debug();

  sendCmd(CC1101_CMD_IDLE);
  flushRxBuffer();
  flushTxBuffer();

  return STATUS_OK;
}

void CC1101::setRegs() {
  writeReg(CC1101_REG_MCSM0, CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_REG_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
}

void CC1101::setFrequency(double frequency) {
  uint32_t freq = (uint32_t)((frequency * 65536.0) / 26.0);

  sendCmd(CC1101_CMD_IDLE);

  writeReg(CC1101_REG_FREQ0, freq & 0xff);
  writeReg(CC1101_REG_FREQ1, (freq >> 8) & 0xff);
  writeReg(CC1101_REG_FREQ2, (freq >> 16) & 0xff);
}

void CC1101::transmit(byte data) {
  byte txFifoBytes;

  sendCmd(CC1101_CMD_IDLE);
  flushTxBuffer();

  byte paValues[2] = { 0x00, 0xc0 };
  writeRegBurst(CC1101_REG_PATABLE, paValues, 2);

  writeReg(CC1101_REG_FREND0, 0b00010001);

  writeReg(CC1101_REG_MDMCFG2, CC1101_DEFVAL_MDMCFG2);

  txFifoBytes = readReg(CC1101_REG_TXBYTES) & ~(1 << 7);
  Serial.printf("Bytes in TX FIFO: %d\r\n", txFifoBytes);

  writeReg(CC1101_REG_FIFO, data);
  sendCmd(CC1101_CMD_TX);

  do {
    txFifoBytes = readReg(CC1101_REG_TXBYTES) & ~(1 << 7);
  } while (txFifoBytes > 0);

}

void CC1101::saveStatus(byte status) {
  currentState = (CC1101_State)((status >> 3) & 0b111);
}

void CC1101::debug() {
  Serial.printf(
    "=== CC1101 DEBUG:\r\n"
    "  state: %x\r\n",
    currentState
  );

  Serial.println();
}

/*
byte CC1101::getStatusByte() {
  return sendCmd(CC1101_CMD_NOP);
}
*/

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

  //Serial.printf("DEBUG: Reading reg %02x = %02x\r\n", addr, data);

  chipDeselect();
  SPI.endTransaction();
  return data;
}

void CC1101::writeReg(byte addr, byte data) {
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

void CC1101::writeRegBurst(byte addr, byte *data, size_t size) {
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

  Serial.printf("DEBUG: Command %02x\r\n", addr);

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

byte CC1101::getChipPartNumber() {
  return readReg(CC1101_REG_PARTNUM);
}

byte CC1101::getChipVersion() {
  return readReg(CC1101_REG_VERSION);
}

