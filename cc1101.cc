#include "cc1101.h"

Status CC1101::begin() {
  pinMode(cs, OUTPUT);
  chipDeselect();

  SPI.begin();

  hardReset();
  delay(2);

  byte partnum = getChipPartNumber();
  byte version = getChipVersion();

  if (partnum != CC1101_PARTNUM || version != CC1101_VERSION) {
    return STATUS_ERROR_CHIP_NOT_FOUND;
  }

  return STATUS_OK;
}

byte CC1101::readReg(byte addr) {
  byte header = CC1101_READ | (addr & 0b111111);

  if (addr >= 0x30) {
    /* Status registers - access with the burst bit on. */
    header |= CC1101_BURST;
  }

  SPI.beginTransaction(spiSettings);

  chipSelect();
  waitReady();

  SPI.transfer(header);
  byte result = SPI.transfer(0x00);

  chipDeselect();

  SPI.endTransaction();
  return result;
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

  SPI.transfer(CC1101_SRES);

  waitReady();
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

