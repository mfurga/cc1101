#include <Arduino.h>
#include "cc1101.h"

using namespace CC1101;

Radio radio(10);
uint8_t data[] = { 0xde, 0xad, 0xbe, 0xef };

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ...");
  delay(3000);

  if (radio.begin() == STATUS_ERROR_CHIP_NOT_FOUND) {
    Serial.println("Chip not found!");
    for (;;);
  }

  radio.setModulation(MOD_ASK_OOK);
  radio.setFrequency(433.804 + 0.05);
  radio.setDataRate(2.694);
  radio.setOutputPower(10);

  radio.setPacketLengthMode(PKT_LEN_MODE_FIXED, sizeof(data));
  radio.setAddressFilteringMode(ADDR_FILTER_MODE_NONE);
  radio.setPreambleLength(16);
  radio.setSyncWord(0xaa00);
  radio.setSyncMode(SYNC_MODE_16_16);
  radio.setCrc(false);
}

void loop() {
  Serial.println("Transmitting ...");
  radio.transmit(data, sizeof(data));
  delay(1000);
}

