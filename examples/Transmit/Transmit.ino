#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

uint8_t data[] = { 0xde, 0xad, 0xbe, 0xef };

Radio radio(/* cs pin */ 10);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ...");
  delay(3000);

  if (radio.begin() == STATUS_CHIP_NOT_FOUND) {
    Serial.println("Chip not found!");
    for (;;);
  }

  radio.setModulation(MOD_ASK_OOK);
  radio.setFrequency(433.8);
  radio.setDataRate(10);
  radio.setOutputPower(10);

  radio.setPacketLengthMode(PKT_LEN_MODE_FIXED, sizeof(data));
  radio.setAddressFilteringMode(ADDR_FILTER_MODE_NONE);
  radio.setPreambleLength(16);
  radio.setSyncWord(0x0001);
  radio.setSyncMode(SYNC_MODE_16_16);
  radio.setCrc(false);
}

void loop() {
  Serial.println("Transmitting ...");
  radio.transmit(data, sizeof(data));
  delay(1000);
}

