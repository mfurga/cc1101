#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

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

  radio.setPacketLengthMode(PKT_LEN_MODE_FIXED, 10);
  radio.setAddressFilteringMode(ADDR_FILTER_MODE_NONE);
  radio.setPreambleLength(64);
  radio.setSyncWord(0x1234);
  radio.setSyncMode(SYNC_MODE_16_16);
  radio.setCrc(true);
}

int counter = 0;

void loop() {
  String id = "000" + String(counter++);
  id = id.substring(id.length() - 3);
  String data = "Hello #" + id;

  Serial.print("Transmitting: " + data + " ");
  Status status = radio.transmit((uint8_t *)data.c_str(), 10);

  if (status == STATUS_OK) {
    Serial.println("[OK]");
  } else {
    Serial.println("[ERRPR]");
  }

  delay(1000);
}
