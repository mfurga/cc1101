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

void loop() {
  char buff[32] = {0};

  Serial.println("Receiving ...");
  Status status = radio.receive((uint8_t *)buff, sizeof(buff));

  if (status == STATUS_OK) {
    Serial.print("Data: ");
    Serial.println(buff);

    Serial.print("RSSI: ");
    Serial.print(radio.getRSSI());
    Serial.println(" dBm");

    Serial.print("LQI: ");
    Serial.println(radio.getLQI());
  } else if (status == STATUS_CRC_MISMATCH) {
    Serial.println("CRC mismatch!");
  } else {
    Serial.println("Error!");
  }

  memset(buff, 0, sizeof(buff));
  Serial.println();
}
