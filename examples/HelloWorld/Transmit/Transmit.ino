#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

Radio radio(/* cs pin */ 10);

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println(F("Starting ..."));
  delay(1000);

  if (radio.begin() == STATUS_CHIP_NOT_FOUND) {
    Serial.println(F("Chip not found!"));
    while (true) { delay(1000); }
  }

  radio.setModulation(MOD_ASK_OOK);
  radio.setFrequency(433.8);
  radio.setDataRate(10);
  radio.setOutputPower(10);

  radio.setPacketLengthMode(PKT_LEN_MODE_VARIABLE);
  radio.setAddressFilteringMode(ADDR_FILTER_MODE_NONE);
  radio.setPreambleLength(64);
  radio.setSyncWord(0x1234);
  radio.setSyncMode(SYNC_MODE_16_16);
  radio.setCrc(true);
  radio.setDataWhitening(true);
  radio.setManchester(false);
  radio.setFEC(false);
}

int counter = 0;

void loop() {
  String data = "Hello #" + String(counter++);

  Serial.print(F("Transmitting: "));
  Serial.print(data);
  Serial.print(F(" "));
  Status status = radio.transmit((uint8_t *)data.c_str(), data.length());

  if (status == STATUS_OK) {
    Serial.println(F("[OK]"));
  } else {
    Serial.print("[ERROR ");
    Serial.print(status);
    Serial.println("]");
  }

  delay(1000);
}
