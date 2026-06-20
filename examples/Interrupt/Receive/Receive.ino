// Non-blocking (interrupt-driven) receive example, standard FIFO packet mode.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN    10
#define GDO0_PIN  2  // must be an interrupt-capable pin

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN);

volatile bool packetReceived = false;

#if defined(ESP8266) || defined(ESP32)
IRAM_ATTR
#endif
void onReceive() {  // keep ISRs tiny: just set a flag
  packetReceived = true;
}

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

  radio.setReceiveAction(onReceive);
  radio.startReceive();

  Serial.println(F("Listening ..."));
}

void loop() {
  if (!packetReceived) {
    // ... loop() is free to do other work here
    static unsigned long lastBeat = 0;
    if (millis() - lastBeat >= 200) {
      lastBeat = millis();
      Serial.println(F("Doing other work while waiting ..."));
    }
    return;
  }
  packetReceived = false;

  char buff[32];
  size_t read;
  Status status = radio.readData((uint8_t *)buff, sizeof(buff) - 1, &read);

  if (status == STATUS_OK) {
    buff[read] = '\0';
    Serial.print(F("Data: "));
    Serial.println(buff);
    Serial.print(F("RSSI: "));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
    Serial.print(F("LQI: "));
    Serial.println(radio.getLQI());
  } else if (status == STATUS_CRC_MISMATCH) {
    Serial.println(F("CRC mismatch!"));
  } else {
    Serial.print(F("Error: "));
    Serial.println(status);
  }

  radio.startReceive();
}
