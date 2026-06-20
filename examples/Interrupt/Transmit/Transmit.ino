// Non-blocking (interrupt-driven) transmit example, standard FIFO packet mode.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN    10
#define GDO0_PIN  15  // must be an interrupt-capable pin

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN);

volatile bool packetSent = false;

#if defined(ESP8266) || defined(ESP32)
IRAM_ATTR
#endif
void onTransmit() {  // keep ISRs tiny: just set a flag
  packetSent = true;
}

int counter = 0;

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

  Status status = radio.setTransmitAction(onTransmit);
  if (status != STATUS_OK) {
    Serial.print(F("setTransmitAction error: "));
    Serial.println(status);
    while (true) { delay(1000); }
  }
}

void loop() {
  String data = "Hello #" + String(counter++);

  Serial.print(F("Transmitting: "));
  Serial.println(data);

  Status status = radio.startTransmit((uint8_t *)data.c_str(), data.length());
  if (status != STATUS_OK) {
    Serial.print(F("startTransmit error: "));
    Serial.println(status);
    delay(1000);
    return;
  }

  // ... loop() is free to do other work while the packet is sent
  unsigned long lastBeat = 0;
  while (!packetSent) {
    if (millis() - lastBeat >= 50) {
      lastBeat = millis();
      Serial.println(F("Doing other work while transmitting ..."));
    }
    yield();
  }

  packetSent = false;
  status = radio.finishTransmit();

  if (status == STATUS_OK) {
    Serial.println(F("[OK]"));
  } else {
    Serial.print("[ERROR ");
    Serial.print(status);
    Serial.println("]");
  }

  delay(1000);
}
