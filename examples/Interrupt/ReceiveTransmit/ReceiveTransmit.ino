// Non-blocking (interrupt-driven) transmit example, standard FIFO packet mode.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN     10
#define GDO0_PIN    2 // must be an interrupt-capable pin
#define GDO2_PIN    3 // must be an interrupt-capable pin
#define RANDOM_PIN A0 // must be an unconnected analog pin

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN, /* gd2 */ GDO2_PIN);

volatile bool packetSent = false;
volatile bool packetReceived = false;

#if defined(ESP8266) || defined(ESP32)
IRAM_ATTR
#endif
void onTransmit() {  // keep ISRs tiny: just set a flag
  packetSent = true;
}

#if defined(ESP8266) || defined(ESP32)
IRAM_ATTR
#endif
void onReceive() {  // keep ISRs tiny: just set a flag
  packetReceived = true;
}

int counter = 0;

void setup() {
  randomSeed(analogRead(RANDOM_PIN));

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

  Status status;

  status = radio.setTransmitAction(onTransmit, GDO0);
  if (status != STATUS_OK) {
    Serial.print(F("setTransmitAction error: "));
    Serial.println(status);
    while (true) { delay(1000); }
  }

  status = radio.setReceiveAction(onReceive, GDO2);
  if (status != STATUS_OK) {
    Serial.print(F("setReceiveAction error: "));
    Serial.println(status);
    while (true) { delay(1000); }
  }
  radio.startReceive();

  Serial.println(F("Listening ..."));
}

void loop() {
  static unsigned long nextPacketTime = 0;

  if (millis() > nextPacketTime) {
    nextPacketTime = millis() + random(1500, 2000);

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
  }

  if (packetSent) {
    packetSent = false;
    Status status = radio.finishTransmit();

    if (status == STATUS_OK) {
      Serial.println(F("[OK]"));
    } else {
      Serial.print("[ERROR ");
      Serial.print(status);
      Serial.println("]");
    }

    radio.startReceive();
    Serial.println(F("Listening ..."));
  }

  if (packetReceived) {
    packetReceived = false;

    char buff[32];
    size_t read;
    Status status = radio.readData((uint8_t *)buff, sizeof(buff) - 1, &read);

    if (status == STATUS_OK) {
      buff[read] = '\0';
      Serial.print(F("Received data: "));
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
    Serial.println(F("Listening ..."));
  }
}