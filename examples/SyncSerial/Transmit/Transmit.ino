// Synchronous serial mode transmit example.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN 10
#define GDO0_PIN 15  // data
#define GDO2_PIN 16  // serial clock

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN, /* gd2 */ GDO2_PIN);

// Every packet is a marker followed by a counter, both sent MSB first. No
// preamble or sync word is added by the chip; the receive example finds the
// marker at the bit level. MARKER must match there.
const uint32_t MARKER = 0xdeadbeef;

uint32_t counter = 0;

static inline void clockOutBit(uint8_t bit) {
  while (digitalRead(GDO2_PIN) == HIGH) { yield(); }
  digitalWrite(GDO0_PIN, bit ? HIGH : LOW);
  while (digitalRead(GDO2_PIN) == LOW) { yield(); }
}

static void clockOutWord(uint32_t word) {
  for (int b = 31; b >= 0; b--) {
    clockOutBit((word >> b) & 0x01);
  }
}

static void printHex(uint32_t word) {
  for (int shift = 24; shift >= 0; shift -= 8) {
    uint8_t b = word >> shift;
    if (b < 0x10) {
      Serial.print('0');
    }
    Serial.print(b, HEX);
    Serial.print(' ');
  }
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

  radio.setModulation(MOD_2FSK);
  radio.setFrequency(433.8);
  radio.setFrequencyDeviation(20);
  radio.setDataRate(10);
  radio.setOutputPower(10);

  // Send raw data only: disable all of the chip's packet handling.
  radio.setPacketLengthMode(PKT_LEN_MODE_INFINITE);
  radio.setSyncMode(SYNC_MODE_NO_PREAMBLE);
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setManchester(false);
  radio.setFEC(false);

  radio.setPacketFormat(PKT_FORMAT_SYNC_SERIAL);
}

void loop() {
  Serial.print(F("Transmitting "));
  printHex(MARKER);
  Serial.print(F("| "));
  printHex(counter);
  Serial.println();

  if (radio.serialTransmit() != STATUS_OK) {
    Serial.println(F("serialTransmit() failed"));
    delay(1000);
    return;
  }

  clockOutWord(MARKER);
  clockOutWord(counter);

  // Clock out exactly 12 dummy bits before leaving TX (CC1101 errata SWRZ020E,
  // "Extra Byte Transmitted in TX").
  for (int i = 0; i < 12; i++) {
    clockOutBit(0);
  }

  radio.idle();
  counter++;
  delay(1000);
}
