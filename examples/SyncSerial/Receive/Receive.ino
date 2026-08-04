// Synchronous serial mode receive example.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN 10
#define GDO0_PIN 15  // data
#define GDO2_PIN 16  // serial clock

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN, /* gd2 */ GDO2_PIN);

// Marker that every packet starts with, matched at the bit level because the
// chip adds no preamble or sync word. Must match MARKER in the transmit
// example, which sends a counter right after it.
const uint32_t MARKER = 0xdeadbeef;

static inline uint8_t clockInBit() {
  while (digitalRead(GDO2_PIN) == HIGH) { yield(); }  // wait for the low phase of the clock
  while (digitalRead(GDO2_PIN) == LOW) { yield(); }   // rising edge: the data is valid
  return digitalRead(GDO0_PIN) & 1;
}

static uint32_t clockInWord() {
  uint32_t word = 0;
  for (int b = 0; b < 32; b++) {
    word = (word << 1) | clockInBit();
  }
  return word;
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

  // Receive raw data only: disable all of the chip's packet handling. Infinite
  // packet length mode keeps the chip in RX with the serial clock free-running;
  // fixed/variable mode would terminate RX after a byte count and stop the clock,
  // hanging the sampling loop.
  radio.setPacketLengthMode(PKT_LEN_MODE_INFINITE);
  radio.setSyncMode(SYNC_MODE_NO_PREAMBLE);
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setManchester(false);
  radio.setFEC(false);

  radio.setPacketFormat(PKT_FORMAT_SYNC_SERIAL);

  if (radio.serialReceive() != STATUS_OK) {
    Serial.println(F("serialReceive() failed"));
    while (true) { delay(1000); }
  }

  Serial.println(F("Receiving ..."));
}

void loop() {
  // Slide a 32 bit window over the stream until the marker lines up.
  uint32_t window = 0;
  while (window != MARKER) {
    window = (window << 1) | clockInBit();
  }

  uint32_t payload = clockInWord();

  Serial.print(F("Received "));
  printHex(MARKER);
  Serial.print(F("| "));
  printHex(payload);
  Serial.println();
}
