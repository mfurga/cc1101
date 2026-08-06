// Asynchronous serial mode receive example.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN   10
#define GDO0_PIN 2  // demodulated data output from the chip

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN);

// Data rate in kBaud. One bit therefore lasts 1000 / DATA_RATE microseconds.
const double DATA_RATE = 10.0;
const unsigned int BIT_US = (unsigned int)(1000.0 / DATA_RATE);

// Marker that every packet starts with, matched at the bit level because the
// chip adds no preamble or sync word. Must match MARKER in the transmit
// example, which sends a counter right after it.
const uint32_t MARKER = 0xdeadbeef;
uint32_t nextSampleUs = 0;

static inline uint8_t sampleBit() {
  while ((int32_t)(micros() - nextSampleUs) < 0) { yield(); }

  uint8_t b = digitalRead(GDO0_PIN) & 0x01;
  nextSampleUs += BIT_US;

  if ((int32_t)(micros() - nextSampleUs) > (int32_t)BIT_US) {
    nextSampleUs = micros() + BIT_US;
  }

  return b;
}

static uint32_t sampleWord() {
  uint32_t word = 0;
  for (int b = 0; b < 32; b++) {
    word = (word << 1) | sampleBit();
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

  radio.setModulation(MOD_ASK_OOK);
  radio.setFrequency(433.8);
  radio.setRxBandwidth(64.0);
  radio.setDataRate(DATA_RATE);
  radio.setOutputPower(10);

  // Receive raw data only: disable all of the chip's packet handling. This
  // matches the configuration used by the Transmit example.
  radio.setPacketLengthMode(PKT_LEN_MODE_INFINITE);
  radio.setSyncMode(SYNC_MODE_NO_PREAMBLE);
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setManchester(false);
  radio.setFEC(false);

  radio.setPacketFormat(PKT_FORMAT_ASYNC_SERIAL);

  if (radio.serialReceive() != STATUS_OK) {
    Serial.println(F("serialReceive() failed"));
    while (true) { delay(1000); }
  }

  nextSampleUs = micros() + BIT_US;

  Serial.println(F("Receiving ..."));
}

void loop() {
  uint32_t window = 0;
  while (window != MARKER) {
    window = (window << 1) | sampleBit();
  }

  uint32_t payload = sampleWord();

  Serial.print(F("Received "));
  printHex(MARKER);
  Serial.print(F("| "));
  printHex(payload);
  Serial.println();
}
