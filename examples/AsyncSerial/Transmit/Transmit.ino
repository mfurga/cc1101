// Asynchronous serial mode transmit example.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN   10
#define GDO0_PIN 15  // data input to the chip (driven by the MCU in TX)

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN);

// Data rate in kBaud. One bit therefore lasts 1000 / DATA_RATE microseconds.
const double DATA_RATE = 10.0;
const unsigned int BIT_US = (unsigned int)(1000.0 / DATA_RATE);

// Every packet is a marker followed by a counter, both sent MSB first. No
// preamble or sync word is added by the chip; the receive example finds the
// marker at the bit level. MARKER must match there.
const uint32_t MARKER = 0xdeadbeef;

uint32_t counter = 0;

static inline void sendBit(uint8_t bit) {
  digitalWrite(GDO0_PIN, bit ? HIGH : LOW);
  delayMicroseconds(BIT_US);
}

static void sendWord(uint32_t word) {
  for (int b = 31; b >= 0; b--) {
    sendBit((word >> b) & 0x01);
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

  radio.setModulation(MOD_ASK_OOK);
  radio.setFrequency(433.8);
  radio.setDataRate(DATA_RATE);
  radio.setOutputPower(10);

  radio.setPacketLengthMode(PKT_LEN_MODE_INFINITE);
  radio.setSyncMode(SYNC_MODE_NO_PREAMBLE);
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setManchester(false);
  radio.setFEC(false);

  radio.setPacketFormat(PKT_FORMAT_ASYNC_SERIAL);
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

  sendWord(MARKER);
  sendWord(counter);

  digitalWrite(GDO0_PIN, LOW);  // leave the line idle
  radio.idle();
  counter++;

  delay(1000);
}
