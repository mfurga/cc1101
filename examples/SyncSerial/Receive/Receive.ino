// Synchronous serial mode receive example.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN 10
#define GDO0_PIN 15  // data
#define GDO2_PIN 16  // serial clock

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN, /* gd2 */ GDO2_PIN);

// Marker M that every packet starts with. It is matched at the bit level, so it
// can be up to 8 bytes. Set MARKER_LEN to 0 to disable matching (print all data).
const uint8_t MARKER[] = {0xaa, 0xaa, 0x00, 0x00};
const size_t MARKER_LEN = sizeof(MARKER);  // marker bytes to match; 0 = disabled
const size_t PAYLOAD_LEN = 4; // bytes to capture and print after the marker

static_assert(MARKER_LEN <= 8, "MARKER_LEN must be <= 8 (matched in a uint64_t)");

uint64_t markerBits = 0;
uint64_t markerMask = 0;

static inline uint8_t clockInBit() {
  while (digitalRead(GDO2_PIN) == HIGH) { yield(); }  // wait for the low phase of the clock
  while (digitalRead(GDO2_PIN) == LOW) { yield(); }   // rising edge: the data is valid
  return digitalRead(GDO0_PIN) & 1;
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

  // Pack the marker into a right-aligned bit pattern and build its mask.
  for (size_t i = 0; i < MARKER_LEN; i++) {
    markerBits = (markerBits << 8) | MARKER[i];
  }
  markerMask = (MARKER_LEN == 0) ? 0
             : (MARKER_LEN >= 8) ? ~0ULL
                                 : ((1ULL << (MARKER_LEN * 8)) - 1);

  radio.serialReceive();
  Serial.println(F("Receiving ..."));
}

void loop() {
  static uint64_t window = 0;

  if (MARKER_LEN > 0) {
    window = (window << 1) | clockInBit();
    if ((window & markerMask) != markerBits) {
      return;
    }
  }

  uint8_t payload[PAYLOAD_LEN];
  for (size_t i = 0; i < PAYLOAD_LEN; i++) {
    uint8_t b = 0;
    for (int k = 0; k < 8; k++) {
      b = (b << 1) | clockInBit();
    }
    payload[i] = b;
  }

  for (size_t i = 0; i < PAYLOAD_LEN; i++) {
    if (payload[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(payload[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  window = 0;
}
