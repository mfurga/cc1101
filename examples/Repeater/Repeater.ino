// Universal OOK signal repeater using asynchronous serial mode.
//
// This listens for a 433 MHz OOK burst, records it, and re-transmits an identical
// copy. It works WITHOUT knowing the data rate, bit encoding or framing, because
// it never decodes anything. We simply time the high/low pulse widths on capture
// and reproduce those exact timings on replay.

#include <Arduino.h>
#include <cc1101.h>

using namespace CC1101;

#define CS_PIN   10
#define GDO0_PIN 15  // data

Radio radio(/* cs */ CS_PIN, /* gd0 */ GDO0_PIN);

const double   FREQUENCY    = 433.8;  // MHz (must match the signal you repeat)
const double   RX_BANDWIDTH = 64.0;   // kHz (wide enough for the signal you repeat)
// A high data rate makes the TX modulator oversample finely so arbitrary
// captured timings are reproduced well.
const double   DATA_RATE    = 250.0;  // kBaud
const int8_t   OUTPUT_POWER = 10;     // dBm
const size_t   MAX_PULSES   = 512;    // capture buffer size (transitions)
const size_t   MIN_PULSES   = 16;     // ignore bursts shorter than this (noise)
const uint32_t PULSE_MIN_US = 450;    // a real burst must start with a HIGH pulse this long
const uint32_t GAP_US       = 6000;   // idle this long; end of burst

uint16_t pulses[MAX_PULSES];
size_t   pulseCount = 0;
uint8_t  firstLevel = HIGH;

static inline uint16_t clampDur(uint32_t us) {
  return (us > 0xffff) ? 0xffff : (uint16_t)us;
}

// delayMicroseconds() is only reliable up to ~16 ms on some cores; chunk longer
// waits so replay timing stays accurate on every platform.
static inline void delayUs(uint32_t us) {
  while (us > 16000) {
    delayMicroseconds(16000);
    us -= 16000;
  }
  delayMicroseconds(us);
}

bool captureBurst() {
  pulseCount = 0;

  // Wait for the start of a real burst: a HIGH pulse at least PULSE_MIN_US long.
  unsigned long edge;
  for (;;) {
    while (digitalRead(GDO0_PIN) == HIGH) { yield(); }
    while (digitalRead(GDO0_PIN) == LOW) { yield(); }
    edge = micros();
    while (digitalRead(GDO0_PIN) == HIGH) { yield(); }

    uint32_t high = micros() - edge;
    if (high >= PULSE_MIN_US) {
      firstLevel = HIGH;
      pulses[0] = clampDur(high);
      pulseCount = 1;
      edge = micros();
      break;
    }
  }

  // Record alternating segments until the line stays idle for GAP_US or the buffer fills.
  uint8_t level = LOW;
  while (pulseCount < MAX_PULSES) {
    while (digitalRead(GDO0_PIN) == level) {
      if (micros() - edge > GAP_US) {
        return pulseCount >= MIN_PULSES;
      }
    }
    unsigned long now = micros();
    pulses[pulseCount++] = clampDur(now - edge);
    edge = now;
    level = !level;
  }

  return true;
}

void replayBurst() {
  if (radio.serialTransmit() != STATUS_OK) {
    Serial.println(F("serialTransmit() failed"));
    return;
  }

  uint8_t level = firstLevel;
  for (size_t i = 0; i < pulseCount; i++) {
    digitalWrite(GDO0_PIN, level);
    delayUs(pulses[i]);
    level = !level;
  }

  // Leave the carrier off and hold it there for 12 bit periods before leaving TX
  // (CC1101 errata SWRZ020E, "Extra Byte Transmitted in TX").
  digitalWrite(GDO0_PIN, LOW);
  delayUs((uint32_t)(12 * 1000.0 / DATA_RATE + 0.5));

  radio.idle();
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
  radio.setFrequency(FREQUENCY);
  radio.setRxBandwidth(RX_BANDWIDTH);
  radio.setDataRate(DATA_RATE);
  radio.setOutputPower(OUTPUT_POWER);

  // Receive/transmit raw serial data: disable packet engine features.
  radio.setPacketLengthMode(PKT_LEN_MODE_INFINITE);
  radio.setSyncMode(SYNC_MODE_NO_PREAMBLE);
  radio.setCrc(false);
  radio.setDataWhitening(false);
  radio.setManchester(false);
  radio.setFEC(false);

  radio.setPacketFormat(PKT_FORMAT_ASYNC_SERIAL);

  Serial.println(F("Repeater ready."));
}

void loop() {
  if (radio.serialReceive() != STATUS_OK) {
    Serial.println(F("serialReceive() failed"));
    delay(1000);
    return;
  }
  Serial.println(F("Listening for a burst ..."));

  if (!captureBurst()) {
    radio.idle();
    return;
  }
  radio.idle();

  Serial.print(F("Captured "));
  Serial.print(pulseCount);
  Serial.println(F(" pulses, ready to repeat"));

  delay(3000);

  replayBurst();

  Serial.println(F("Burst replayed"));

  delay(500);
}
