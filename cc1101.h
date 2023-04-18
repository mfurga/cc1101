#pragma once

#include <Arduino.h>

#define PIN_UNUSED 0xff

class CC1101 {
 public:
  CC1101(byte ss, byte gd0 = PIN_UNUSED, byte gd2 = PIN_UNUSED)
    : ss(ss), gd0(gd0), gd2(gd2) {}

 private:
  byte ss, gd0, gd2;
};


