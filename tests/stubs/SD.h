#pragma once

#include <stdint.h>

#include "SPI.h"

class SDClass {
 public:
  bool begin(int, SPIClass&, uint32_t) { return true; }
};

extern SDClass SD;
