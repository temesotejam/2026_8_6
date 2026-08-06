#pragma once

#include <stdint.h>

class SPIClass {
 public:
  void begin(int, int, int, int) {}
};

extern SPIClass SPI;
