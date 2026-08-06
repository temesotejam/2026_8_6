#pragma once

#include <Arduino.h>

class Preferences {
 public:
  bool begin(const char*,bool){return true;}
  uint32_t getUInt(const char*,uint32_t fallback=0){return fallback;}
  size_t putUInt(const char*,uint32_t){return sizeof(uint32_t);}
  void end(){}
};
