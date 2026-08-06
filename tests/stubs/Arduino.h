#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

#define SERIAL_8N1 0
#define portMUX_INITIALIZER_UNLOCKED 0

constexpr int D0=0,D1=1,D2=2,D3=3,D4=4,D5=5,D6=6,D7=7,D8=8,D9=9,D10=10;

using TaskHandle_t=void*;
using portMUX_TYPE=int;
using UBaseType_t=unsigned int;

class String {
 public:
  String()=default;
  String(const char* value):value_(value?value:""){}
  size_t length() const{return value_.length();}
  const char* c_str() const{return value_.c_str();}
  bool operator==(const char* other) const{return value_==(other?other:"");}
 private:
  std::string value_{};
};

class HardwareSerial {
 public:
  explicit HardwareSerial(int=0) {}
  void begin(uint32_t,int,int,int) {}
  void setRxBufferSize(size_t) {}
  void setTimeout(uint32_t) {}
  int available() const{return 0;}
  int read(){return -1;}
  size_t read(uint8_t*,size_t){return 0;}
  size_t write(const uint8_t*,size_t count){return count;}
};

class SerialConsole {
 public:
  void begin(uint32_t) {}
  int printf(const char*, ...) __attribute__((format(printf, 2, 3)));
};

extern SerialConsole Serial;

template <typename T>
constexpr T min(T left,T right){return left<right?left:right;}

template <typename T>
constexpr T constrain(T value,T low,T high){return value<low?low:(value>high?high:value);}

inline uint32_t millis(){return 0;}
inline void delay(uint32_t){}
inline void portENTER_CRITICAL(portMUX_TYPE*){}
inline void portEXIT_CRITICAL(portMUX_TYPE*){}
inline uint32_t pdMS_TO_TICKS(uint32_t value){return value;}
inline void vTaskDelay(uint32_t){}
inline int xTaskCreatePinnedToCore(void(*)(void*),const char*,uint32_t,void*,int,TaskHandle_t*,int){return 1;}
