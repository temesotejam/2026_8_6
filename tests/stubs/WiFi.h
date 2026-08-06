#pragma once

#include <Arduino.h>

constexpr int WIFI_AP=0;
class IPAddressStub { public: String toString() const{return String("192.168.4.1");} };
class WiFiStub {
 public:
  void mode(int){}
  void setSleep(bool){}
  bool softAP(const char*,const char*){return true;}
  IPAddressStub softAPIP() const{return {};}
};
extern WiFiStub WiFi;
