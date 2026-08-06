#pragma once

#include <Arduino.h>

constexpr int HTTP_GET=0;
constexpr int HTTP_POST=1;
class WebServer {
 public:
  explicit WebServer(uint16_t){}
  String arg(const char*) const{return {};}
  void send(int,const char*,const char*){}
  template <typename Handler> void on(const char*,int,Handler){}
  template <typename Handler> void onNotFound(Handler){}
  void begin(){}
  void handleClient(){}
};
