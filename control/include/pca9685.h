#pragma once
#include <Arduino.h>
struct PcaChannel { uint16_t on=0, off=0; bool fullOff=true; };
class Pca9685 {
 public:
  bool begin(); bool allOff(); bool setFullOff(uint8_t channel); bool setPulse(uint8_t channel,uint16_t pulseUs,PcaChannel* actual=nullptr); bool readChannel(uint8_t channel,PcaChannel& out);
  bool detected=false; uint8_t mode1=0,mode2=0,prescale=0; uint32_t errors=0,readbackErrors=0; float actualHz=0;
 private:
  bool write8(uint8_t reg,uint8_t value); bool writeN(uint8_t reg,const uint8_t* data,size_t count); bool read8(uint8_t reg,uint8_t& value); bool ack();
};
