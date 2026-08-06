#include "pca9685.h"
#include <Wire.h>
#include "app_config.h"
using namespace cfg;
namespace { constexpr uint8_t MODE1=0x00,MODE2=0x01,PRE_SCALE=0xFE,LED0_ON_L=0x06,ALL_LED_ON_L=0xFA; }
bool Pca9685::ack(){Wire.beginTransmission(kPcaAddress);return Wire.endTransmission()==0;}
bool Pca9685::write8(uint8_t r,uint8_t v){Wire.beginTransmission(kPcaAddress);Wire.write(r);Wire.write(v);if(Wire.endTransmission()!=0){++errors;return false;}return true;}
bool Pca9685::writeN(uint8_t r,const uint8_t*d,size_t n){Wire.beginTransmission(kPcaAddress);Wire.write(r);Wire.write(d,n);if(Wire.endTransmission()!=0){++errors;return false;}return true;}
bool Pca9685::read8(uint8_t r,uint8_t&v){Wire.beginTransmission(kPcaAddress);Wire.write(r);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)kPcaAddress,1)!=1){++errors;return false;}v=Wire.read();return true;}
bool Pca9685::allOff(){uint8_t d[4]={0,0,0,0x10};for(uint8_t ch=0;ch<16;++ch)if(!writeN(LED0_ON_L+4*ch,d,4))return false;for(uint8_t ch=0;ch<16;++ch){PcaChannel r;if(!readChannel(ch,r)||!r.fullOff){++readbackErrors;return false;}}return true;}
bool Pca9685::setFullOff(uint8_t ch){if(ch>15)return false;uint8_t d[4]={0,0,0,0x10};if(!writeN(LED0_ON_L+4*ch,d,4))return false;PcaChannel r;if(!readChannel(ch,r)||!r.fullOff){++readbackErrors;return false;}return true;}
bool Pca9685::begin(){detected=false;if(!ack())return false; if(!write8(MODE1,0x31))return false; uint32_t pre=(kOscillatorHz+(uint32_t)(2048*kServoPwmHz))/(uint32_t)(4096*kServoPwmHz)-1; if(pre<3)pre=3;if(pre>255)pre=255;prescale=(uint8_t)pre;if(!write8(PRE_SCALE,prescale)||!write8(MODE2,0x04)||!write8(MODE1,0x21))return false;delay(5);if(!write8(MODE1,0xA1)||!allOff())return false;if(!read8(MODE1,mode1)||!read8(MODE2,mode2)||!read8(PRE_SCALE,prescale)){++readbackErrors;return false;}actualHz=(float)kOscillatorHz/(4096.0f*(prescale+1));detected=true;return true;}
bool Pca9685::setPulse(uint8_t ch,uint16_t us,PcaChannel*a){if(ch>15)return false;uint32_t count=((uint32_t)us*4096UL*(uint32_t)(kServoPwmHz+0.5f)+500000UL)/1000000UL;if(count>4095)count=4095;uint8_t d[4]={0,0,(uint8_t)(count&0xff),(uint8_t)((count>>8)&0x0f)};if(!writeN(LED0_ON_L+4*ch,d,4))return false;PcaChannel r;if(!readChannel(ch,r)){++readbackErrors;return false;}if(r.on!=0||r.off!=count||r.fullOff){++readbackErrors;return false;}if(a)*a=r;return true;}
bool Pca9685::readChannel(uint8_t ch,PcaChannel&o){if(ch>15)return false;uint8_t r=LED0_ON_L+4*ch;Wire.beginTransmission(kPcaAddress);Wire.write(r);if(Wire.endTransmission(false)!=0||Wire.requestFrom((int)kPcaAddress,4)!=4){++errors;return false;}uint8_t a=Wire.read(),b=Wire.read(),c=Wire.read(),d=Wire.read();o.on=((uint16_t)(b&0x0f)<<8)|a;o.off=((uint16_t)(d&0x0f)<<8)|c;o.fullOff=(d&0x10)!=0;return true;}
