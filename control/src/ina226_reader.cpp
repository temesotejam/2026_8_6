#include "ina226_reader.h"
#include <Wire.h>
#include <esp_timer.h>
#include "app_config.h"
using namespace app_config;
bool Ina226::read16(uint8_t r,uint16_t&v){benchmark.transactions+=2;benchmark.bytes+=5;Wire.beginTransmission(kInaAddress);Wire.write(r);if(Wire.endTransmission(false)!=0)return false;if(Wire.requestFrom((int)kInaAddress,2)!=2)return false;v=((uint16_t)Wire.read()<<8)|Wire.read();return true;}
bool Ina226::write16(uint8_t r,uint16_t v){benchmark.transactions++;benchmark.bytes+=3;Wire.beginTransmission(kInaAddress);Wire.write(r);Wire.write(v>>8);Wire.write(v);return Wire.endTransmission()==0;}
bool Ina226::present(){uint16_t x=0;return read16(0xFE,x)&&x==0x5449;}
bool Ina226::configure(InaProfile p){profile=p;const uint16_t requested=inaConfigFor(p);if(!write16(0x00,requested)||!write16(0x05,kCalibration)||!write16(0x06,0x0400))return false;uint16_t readback=0;if(!read16(0x00,readback)||!read16(0x05,calibration))return false;configuration=readback;return (configuration&inaConfigMask())==(requested&inaConfigMask())&&calibration==kCalibration;}
bool Ina226::begin(){++reinit; uint16_t x=0;if(!read16(0xFE,manufacturer)||manufacturer!=0x5449||!read16(0xFF,die)||die!=0x2260)return false;return configure(profile);}
void Ina226::resetBenchmark(){benchmark={};}
bool Ina226::read(InaSample&s){uint64_t t=esp_timer_get_time();uint16_t a,b,c,d,e;benchmark.attempts++;if(!(read16(0x06,e)&&read16(0x01,a)&&read16(0x02,b)&&read16(0x03,c)&&read16(0x04,d))){++errors;return false;}s.us=t;s.rawShunt=(int16_t)a;s.rawBus=b;s.rawPower=c;s.rawCurrent=(int16_t)d;s.mask=e;s.fresh=(e&0x0008u)!=0;if(s.fresh){benchmark.conversionReady++;benchmark.fresh++;benchmark.lastFreshUs=t;}else benchmark.duplicates++;s.shuntV=s.rawShunt*2.5e-6f;s.busV=s.rawBus*1.25e-3f;s.currentShuntA=s.shuntV/kShuntOhm;s.currentRegA=s.rawCurrent*kCurrentLsbA;s.powerRegW=s.rawPower*kPowerLsbW;s.powerCalcW=s.busV*s.currentShuntA;s.i2cUs=(uint32_t)(esp_timer_get_time()-t);if(s.i2cUs>benchmark.maxReadUs)benchmark.maxReadUs=s.i2cUs;s.valid=true;return true;}
