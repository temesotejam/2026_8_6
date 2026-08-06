#pragma once
#include <Arduino.h>
enum class InaProfile:uint8_t { Current=0, Balanced=1, Fast=2 };
struct InaSample{uint64_t us=0;int16_t rawShunt=0,rawCurrent=0;uint16_t rawBus=0,rawPower=0,mask=0;float shuntV=NAN,busV=NAN,currentShuntA=NAN,currentRegA=NAN,powerRegW=NAN,powerCalcW=NAN;uint32_t i2cUs=0;bool fresh=false,valid=false;};
struct InaBenchmarkStats { uint32_t attempts=0,conversionReady=0,fresh=0,duplicates=0,transactions=0,bytes=0,maxReadUs=0; uint64_t lastFreshUs=0; };
constexpr uint16_t inaConfigFor(InaProfile p){return p==InaProfile::Current?0x08DFu:p==InaProfile::Balanced?0x04DFu:0x02DFu;}
constexpr uint16_t inaConfigMask(){return 0xBFFFu;} // Bit 14 may read back high on INA226.
constexpr float inaInternalHz(InaProfile p){return 1000000.0f/((p==InaProfile::Current?128.0f:p==InaProfile::Balanced?16.0f:4.0f)*2.0f*588.0f);}
class Ina226 {public: bool begin();bool configure(InaProfile);bool read(InaSample&);bool present();void resetBenchmark();uint16_t manufacturer=0,die=0,configuration=0,calibration=0,maskEnable=0;uint32_t errors=0,reinit=0;InaProfile profile=InaProfile::Current;InaBenchmarkStats benchmark{};private:bool read16(uint8_t,uint16_t&);bool write16(uint8_t,uint16_t);};
