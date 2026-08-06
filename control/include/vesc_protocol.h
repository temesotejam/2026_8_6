#pragma once
#include <Arduino.h>
#include "app_config.h"
namespace vesc {
constexpr uint8_t kCommFwVersion = 0;
constexpr uint8_t kCommGetValues = 4;
constexpr uint8_t kCommSetDuty = 5;
enum class ParseError : uint8_t { None, Length, Crc, End, Timeout, Command, ValuesLayout };
struct Frame { uint8_t start=0, command=0; uint16_t payloadLength=0, receivedCrc=0, calculatedCrc=0; uint64_t receivedUs=0; bool crcOk=false, endOk=false; uint8_t payload[app_config::kVescMaxPayloadBytes]{}; };
struct FirmwareInfo { bool valid=false; uint8_t major=0, minor=0; char hardwareName[48]=""; char firmwareName[48]=""; uint8_t uuid[12]{}; bool uuidValid=false; };
struct Values { bool valid=false; float mosTempC=NAN,motorTempC=NAN,motorCurrentA=NAN,inputCurrentA=NAN,inputVoltageV=NAN,duty=NAN,erpm=NAN,ampHours=NAN,ampHoursCharged=NAN,wattHours=NAN,wattHoursCharged=NAN; int32_t tachometer=0,tachometerAbs=0; uint8_t fault=0; };
class Parser { public: bool feed(uint8_t byte,uint64_t receivedUs,Frame& out,ParseError& error); bool timeout(uint64_t nowUs,ParseError& error); void reset(); static uint16_t crc16(const uint8_t* data,size_t length); static bool parseFirmware(const Frame&,FirmwareInfo&); static bool parseCurrentValues(const Frame&,Values&); private: enum class State:uint8_t{Start,LengthShort,LengthLongHi,LengthLongLo,Payload,CrcHi,CrcLo,End}; State state_=State::Start; Frame frame_{}; uint16_t expected_=0,index_=0; uint64_t lastByteUs_=0; };
}