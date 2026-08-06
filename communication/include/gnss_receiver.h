#pragma once

#include <Arduino.h>

#include "app_config.h"

namespace gnss {

enum ValidFlag : uint32_t {
  FixValid=1u<<0, LatitudeValid=1u<<1, LongitudeValid=1u<<2,
  AltitudeValid=1u<<3, SpeedValid=1u<<4, CourseValid=1u<<5,
  SatellitesValid=1u<<6, HdopValid=1u<<7, UtcTimeValid=1u<<8,
  UtcDateValid=1u<<9, FixTypeValid=1u<<10,
};

struct Latest {
  uint32_t flags=0;
  double latitude=0,longitude=0;
  float altitudeM=0,speedMps=0,courseDeg=0,hdop=0;
  uint8_t satellites=0,fixType=0;
  char utcTime[12]="",utcDate[8]="",lastType[7]="";
  uint64_t lastSentenceEndUs=0,lastParseUs=0,lastValidFixUs=0;
};

struct Counts {
  uint32_t bytes=0,sentences=0,validSentences=0,checksumErrors=0;
  uint32_t parseErrors=0,overlong=0,unfinished=0;
};

struct Sentence {
  char raw[app_config::kGnssMaxSentenceChars+1]="",type[7]="";
  uint8_t rawLength=0;
  bool checksumValid=false,parsed=false;
  uint64_t endUs=0,parseUs=0;
};

class Receiver {
 public:
  void begin(HardwareSerial& serial);
  bool feed(char value,uint64_t timestampUs,Sentence& sentence);
  void expire(uint64_t timestampUs);
  const Latest& latest() const{return latest_;}
  const Counts& totals() const{return total_;}

 private:
  bool finish(uint64_t timestampUs,Sentence& sentence);
  bool parse(char* raw,uint64_t timestampUs,Sentence& sentence);
  void count(uint32_t Counts::*field);
  static bool checksum(const char* raw);
  static bool number(const char* text,double& value);
  static bool coord(const char* text,const char* hemisphere,bool latitude,double& value);
  static void typeOf(const char* raw,char* type);
  char line_[app_config::kGnssInputLineChars+1]="";
  uint16_t length_=0;
  bool collecting_=false,overlong_=false;
  uint64_t startedUs_=0;
  Latest latest_{};
  Counts total_{};
};

}  // namespace gnss
