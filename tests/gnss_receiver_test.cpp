#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <string>

#include "gnss_receiver.h"

namespace {
bool feedLine(gnss::Receiver& receiver,const char* line,uint64_t& timestamp,gnss::Sentence& sentence){
  bool complete=false;
  for(const char* cursor=line;*cursor;++cursor){timestamp+=100;complete=receiver.feed(*cursor,timestamp,sentence)||complete;}
  return complete;
}

std::string nmea(const char* body){
  uint8_t checksum=0;
  for(const char* cursor=body;*cursor;++cursor)checksum^=static_cast<uint8_t>(*cursor);
  char suffix[8]{};std::snprintf(suffix,sizeof(suffix),"*%02X\r\n",checksum);
  return std::string("$")+body+suffix;
}
}

int main(){
  gnss::Receiver receiver;uint64_t timestamp=1'000'000;gnss::Sentence sentence{};
  const char* rmc="$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
  assert(feedLine(receiver,rmc,timestamp,sentence));
  assert(sentence.checksumValid&&sentence.parsed);
  const auto& fix=receiver.latest();
  assert((fix.flags&gnss::FixValid)!=0);
  assert(std::fabs(fix.latitude-48.1173)<1e-6);
  assert(std::fabs(fix.longitude-11.5166666667)<1e-6);
  assert(std::fabs(fix.speedMps-11.5235456)<1e-5);
  assert(std::fabs(fix.courseDeg-84.4)<1e-4);

  const char* gga="$GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*4D\r\n";
  assert(feedLine(receiver,gga,timestamp,sentence));
  assert(sentence.checksumValid&&sentence.parsed);
  assert(receiver.latest().fixType==1);
  assert(receiver.latest().satellites==8);
  assert(std::fabs(receiver.latest().hdop-.9f)<1e-6);

  // Empty NMEA fields must remain in place instead of shifting latitude,
  // longitude, or later values into the wrong field indexes.
  const std::string emptyOptional=nmea(
      "GPRMC,123521,A,4807.038,N,01131.000,E,,,230394,,,A");
  assert(feedLine(receiver,emptyOptional.c_str(),timestamp,sentence));
  assert(sentence.checksumValid&&sentence.parsed);
  assert(std::fabs(receiver.latest().latitude-48.1173)<1e-6);
  assert(std::fabs(receiver.latest().longitude-11.5166666667)<1e-6);
  assert(std::strcmp(sentence.type,"RMC")==0);

  const uint32_t checksumErrorsBefore=receiver.totals().checksumErrors;
  const char* corrupt="$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00\r\n";
  assert(feedLine(receiver,corrupt,timestamp,sentence));
  assert(!sentence.checksumValid);
  assert(receiver.totals().checksumErrors==checksumErrorsBefore+1);
  std::cout<<"GNSS receiver tests passed\n";
}
