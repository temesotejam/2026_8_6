#include "gnss_receiver.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace gnss {
namespace {
bool equal(const char* a,const char* b){return strcmp(a,b)==0;}
bool nonEmpty(const char* text){return text&&*text;}
float knotsToMps(double knots){return static_cast<float>(knots*0.514444);}
}

void Receiver::begin(HardwareSerial& serial){
  serial.begin(app_config::kGnssBaud,SERIAL_8N1,app_config::kGnssRxPin,app_config::kGnssTxPin);
}

void Receiver::count(uint32_t Counts::*field){total_.*field+=1;}

bool Receiver::checksum(const char* raw){
  if(!raw||raw[0]!='$')return false;
  const char* star=strchr(raw,'*');
  if(!star||!isxdigit(star[1])||!isxdigit(star[2]))return false;
  uint8_t value=0;
  for(const char* cursor=raw+1;cursor<star;++cursor)value^=static_cast<uint8_t>(*cursor);
  char given[3]={star[1],star[2],0};
  return value==static_cast<uint8_t>(strtoul(given,nullptr,16));
}

bool Receiver::number(const char* text,double& value){
  if(!nonEmpty(text))return false;
  char* end=nullptr;value=strtod(text,&end);
  return end&&*end=='\0'&&isfinite(value);
}

bool Receiver::coord(const char* text,const char* hemisphere,bool latitude,double& value){
  double raw=0;if(!number(text,raw)||!nonEmpty(hemisphere))return false;
  const double degrees=floor(raw/100.0);
  const double minutes=raw-degrees*100.0;
  if(minutes<0||minutes>=60||degrees>(latitude?90.0:180.0))return false;
  value=degrees+minutes/60.0;
  const char direction=toupper(hemisphere[0]);
  if((latitude&&direction=='S')||(!latitude&&direction=='W'))value=-value;
  return (latitude&&(direction=='N'||direction=='S'))||(!latitude&&(direction=='E'||direction=='W'));
}

void Receiver::typeOf(const char* raw,char* type){
  type[0]=0;if(!raw||raw[0]!='$')return;
  const size_t length=strlen(raw+1);if(length<5)return;
  type[0]=raw[3];type[1]=raw[4];type[2]=raw[5];type[3]=0;
}

bool Receiver::parse(char* raw,uint64_t timestampUs,Sentence& sentence){
  char* star=strchr(raw,'*');if(star)*star=0;
  char* fields[24]{};uint8_t count=0;
  fields[count++]=raw+1;
  for(char* cursor=raw+1;*cursor&&count<24;++cursor){
    if(*cursor==','){*cursor=0;fields[count++]=cursor+1;}
  }
  if(!count)return false;
  const size_t idLength=strlen(fields[0]);if(idLength<5)return false;
  char kind[4]{};strncpy(kind,fields[0]+idLength-3,3);
  bool changed=false;double value=0;
  if(equal(kind,"RMC")&&count>=10){
    if(nonEmpty(fields[1])){strncpy(latest_.utcTime,fields[1],sizeof(latest_.utcTime)-1);latest_.flags|=UtcTimeValid;changed=true;}
    if(nonEmpty(fields[9])){strncpy(latest_.utcDate,fields[9],sizeof(latest_.utcDate)-1);latest_.flags|=UtcDateValid;changed=true;}
    const bool valid=equal(fields[2],"A");if(valid)latest_.flags|=FixValid;else latest_.flags&=~FixValid;
    double coordinate=0;
    if(coord(fields[3],fields[4],true,coordinate)){latest_.latitude=coordinate;latest_.flags|=LatitudeValid;changed=true;}
    if(coord(fields[5],fields[6],false,coordinate)){latest_.longitude=coordinate;latest_.flags|=LongitudeValid;changed=true;}
    if(number(fields[7],value)){latest_.speedMps=knotsToMps(value);latest_.flags|=SpeedValid;changed=true;}
    if(number(fields[8],value)){latest_.courseDeg=static_cast<float>(value);latest_.flags|=CourseValid;changed=true;}
    if(valid&&(latest_.flags&LatitudeValid)&&(latest_.flags&LongitudeValid))latest_.lastValidFixUs=timestampUs;
  }else if(equal(kind,"GGA")&&count>=10){
    if(nonEmpty(fields[1])){strncpy(latest_.utcTime,fields[1],sizeof(latest_.utcTime)-1);latest_.flags|=UtcTimeValid;changed=true;}
    double coordinate=0;
    if(coord(fields[2],fields[3],true,coordinate)){latest_.latitude=coordinate;latest_.flags|=LatitudeValid;changed=true;}
    if(coord(fields[4],fields[5],false,coordinate)){latest_.longitude=coordinate;latest_.flags|=LongitudeValid;changed=true;}
    if(number(fields[6],value)){latest_.fixType=static_cast<uint8_t>(value);latest_.flags|=FixTypeValid;if(value>0)latest_.flags|=FixValid;else latest_.flags&=~FixValid;changed=true;}
    if(number(fields[7],value)){latest_.satellites=static_cast<uint8_t>(value);latest_.flags|=SatellitesValid;changed=true;}
    if(number(fields[8],value)){latest_.hdop=static_cast<float>(value);latest_.flags|=HdopValid;changed=true;}
    if(number(fields[9],value)){latest_.altitudeM=static_cast<float>(value);latest_.flags|=AltitudeValid;changed=true;}
    if((latest_.flags&FixValid)&&(latest_.flags&LatitudeValid)&&(latest_.flags&LongitudeValid))latest_.lastValidFixUs=timestampUs;
  }else if(equal(kind,"VTG")&&count>=8){
    if(number(fields[1],value)){latest_.courseDeg=static_cast<float>(value);latest_.flags|=CourseValid;changed=true;}
    if(number(fields[5],value)){latest_.speedMps=knotsToMps(value);latest_.flags|=SpeedValid;changed=true;}
  }else if(equal(kind,"GSA")&&count>=3){
    if(number(fields[2],value)){latest_.fixType=static_cast<uint8_t>(value);latest_.flags|=FixTypeValid;changed=true;}
  }else if(equal(kind,"ZDA")&&count>=5){
    if(nonEmpty(fields[1])){strncpy(latest_.utcTime,fields[1],sizeof(latest_.utcTime)-1);latest_.flags|=UtcTimeValid;changed=true;}
  }
  latest_.lastParseUs=timestampUs;sentence.parseUs=timestampUs;return changed;
}

bool Receiver::finish(uint64_t timestampUs,Sentence& sentence){
  if(!collecting_)return false;
  collecting_=false;
  if(overlong_){count(&Counts::overlong);return false;}
  line_[length_]=0;if(!length_)return false;
  count(&Counts::sentences);sentence=Sentence{};
  strncpy(sentence.raw,line_,sizeof(sentence.raw)-1);
  sentence.rawLength=static_cast<uint8_t>(min<size_t>(length_,255));
  typeOf(line_,sentence.type);sentence.endUs=timestampUs;
  sentence.checksumValid=checksum(line_);
  if(!sentence.checksumValid){count(&Counts::checksumErrors);return true;}
  char copy[app_config::kGnssInputLineChars+1]{};strncpy(copy,line_,sizeof(copy)-1);
  sentence.parsed=parse(copy,timestampUs,sentence);
  if(sentence.parsed)count(&Counts::validSentences);else count(&Counts::parseErrors);
  strncpy(latest_.lastType,sentence.type,sizeof(latest_.lastType)-1);
  latest_.lastSentenceEndUs=timestampUs;return true;
}

bool Receiver::feed(char value,uint64_t timestampUs,Sentence& sentence){
  count(&Counts::bytes);
  if(value=='$'){collecting_=true;overlong_=false;length_=0;startedUs_=timestampUs;line_[length_++]=value;return false;}
  if(!collecting_)return false;
  if(value=='\r'||value=='\n')return finish(timestampUs,sentence);
  if(length_>=app_config::kGnssInputLineChars){overlong_=true;return false;}
  line_[length_++]=value;return false;
}

void Receiver::expire(uint64_t timestampUs){
  if(collecting_&&timestampUs-startedUs_>app_config::kGnssSentenceTimeoutMs*1000ULL){collecting_=false;length_=0;count(&Counts::unfinished);}
}

}  // namespace gnss
