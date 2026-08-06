#pragma once
#include <cmath>
#include <stdint.h>
namespace production_control {
inline bool motorRelayRequired(float duty){return std::isfinite(duty)&&std::fabs(duty)>0.0f;}
struct ServoTuning { float minUs,neutralUs,maxUs; bool reversed; constexpr ServoTuning(float min=1480.0f,float neutral=1500.0f,float max=1520.0f,bool reverse=false):minUs(min),neutralUs(neutral),maxUs(max),reversed(reverse){} };
struct ServoResult { uint16_t pulseUs=1500; bool clamped=false,finite=true; };
class ServoMapper { public: explicit ServoMapper(const ServoTuning& tuning=ServoTuning{}); void reset(); ServoResult map(float normalized); uint16_t previousPulseUs()const{return previousUs_;} static bool valid(const ServoTuning& tuning); private: ServoTuning tuning_{}; uint16_t previousUs_=1500; };
class ServoSpikeFilter {
 public:
  explicit ServoSpikeFilter(float threshold=.10f);
  void reset(float accepted=0.0f);
  float apply(float desired);
  float accepted()const{return accepted_;}
 private:
  float threshold_,accepted_=0.0f;
  int8_t pendingDirection_=0;
};
class DutyRamp { public: constexpr DutyRamp(float maximum=.60f,float riseSeconds=2.0f,float fallSeconds=.35f):maximum_(maximum),riseSeconds_(riseSeconds),fallSeconds_(fallSeconds){} void setTarget(float duty); float step(float dtSeconds); void stopImmediate(){target_=applied_=0.0f;} float target()const{return target_;} float applied()const{return applied_;} bool active()const{return target_!=applied_;} private: float maximum_,riseSeconds_,fallSeconds_,target_=0.0f,applied_=0.0f; };
}
