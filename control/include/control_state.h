#pragma once

#include <Arduino.h>
#include <math.h>
#include <boat_protocol.h>

namespace control_state {

// Thread-safe adapter from sensor outputs to the state consumed by the
// controller. Attitude comes directly from the BNO08X Rotation Vector; this
// class does not integrate gyro data and does not run an attitude filter.
class State {
 public:
  void ingestRotation(uint64_t sensorUs, uint64_t receivedUs, uint8_t accuracy,
                      float qx, float qy, float qz, float qw);
  void ingestGyro(uint64_t sensorUs, uint64_t receivedUs,
                  float x, float y, float z);
  void noteAccel(uint64_t receivedUs);
  void noteMagnetic(uint64_t receivedUs);
  void updateGnss(uint64_t receivedUs, double latitudeDeg,
                  double longitudeDeg, float speedMps, float courseRad,
                  bool valid);
  void updateWaterDistance(uint64_t receivedUs, float distanceM, bool valid);

  boat::EstimatedStatePayload snapshot(uint64_t nowUs) const;

 private:
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  float qw_ = 1.0f, qx_ = 0.0f, qy_ = 0.0f, qz_ = 0.0f;
  float rollRad_ = NAN, pitchRad_ = NAN, yawRad_ = NAN;
  float referenceStandardRollRad_ = 0.0f;
  float referenceStandardPitchRad_ = 0.0f;
  float referenceStandardYawRad_ = 0.0f;
  float rollRateRadS_ = NAN, pitchRateRadS_ = NAN, yawRateRadS_ = NAN;
  double latitudeDeg_ = NAN, longitudeDeg_ = NAN;
  float groundSpeedMps_ = NAN, courseRad_ = NAN, waterDistanceM_ = NAN;

  uint64_t rotationSensorUs_ = 0, rotationReceivedUs_ = 0;
  uint64_t gyroSensorUs_ = 0, gyroReceivedUs_ = 0;
  uint64_t accelReceivedUs_ = 0, magneticReceivedUs_ = 0;
  uint64_t gnssReceivedUs_ = 0, tofReceivedUs_ = 0;
  uint8_t rotationAccuracy_ = 0;
  bool rotationValid_ = false, referenceAttitudeSet_ = false;
  bool gyroValid_ = false;
  bool gnssValid_ = false, waterValid_ = false;
};

}  // namespace control_state
