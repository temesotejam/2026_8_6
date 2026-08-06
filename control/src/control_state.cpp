#include "control_state.h"

#include <math.h>

#include "app_config.h"

namespace control_state {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float wrapPi(float value) {
  while (value > kPi) value -= 2.0f * kPi;
  while (value < -kPi) value += 2.0f * kPi;
  return value;
}

uint32_t ageUs(uint64_t then, uint64_t now) {
  return then && now >= then
             ? static_cast<uint32_t>(min<uint64_t>(now - then, UINT32_MAX))
             : UINT32_MAX;
}

void bodyVector(float& x, float& y, float& z) {
  const float sx = x, sy = y, sz = z;
  x = app_config::kBnoBodyXx * sx + app_config::kBnoBodyXy * sy +
      app_config::kBnoBodyXz * sz;
  y = app_config::kBnoBodyYx * sx + app_config::kBnoBodyYy * sy +
      app_config::kBnoBodyYz * sz;
  z = app_config::kBnoBodyZx * sx + app_config::kBnoBodyZy * sy +
      app_config::kBnoBodyZz * sz;
}

}  // namespace

void State::ingestRotation(uint64_t sensorUs, uint64_t receivedUs,
                           uint8_t accuracy, float qx, float qy, float qz,
                           float qw) {
  const float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
  const bool finite = isfinite(norm) && norm > 1e-6f;

  portENTER_CRITICAL(&mux_);
  rotationSensorUs_ = sensorUs;
  rotationReceivedUs_ = receivedUs;
  rotationAccuracy_ = accuracy;
  rotationValid_ = finite;
  if (finite) {
    qw_ = qw / norm;
    qx_ = qx / norm;
    qy_ = qy / norm;
    qz_ = qz / norm;

    const float standardRoll =
        atan2f(2.0f * (qw_ * qx_ + qy_ * qz_),
               1.0f - 2.0f * (qx_ * qx_ + qy_ * qy_));
    const float standardPitch =
        asinf(constrain(2.0f * (qw_ * qy_ - qz_ * qx_), -1.0f, 1.0f));
    const float standardYaw =
        atan2f(2.0f * (qw_ * qz_ + qx_ * qy_),
               1.0f - 2.0f * (qy_ * qy_ + qz_ * qz_));

    // The first valid attitude after program start defines the boat's neutral
    // pose. This removes the installed sensor's fixed mounting orientation
    // (including the observed level reading near +/-pi) without assuming a
    // particular physical mounting rotation.
    if (!referenceAttitudeSet_) {
      referenceStandardRollRad_ = standardRoll;
      referenceStandardPitchRad_ = standardPitch;
      referenceStandardYawRad_ = standardYaw;
      referenceAttitudeSet_ = true;
    }

    // Preserve the controller convention: roll about +Y, pitch about -X, and
    // yaw about +Z, now expressed relative to the program-start pose.
    rollRad_ = wrapPi(standardPitch - referenceStandardPitchRad_);
    pitchRad_ = -wrapPi(standardRoll - referenceStandardRollRad_);
    yawRad_ = wrapPi(standardYaw - referenceStandardYawRad_);
  }
  portEXIT_CRITICAL(&mux_);
}

void State::ingestGyro(uint64_t sensorUs, uint64_t receivedUs, float x,
                       float y, float z) {
  bodyVector(x, y, z);
  const bool finite = isfinite(x) && isfinite(y) && isfinite(z);
  portENTER_CRITICAL(&mux_);
  gyroSensorUs_ = sensorUs;
  gyroReceivedUs_ = receivedUs;
  gyroValid_ = finite;
  if (finite) {
    rollRateRadS_ = y;
    pitchRateRadS_ = -x;
    yawRateRadS_ = z;
  }
  portEXIT_CRITICAL(&mux_);
}

void State::noteAccel(uint64_t receivedUs) {
  portENTER_CRITICAL(&mux_);
  accelReceivedUs_ = receivedUs;
  portEXIT_CRITICAL(&mux_);
}

void State::noteMagnetic(uint64_t receivedUs) {
  portENTER_CRITICAL(&mux_);
  magneticReceivedUs_ = receivedUs;
  portEXIT_CRITICAL(&mux_);
}

void State::updateGnss(uint64_t receivedUs, double latitudeDeg,
                       double longitudeDeg, float speedMps, float courseRad,
                       bool valid) {
  portENTER_CRITICAL(&mux_);
  latitudeDeg_ = latitudeDeg;
  longitudeDeg_ = longitudeDeg;
  groundSpeedMps_ = speedMps;
  courseRad_ = courseRad;
  gnssReceivedUs_ = receivedUs;
  gnssValid_ = valid;
  portEXIT_CRITICAL(&mux_);
}

void State::updateWaterDistance(uint64_t receivedUs, float distanceM,
                                bool valid) {
  portENTER_CRITICAL(&mux_);
  if (valid) waterDistanceM_ = distanceM;
  tofReceivedUs_ = receivedUs;
  waterValid_ = valid;
  portEXIT_CRITICAL(&mux_);
}

boat::EstimatedStatePayload State::snapshot(uint64_t nowUs) const {
  boat::EstimatedStatePayload state{};
  portENTER_CRITICAL(&mux_);
  state.estimateUs = rotationReceivedUs_;
  state.qw = qw_;
  state.qx = qx_;
  state.qy = qy_;
  state.qz = qz_;
  state.rollRad = rollRad_;
  state.pitchRad = pitchRad_;
  state.yawRad = yawRad_;
  state.rollRateRadS = rollRateRadS_;
  state.pitchRateRadS = pitchRateRadS_;
  state.yawRateRadS = yawRateRadS_;
  state.gyroBiasX = 0.0f;
  state.gyroBiasY = 0.0f;
  state.gyroBiasZ = 0.0f;
  state.latitudeDeg = latitudeDeg_;
  state.longitudeDeg = longitudeDeg_;
  state.groundSpeedMps = groundSpeedMps_;
  state.courseOverGroundRad = courseRad_;
  state.sideslipEstimateRad = NAN;
  state.waterDistanceM = waterDistanceM_;
  state.gyroAgeUs = ageUs(gyroReceivedUs_, nowUs);
  state.accelAgeUs = ageUs(accelReceivedUs_, nowUs);
  state.magAgeUs = ageUs(magneticReceivedUs_, nowUs);
  state.gnssAgeUs = ageUs(gnssReceivedUs_, nowUs);
  state.tofAgeUs = ageUs(tofReceivedUs_, nowUs);

  const bool attitudeFresh =
      rotationValid_ && ageUs(rotationReceivedUs_, nowUs) <=
                            app_config::kBnoAttitudeStaleUs;
  const bool gyroFresh = gyroValid_ &&
                         state.gyroAgeUs <= app_config::kBnoGyroStaleUs;
  state.attitudeHealth = static_cast<uint8_t>(
      !attitudeFresh
          ? boat::EstimateHealth::Invalid
          : (app_config::kBnoMountValidated && rotationAccuracy_ >= 2
                 ? boat::EstimateHealth::Valid
                 : boat::EstimateHealth::Degraded));
  state.yawHealth = state.attitudeHealth;
  state.navigationHealth = static_cast<uint8_t>(
      gnssValid_ && state.gnssAgeUs <= app_config::kGnssStateStaleUs
          ? boat::EstimateHealth::Valid
          : boat::EstimateHealth::Invalid);
  state.heightHealth = static_cast<uint8_t>(
      waterValid_ && state.tofAgeUs <= app_config::kTofStateStaleUs
          ? boat::EstimateHealth::Valid
          : boat::EstimateHealth::Invalid);
  if (!gyroFresh) {
    state.rollRateRadS = NAN;
    state.pitchRateRadS = NAN;
    state.yawRateRadS = NAN;
  }
  if (app_config::kBnoMountValidated) state.flags |= boat::EstimateMountValidated;
  portEXIT_CRITICAL(&mux_);
  return state;
}

}  // namespace control_state
