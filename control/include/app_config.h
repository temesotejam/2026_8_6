#pragma once
#include <Arduino.h>
#include "experiment_config.h"

namespace app_config {

constexpr char kFirmwareName[]="xiao-boat-control-integration";
constexpr char kFirmwareVersion[]="1.2.4-servo-spike-filter";
constexpr int kPeripheralSdaPin=D1,kPeripheralSclPin=D0;
constexpr int kBnoRstPin=D2,kBnoIntPin=D3,kBnoSdaPin=D4,kBnoSclPin=D5;
constexpr int kLinkRxPin=D6,kLinkTxPin=D7,kVescRxPin=D8,kVescTxPin=D9,kMotorRelayPin=D10;
constexpr bool kMotorRelayActiveHigh=true;
constexpr uint8_t kBnoAddress=0x4A,kBnoAlternateAddress=0x4B,kTofAddress=0x29,kInaAddress=0x44,kPcaAddress=0x40;
constexpr uint32_t kBnoI2cHz=100000UL,kPeripheralI2cHz=experiment_config::kPeripheralI2cHz,kAccelGyroIntervalUs=20000UL,kRotationIntervalUs=20000UL,kMagneticIntervalUs=50000UL;
constexpr uint16_t kBnoEventQueueDepth=96; constexpr uint8_t kBnoServiceCallBudget=8;
// The TX drain must preempt active-INT BNO servicing on the same core.
constexpr uint32_t kBnoTaskFallbackMs=2UL; constexpr UBaseType_t kBnoTaskPriority=3,kLinkTxTaskPriority=4;
constexpr uint32_t kTofFrequencyHz=10UL,kInaSampleUs=20000UL,kServoControlUs=20000UL;
constexpr uint32_t kOscillatorHz=25000000UL; constexpr float kServoPwmHz=50.0f; constexpr uint8_t kServoChannel=0;
constexpr uint16_t kHardMinUs=500,kHardMaxUs=2500,kServoCenterUs=1500,kServoIntegrationMinUs=1400,kServoIntegrationMaxUs=1600,kServoSlewUsPerUpdate=5;
constexpr float kVescMaxDuty=0.60f,kVescTestDuty=0.10f,kVescRampRiseSeconds=2.0f,kVescRampFallSeconds=.35f; constexpr uint32_t kVescUartBaud=115200UL,kVescRequestIntervalMs=20UL,kVescFrameTimeoutMs=100UL,kVescMaxPayloadBytes=512,kVescKeepaliveMs=50UL,kVescControlIntervalUs=20000UL,kMaxTestMs=300000UL;
constexpr uint32_t kLinkBaud=921600UL,kProtocolVersion=1,kLinkHeartbeatTimeoutMs=500UL; constexpr bool kRequireHostHeartbeat=false;
constexpr bool kBenchmarkEnable=false,kReplayEnable=false,kCompetitionControlEnable=true;
constexpr bool kCompetitionHardwareEnable=true,kDryRunActuators=false;
constexpr bool kPhysicalOutputCompileEnabled=true;
constexpr bool kEnableIna226=true;
constexpr uint32_t kGnssNavExpectedIntervalMs=100UL,kControlHeartbeatIntervalMs=100UL,kLinkFailSafeTimeoutMs=500UL;
// Production linkage calibration. Normalized -1..+1 spans the usable mechanical range.
constexpr uint8_t kCompetitionLeftChannel=0,kCompetitionRightChannel=1,kCompetitionRearChannel=2;
constexpr float kCompetitionServoMinUs=1200.0f,kCompetitionServoNeutralUs=1500.0f,kCompetitionServoMaxUs=1800.0f;
constexpr bool kCompetitionLeftReversed=false,kCompetitionRightReversed=false,kCompetitionRearReversed=false;
constexpr float kCompetitionKpPitch=0.80f,kCompetitionKdPitch=0.10f,kCompetitionKpRoll=1.25f,kCompetitionKdRoll=0.22f,kCompetitionKpHeight=0.75f,kCompetitionKpYaw=0.90f,kCompetitionKdYaw=0.12f;
constexpr float kCompetitionTargetHeightM=0.45f,kCompetitionAutoPropulsion=0.55f,kCompetitionWaypointReachM=1.5f,kCompetitionLosLookaheadM=4.0f;
// With the current +/-300 us calibrated range, 0.50 is approximately +/-15 deg.
constexpr float kCompetitionAttitudeServoLimit=0.50f;
// Ignore a one-frame attitude-servo jump larger than about 3 degrees. A
// second 50 Hz frame in the same direction confirms it with only 20 ms delay.
constexpr float kCompetitionServoSpikeThreshold=0.10f;
// Current operation is continuously supervised by a person. Set true only
// when an unattended operating procedure defines and validates a safe angle.
constexpr bool kEnableAttitudeDangerTrip=false;
constexpr float kAttitudeDangerTripRad=0.61f;
constexpr bool kPrimaryBnoEnabled=true,kSecondaryBnoEnabled=false;
constexpr uint32_t kBnoAttitudeStaleUs=100000UL,kBnoGyroStaleUs=50000UL;
constexpr uint32_t kGnssStateStaleUs=500000UL,kTofStateStaleUs=250000UL;
constexpr float kTofMaxSpreadM=0.25f;
// The control node has no radio role. Integration status is served by the communication node.
constexpr bool kEnableTemporaryDebugWifi=false; constexpr char kDebugApSsid[]="XIAO-BOAT-DEBUG",kDebugApPass[]="12345678"; constexpr uint16_t kDebugHttpPort=80;
constexpr uint16_t kInaConfig=0x08DF,kInaCalibration=0x0800; constexpr uint16_t kConfig=kInaConfig,kCalibration=kInaCalibration; constexpr float kShuntOhm=0.002f,kCurrentLsbA=0.00125f,kPowerLsbW=0.03125f;
constexpr bool kEnableOverCurrentTrip=true,kEnableLowVoltageTrip=true; constexpr float kOverCurrentTripA=22.0f,kCriticalCurrentTripA=28.0f,kLowVoltageTripV=9.5f,kCriticalVoltageTripV=8.5f;
constexpr uint32_t kPowerStateStaleUs=200000UL,kVescStateStaleUs=300000UL;
// VESC reports electrical RPM (ERPM). No external shaft-angle sensor is used.
constexpr float kStallCurrentA=8.0f,kStallErpm=100.0f,kCavitationErpm=4500.0f,kCavitationCurrentA=2.0f,kCavitationSpeedMps=0.35f;
constexpr uint32_t kDiagnosticIntervalMs=1000UL,kBnoNoDataTimeoutMs=3000UL,kReinitIntervalMs=2000UL;
constexpr uint16_t kLinkMaxPayload=768,kLinkTxQueueDepth=64;
constexpr uint16_t kLinkRxByteBudget=512;
// Production assumption: sensor axes are mounted in the body frame.
constexpr bool kBnoMountValidated=true;
constexpr float kBnoBodyXx=1,kBnoBodyXy=0,kBnoBodyXz=0,kBnoBodyYx=0,kBnoBodyYy=1,kBnoBodyYz=0,kBnoBodyZx=0,kBnoBodyZy=0,kBnoBodyZz=1;
constexpr uint32_t kEstimatedStateTxIntervalMs=100UL,kPrimaryImuSnapshotTxIntervalMs=50UL;
}
namespace cfg=app_config;
