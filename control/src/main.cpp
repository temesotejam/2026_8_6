#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <esp_timer.h>
#include <stddef.h>
#include <math.h>
#include "app_config.h"
#include "safety_state.h"
#include "bno_reader.h"
#include "control_state.h"
#include "ina226_reader.h"
#include "pca9685.h"
#include "vesc_protocol.h"
#include <production_control.h>
#include <competition_actuators.h>
#include <command_ingress.h>
#include <boat_protocol.h>
using namespace app_config;
WebServer web(kDebugHttpPort); HardwareSerial vescUart(1),linkUart(2); SparkFun_VL53L5CX tof; VL53L5CX_ResultsData tofData{}; bno::Reader imu; control_state::State controlState; Ina226 ina; Pca9685 pca; vesc::Parser vescParser; vesc::Values vescValues{}; InaSample latestIna{};
SafetyState safety=SafetyState::BOOT; TestProfile profile=TestProfile::None; bool tofReady=false,inaReady=false,pcaReady=false,navOriginSet=false,competitionOutputsOff=true,motorRelayEnabled=false; uint32_t bootId=0,lastVescReqMs=0,lastDutyMs=0; uint64_t lastInaUs=0,lastInaPollUs=0,lastTofUs=0,lastVescRxUs=0,testStartMs=0,lastDiagMs=0,lastHostHeartbeatMs=0,lastControlHeartbeatMs=0,lastNavSequence=0,navSequenceGaps=0,navCrcErrors=0,tofIntegrationMsActual=0; int32_t navOriginLatE7=0,navOriginLonE7=0; float navNorthM=0.0f,navEastM=0.0f; uint16_t servoTarget=kServoCenterUs,servoOutput=kServoCenterUs,tofResolutionActual=0,tofHzActual=0,tofI2cPacketBytes=0; uint8_t tofModeActual=0; uint32_t tofFrames=0,vescReq=0,vescResp=0,vescErrors=0,linkSeq=0,linkDrops=0,linkQueueHighWater=0,linkRxFrames=0,linkStopCommands=0,linkEstopCommands=0,gnssNavRx=0,gnssResultTx=0; boat::Decoder linkDecoder; struct TxItem{uint16_t length=0,offset=0;uint32_t sequence=0;uint8_t type=0,diagKind=0;bool bnoFrame=false,boundaryAck=false;uint8_t data[boat::kMaxEncoded]{};}; constexpr uint8_t kLinkQueueDepth=96; TxItem linkQueue[kLinkQueueDepth]{};uint8_t linkHead=0,linkTail=0,linkUsed=0;portMUX_TYPE linkMux=portMUX_INITIALIZER_UNLOCKED;
struct BenchmarkControl { bool prepared=false,active=false; uint32_t campaignId=0,startMs=0,minFreeHeap=UINT32_MAX,syntheticRx=0,tofFrames=0,tofIncomplete=0,maxTofReadUs=0,maxTofReadyUs=0,maxTofServiceUs=0,maxLinkRxUs=0,maxImuPollUs=0,maxImuRecoverUs=0,maxVescUs=0,maxLoopUs=0,vescReqStart=0,vescRespStart=0,vescErrStart=0; uint16_t phaseId=0; boat::BenchmarkPhase phase=boat::BenchmarkPhase::Baseline; boat::BenchmarkStatus status=boat::BenchmarkStatus::Pass; uint32_t flags=0,i2cClockHz=0; } bench;
volatile uint32_t bnoTaskMaxPollUs=0,bnoTaskMaxRecoverUs=0,bnoIntEdges=0,bnoTaskTimeouts=0,bnoLinkFrames=0,bnoLinkFrameDrops=0; volatile bool bnoBenchmarkActive=false,p1CaptureActive=false; TaskHandle_t bnoTaskHandle=nullptr; portMUX_TYPE bnoMetricsMux=portMUX_INITIALIZER_UNLOCKED;
  struct BnoTxDiag{uint64_t events=0,sendRequests=0,queueEnqueues=0,queueDrops=0,completed=0,encodedBytes=0;}; BnoTxDiag bnoTxDiag[6]{}; struct TypeTxDiag{uint64_t requested=0,enqueued=0,completed=0,encodedBytes=0;}; TypeTxDiag txTypeDiag[256]{};
  uint64_t txFramesCompleted=0,txBytesSent=0,txWriteCalls=0,txPartialWrites=0,txZeroWrites=0; uint32_t txMaxWriteUs=0,txMinFreeHeap=UINT32_MAX,txDiagResetGeneration=0; uint32_t lastEstimatedStateTxMs=0; uint32_t txTrialFirstSequence=0,txTrialLastSequence=0; bool txTrialHasSequence=false;
  struct P1BnoTimestampStats{uint32_t events=0,duplicates=0,reversals=0;bool hasPrevious=false;uint64_t firstSensorUs=0,lastSensorUs=0,firstRxUs=0,lastRxUs=0;int64_t sensorDeltaSum=0,sensorDeltaMin=0,sensorDeltaMax=0;uint64_t rxDeltaSum=0,rxDeltaMin=0,rxDeltaMax=0;}; P1BnoTimestampStats p1BnoTimestampStats[6]{}; bool p1BnoTimestampStatsActive=false;
constexpr uint16_t kTxSequenceHistoryDepth=2048;
enum TxHistoryFlag:uint8_t{TxHistAllocated=1u<<0,TxHistEnqueued=1u<<1,TxHistCompleted=1u<<2,TxHistDrop=1u<<3,TxHistPartial=1u<<4,TxHistZero=1u<<5,TxHistBoundaryAck=1u<<6};
struct TxSequenceHistoryEntry{uint32_t sequence=0;uint8_t type=0,flags=0;uint16_t encodedBytes=0;uint64_t allocatedUs=0,enqueueUs=0,writeStartUs=0,writeEndUs=0;};
TxSequenceHistoryEntry txHistory[kTxSequenceHistoryDepth]{};
volatile uint32_t linkSendsInFlight=0; volatile bool p1StartGate=false,p1StopGate=false; uint32_t txCaptureSuppressed=0;
struct PrimaryImuLatest { uint64_t accelUs=0,gyroUs=0,magneticUs=0,accelRxUs=0,gyroRxUs=0,magneticRxUs=0; uint8_t accelAccuracy=0,gyroAccuracy=0,magneticAccuracy=0; float accel[3]{},gyro[3]{},magnetic[3]{}; } primaryImu;

production_control::Controller competitionController{}; production_control::CommandIngress competitionIngress{}; production_control::Output competitionOutput{}; production_control::StopReason latchedCompetitionReason=production_control::StopReason::None; uint32_t lastCompetitionOutputTxMs=0,competitionPwmWrites=0,competitionPwmErrors=0; uint64_t lastCompetitionActuatorUs=0,lastVescControlUs=0; uint16_t competitionLeftPulseUs=0,competitionRightPulseUs=0,competitionRearPulseUs=0; float vescTargetDuty=0.0f,vescAppliedDuty=0.0f; production_control::DutyRamp vescRamp(kVescMaxDuty,kVescRampRiseSeconds,kVescRampFallSeconds); production_control::ServoMapper competitionLeftServo(production_control::ServoTuning(kCompetitionServoMinUs,kCompetitionServoNeutralUs,kCompetitionServoMaxUs,kCompetitionLeftReversed)),competitionRightServo(production_control::ServoTuning(kCompetitionServoMinUs,kCompetitionServoNeutralUs,kCompetitionServoMaxUs,kCompetitionRightReversed)),competitionRearServo(production_control::ServoTuning(kCompetitionServoMinUs,kCompetitionServoNeutralUs,kCompetitionServoMaxUs,kCompetitionRearReversed)); production_control::ServoSpikeFilter competitionLeftSpikeFilter(kCompetitionServoSpikeThreshold),competitionRightSpikeFilter(kCompetitionServoSpikeThreshold);
struct NavigationInput { double latitudeDeg=0,longitudeDeg=0; float speedMps=0,courseRad=0; uint64_t timestampUs=0; bool valid=false; } navigation{};
uint32_t waypointRevision=0,waypointRequestId=0; float waypointReachRadiusM=1.5f; boat::WaypointGeo waypointGeo[16]{}; uint8_t waypointGeoCount=0,waypointActive=0;
uint32_t lastPrimaryImuSnapshotTxMs=0; production_control::AuthoritativeSafety competitionSafety(SafetyState s){switch(s){case SafetyState::BOOT:return production_control::AuthoritativeSafety::Boot;case SafetyState::DISARMED:return production_control::AuthoritativeSafety::Disarmed;case SafetyState::ARMED_IDLE:return production_control::AuthoritativeSafety::ArmedIdle;case SafetyState::RUNNING:return production_control::AuthoritativeSafety::Running;case SafetyState::E_STOP:return production_control::AuthoritativeSafety::EStop;default:return production_control::AuthoritativeSafety::Fault;}}
void stop(); void printP1TxSnapshot(); void syncCompetitionWaypoints();
#include "modules/control_link.inc"
#include "modules/benchmark_service.inc"
#include "modules/sensor_service.inc"
#include "modules/navigation_service.inc"
#include "modules/capture_diagnostics.inc"
#include "modules/controller_service.inc"
#include "modules/debug_service.inc"
void setup(){pinMode(kMotorRelayPin,OUTPUT);digitalWrite(kMotorRelayPin,kMotorRelayActiveHigh?LOW:HIGH);motorRelayEnabled=false;Serial.begin(115200);
  production_control::Config competitionConfig{};competitionConfig.kpPitch=kCompetitionKpPitch;competitionConfig.kdPitch=kCompetitionKdPitch;competitionConfig.kpRoll=kCompetitionKpRoll;competitionConfig.kdRoll=kCompetitionKdRoll;competitionConfig.kpHeight=kCompetitionKpHeight;competitionConfig.kpYaw=kCompetitionKpYaw;competitionConfig.kdYaw=kCompetitionKdYaw;competitionConfig.targetHeightM=kCompetitionTargetHeightM;competitionConfig.attitudeServoLimit=kCompetitionAttitudeServoLimit;competitionConfig.autoPropulsion=kCompetitionAutoPropulsion;competitionConfig.waypointReachM=kCompetitionWaypointReachM;competitionConfig.losLookaheadM=kCompetitionLosLookaheadM;competitionConfig.enableAttitudeDangerTrip=kEnableAttitudeDangerTrip;competitionConfig.attitudeStopRad=kAttitudeDangerTripRad;competitionConfig.lowVoltageV=kLowVoltageTripV;competitionConfig.criticalVoltageV=kCriticalVoltageTripV;competitionConfig.overCurrentA=kOverCurrentTripA;competitionConfig.criticalCurrentA=kCriticalCurrentTripA;competitionConfig.stallCurrentA=kStallCurrentA;competitionConfig.stallErpm=kStallErpm;competitionConfig.cavitationErpm=kCavitationErpm;competitionConfig.cavitationCurrentA=kCavitationCurrentA;competitionConfig.cavitationSpeedMps=kCavitationSpeedMps;competitionConfig.powerStaleUs=kPowerStateStaleUs;competitionConfig.vescStaleUs=kVescStateStaleUs;competitionController.setConfig(competitionConfig);
  delay(100);bootId=esp_random();Wire.begin(kPeripheralSdaPin,kPeripheralSclPin,kPeripheralI2cHz);Wire.setTimeOut(20);scan();pcaReady=pca.begin();safeOutputs();ina.profile=InaProfile::Balanced;inaReady=ina.begin();tofReady=configureTof(experiment_config::kTofProfile);imu.begin();vescUart.begin(kVescUartBaud,SERIAL_8N1,kVescRxPin,kVescTxPin);linkUart.begin(kLinkBaud,SERIAL_8N1,kLinkRxPin,kLinkTxPin);xTaskCreatePinnedToCore(linkTxTask,"LinkTx",4096,nullptr,kLinkTxTaskPriority,nullptr,0);// BNO task reads reports and publishes the direct Rotation Vector state.
  xTaskCreatePinnedToCore(bnoTask,"Bno",8192,nullptr,kBnoTaskPriority,&bnoTaskHandle,0);attachInterrupt(digitalPinToInterrupt(kBnoIntPin),bnoIntIsr,FALLING);startDebugWifi();setState(SafetyState::DISARMED);Serial.printf("%s %s boot=%lu\n",kFirmwareName,kFirmwareVersion,bootId);}
#include "modules/diagnostics_service.inc"
