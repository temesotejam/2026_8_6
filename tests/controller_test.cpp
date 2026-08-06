#include <cassert>
#include <cmath>
#include <iostream>

#include "../control/lib/production_control/src/competition_actuators.h"
#include "../control/lib/production_control/src/production_control.h"
#include "../communication/include/fixed_waypoints.h"

using namespace production_control;

SensorInput nominal(uint64_t now=1'000'000) {
  SensorInput input{};
  input.safety=AuthoritativeSafety::Running;
  input.nowUs=now;
  input.heartbeat=input.imuValid=input.tofValid=input.gnssValid=true;
  input.powerValid=input.vescValid=true;
  input.heartbeatUs=input.imuUs=input.tofUs=input.gnssUs=now;
  input.powerUs=input.vescUs=now;
  input.tofM=.45f;
  input.busVoltageV=12.0f;
  input.currentA=3.0f;
  input.vescErpm=1000.0f;
  input.groundSpeedMps=1.0f;
  return input;
}

void testAutoWaypoint() {
  Config config{};
  Controller controller(config);
  Waypoint route[]={{20.0f,0.0f},{30.0f,5.0f}};
  assert(controller.setWaypoints(route,2,1,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  assert(controller.setMode(ControlMode::AutoWaypoint,2,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  auto input=nominal();
  const auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.physicalGate);
  assert(output.propulsion>0);
  assert(output.waypointDistanceM>19.9f);
}

void testIndependentWaypointAndAttitudeModes() {
  Waypoint route[]={{20.0f,5.0f}};
  auto input=nominal();
  input.pitchRad=.10f;
  input.rollRad=-.08f;
  input.yawRad=.20f;

  Controller waypointOnly;
  assert(waypointOnly.setWaypoints(route,1,1,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  assert(waypointOnly.setMode(ControlMode::WaypointOnly,2,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  const auto navigation=waypointOnly.step(input);
  assert(navigation.safetyRequest==SafetyRequest::None);
  assert(navigation.physicalGate);
  assert(navigation.enabledMask==ManualAll);
  assert(navigation.leftFront==0&&navigation.rightFront==0);
  assert(navigation.rearYaw!=0);
  assert(navigation.propulsion>0);
  assert(navigation.uPitch==0&&navigation.uRoll==0&&navigation.uHeight==0);
  assert(!(navigation.flags&HeightDegraded));

  auto noTof=input;noTof.tofValid=false;
  const auto navigationWithoutTof=waypointOnly.step(noTof);
  assert(navigationWithoutTof.safetyRequest==SafetyRequest::None);
  assert(!(navigationWithoutTof.flags&HeightDegraded));

  Controller combined;
  assert(combined.setWaypoints(route,1,1,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  assert(combined.setMode(ControlMode::AutoWaypoint,2,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  const auto fullAuto=combined.step(input);
  assert(fullAuto.safetyRequest==SafetyRequest::None);
  assert(fullAuto.physicalGate);
  assert(fullAuto.leftFront!=0||fullAuto.rightFront!=0);
  assert(fullAuto.rearYaw!=0);
  assert(fullAuto.propulsion>0);

  Controller attitudeOnly;
  assert(attitudeOnly.setMode(ControlMode::AttitudeAssist,1,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  attitudeOnly.setManual({.7f,-.7f,.3f,.4f,ManualAll},input.nowUs);
  const auto assisted=attitudeOnly.step(input);
  assert(assisted.safetyRequest==SafetyRequest::None);
  assert(std::fabs(assisted.leftPrelimit)<=.50f);
  assert(std::fabs(assisted.rightPrelimit)<=.50f);
  assert(assisted.leftFront!=0||assisted.rightFront!=0);
  assert(assisted.rearYaw>0);
  assert(assisted.propulsion>0);

  // GNSS is required only by waypoint modes. Attitude assist must remain
  // usable indoors and on the bench without a satellite fix.
  auto attitudeWithoutGnss=input;
  attitudeWithoutGnss.gnssValid=false;
  attitudeWithoutGnss.gnssUs=0;
  attitudeOnly.setManual({.7f,-.7f,.3f,.4f,ManualAll},attitudeWithoutGnss.nowUs);
  const auto assistedWithoutGnss=attitudeOnly.step(attitudeWithoutGnss);
  assert(assistedWithoutGnss.safetyRequest==SafetyRequest::None);
  assert(assistedWithoutGnss.physicalGate);

  static_assert(!modeUsesWaypoint(ControlMode::Manual));
  static_assert(!modeUsesWaypoint(ControlMode::AttitudeAssist));
  static_assert(modeUsesWaypoint(ControlMode::WaypointOnly));
  static_assert(modeUsesWaypoint(ControlMode::AutoWaypoint));

  input.gnssValid=false;
  const auto noFix=waypointOnly.step(input);
  assert(noFix.safetyRequest==SafetyRequest::Fault);
  assert(noFix.reason==StopReason::GnssInvalid);

  // Attitude angle alone is never a stop condition. This also covers the
  // installed sensor's level reading near +/-pi.
  input=nominal(1'100'000);input.rollRad=.8f;input.pitchRad=3.0927f;
  const auto largeAttitude=waypointOnly.step(input);
  assert(largeAttitude.safetyRequest==SafetyRequest::None);
  assert(largeAttitude.physicalGate);
  assert(largeAttitude.reason!=StopReason::AttitudeDanger);

  // The trip remains available for a future unattended profile, but is not
  // active unless that profile explicitly enables it.
  Config unattendedConfig{};
  unattendedConfig.enableAttitudeDangerTrip=true;
  Controller unattended(unattendedConfig);
  assert(unattended.setWaypoints(route,1,1,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  assert(unattended.setMode(ControlMode::WaypointOnly,2,AuthoritativeSafety::Disarmed).ack==Ack::Accepted);
  const auto futureTrip=unattended.step(input);
  assert(futureTrip.safetyRequest==SafetyRequest::Fault);
  assert(futureTrip.reason==StopReason::AttitudeDanger);
}

void testFinalWaypointStops() {
  Controller controller;
  Waypoint route[]={{0.5f,0.0f}};
  controller.setWaypointReachRadius(1.5f,AuthoritativeSafety::Disarmed);
  controller.setWaypoints(route,1,1,AuthoritativeSafety::Disarmed);
  controller.setMode(ControlMode::WaypointOnly,2,AuthoritativeSafety::Disarmed);
  const auto output=controller.step(nominal());
  assert(output.waypointReached);
  assert(output.reason==StopReason::FinalWaypoint);
  assert(output.safetyRequest==SafetyRequest::Disarm);
  assert(!output.physicalGate);
  assert(output.leftFront==0&&output.rightFront==0&&output.rearYaw==0&&output.propulsion==0);
}

void testFixedWaypointCatalog() {
  using namespace fixed_waypoints;
  static_assert(kCount==8,"fixed waypoint count");
  assert(kPoints[0].name=='A');
  assert(std::fabs(kPoints[0].latitudeDeg-35.45327)<1e-9);
  assert(std::fabs(kPoints[0].longitudeDeg-136.07198)<1e-9);
  assert(kPoints[7].name=='H');
  assert(std::fabs(kPoints[7].latitudeDeg-35.44196)<1e-9);
  assert(std::fabs(kPoints[7].longitudeDeg-136.09429)<1e-9);
  assert(byIndex(8)==nullptr);
}

void testTofGracefulDegradation() {
  Controller controller;
  controller.setManual({0,0,0,.4f,ManualAll},100);
  controller.setMode(ControlMode::AttitudeAssist,1,AuthoritativeSafety::Disarmed);
  controller.setManual({0,0,0,.4f,ManualAll},1'000'000);
  auto input=nominal();
  input.tofValid=false;
  const auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.flags&HeightDegraded);
}

void testPowerProtection() {
  Controller controller;
  controller.setManual({0,0,0,.8f,ManualPropulsion},1'000'000);
  auto input=nominal();
  input.busVoltageV=9.0f;
  auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.flags&VoltageLimited);
  assert(output.propulsion<=.04f);

  input=nominal(2'000'000);
  input.busVoltageV=8.0f;
  controller.setManual({0,0,0,.8f,ManualPropulsion},input.nowUs);
  output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::Fault);
  assert(output.reason==StopReason::LowVoltage);
}

void testStallAndPitchProtection() {
  Controller controller;
  controller.setManual({0,0,0,.8f,ManualPropulsion},1'000'000);
  auto input=nominal();
  input.currentA=10.0f;
  input.vescErpm=0;
  auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  input.nowUs=2'100'001;
  input.heartbeatUs=input.powerUs=input.vescUs=input.imuUs=input.tofUs=input.gnssUs=input.nowUs;
  controller.setManual({0,0,0,.8f,ManualPropulsion},input.nowUs);
  output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::Fault);
  assert(output.reason==StopReason::MotorStall);

  Controller attitude;
  attitude.setMode(ControlMode::AttitudeAssist,1,AuthoritativeSafety::Disarmed);
  attitude.setManual({0,0,0,.8f,ManualAll},1'000'000);
  input=nominal();
  input.pitchRad=.40f;
  output=attitude.step(input);
  assert(output.flags&PitchPriority);
  assert(output.throttleLimit<=.25f);
}

void testActuatorMapping() {
  ServoMapper mapper(ServoTuning(1200,1500,1800,false));
  auto result=mapper.map(.5f);
  assert(result.pulseUs==1650);  // No servo rate limit: one 50 Hz update.
  result=mapper.map(-.5f);
  assert(result.pulseUs==1350);
  result=mapper.map(1.5f);
  assert(result.pulseUs==1800);
  assert(result.clamped);

  DutyRamp ramp(.6f,2.0f,.35f);
  ramp.setTarget(.6f);
  const float first=ramp.step(.02f);
  assert(first>0&&first<.02f);
  ramp.stopImmediate();
  assert(ramp.applied()==0);
  assert(!motorRelayRequired(0.0f));
  assert(motorRelayRequired(first));
  assert(motorRelayRequired(-first));
  assert(!motorRelayRequired(NAN));
}

void testServoSpikeFilter() {
  ServoSpikeFilter filter(.10f);
  assert(std::fabs(filter.apply(.05f)-.05f)<1e-6f);  // Small change is immediate.
  filter.reset();
  assert(filter.apply(.50f)==0);                    // One large frame is held.
  assert(filter.apply(0)==0);                       // A transient spike is rejected.
  assert(filter.apply(.50f)==0);
  assert(std::fabs(filter.apply(.45f)-.45f)<1e-6f); // Same-direction second frame confirms.
}

void testPartialManualWithoutSensors() {
  Controller controller;
  controller.setManual({.75f,-.5f,.4f,.8f,ManualLeft},1'000'000);
  SensorInput input{};
  input.safety=AuthoritativeSafety::Running;
  input.nowUs=input.heartbeatUs=1'000'000;
  input.heartbeat=true;
  auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.physicalGate);
  assert(output.enabledMask==ManualLeft);
  assert(output.leftFront>0);
  assert(output.rightFront==0&&output.rearYaw==0&&output.propulsion==0);

  controller.setManual({.75f,0,0,.8f,static_cast<uint8_t>(ManualLeft|ManualPropulsion)},1'100'000);
  input.nowUs=input.heartbeatUs=1'100'000;
  output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.physicalGate);
  assert(output.leftFront>0);
  assert(output.propulsion==0);
  assert(output.flags&PropulsionUnavailable);
  assert(output.reason==StopReason::VescStale);
}

void testServoOnlyManualIgnoresUnrelatedSensorTrips() {
  Controller controller;
  controller.setManual({.5f,0,0,0,ManualLeft},1'000'000);
  auto input=nominal();
  input.pitchRad=1.0f;
  input.rollRad=-1.0f;
  input.vescFault=true;
  const auto output=controller.step(input);
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.physicalGate);
  assert(output.enabledMask==ManualLeft);
  assert(output.leftFront>0);
  assert(output.rightFront==0&&output.rearYaw==0&&output.propulsion==0);

  Controller assisted;
  assisted.setMode(ControlMode::AttitudeAssist,1,AuthoritativeSafety::Disarmed);
  assisted.setManual({0,0,0,0,ManualLeft},input.nowUs);
  const auto assistedOutput=assisted.step(input);
  assert(assistedOutput.safetyRequest==SafetyRequest::None);
  assert(assistedOutput.physicalGate);
  assert(assistedOutput.reason!=StopReason::AttitudeDanger);
}

void testFullManualWithPropulsion() {
  Controller controller;
  controller.setManual({.5f,-.4f,.3f,.6f,ManualAll},1'000'000);
  const auto output=controller.step(nominal());
  assert(output.safetyRequest==SafetyRequest::None);
  assert(output.physicalGate);
  assert(output.enabledMask==ManualAll);
  assert(output.leftFront==.5f);
  assert(output.rightFront==-.4f);
  assert(output.rearYaw==.3f);
  assert(output.propulsion>0);
  assert(!(output.flags&PropulsionUnavailable));
}

int main() {
  testAutoWaypoint();
  testIndependentWaypointAndAttitudeModes();
  testFinalWaypointStops();
  testFixedWaypointCatalog();
  testTofGracefulDegradation();
  testPowerProtection();
  testStallAndPitchProtection();
  testActuatorMapping();
  testServoSpikeFilter();
  testPartialManualWithoutSensors();
  testServoOnlyManualIgnoresUnrelatedSensorTrips();
  testFullManualWithPropulsion();
  std::cout << "controller tests passed\n";
}
