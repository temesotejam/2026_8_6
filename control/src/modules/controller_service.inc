// Production waypoint, controller, actuator, and safety path.
bool sendWaypointAckFromHandler(void*,const boat::WaypointAckPayload& ack){
  return linkSend(boat::Type::WaypointAck,reinterpret_cast<const uint8_t*>(&ack),sizeof(ack));
}

void syncCompetitionWaypoints(){
  if(!navOriginSet||!waypointGeoCount||safety!=SafetyState::DISARMED)return;
  production_control::Waypoint points[16]{};
  const double originLat=navOriginLatE7/1e7,originLon=navOriginLonE7/1e7;
  const double eastScale=111320.0*cos(originLat*PI/180.0);
  for(uint8_t i=0;i<waypointGeoCount;++i){
    points[i].northM=static_cast<float>((waypointGeo[i].latitudeDeg-originLat)*111320.0);
    points[i].eastM=static_cast<float>((waypointGeo[i].longitudeDeg-originLon)*eastScale);
  }
  competitionController.setWaypointReachRadius(waypointReachRadiusM,competitionSafety(safety));
  competitionController.setWaypoints(points,waypointGeoCount,waypointRequestId,competitionSafety(safety));
}

void handleWaypointSet(const boat::Frame& frame){
  enum : uint8_t { Accepted=0,Rejected=1,Duplicate=2 };
  enum : uint8_t { None=0,Running=1,Estop=2,Length=3,Crc=4,Range=5,Empty=6,Revision=7,State=8 };
  boat::WaypointAckPayload ack{};ack.status=Rejected;ack.reason=Length;ack.activeIndex=waypointActive;ack.count=waypointGeoCount;
  if(frame.header.length>=4)memcpy(&ack.requestId,frame.payload,4);
  if(frame.header.length>=8)memcpy(&ack.revision,frame.payload+4,4);
  if(frame.header.length!=sizeof(boat::WaypointSetPayload)){ack.canonicalCrc=boat::canonicalCrc(&ack,offsetof(boat::WaypointAckPayload,canonicalCrc));sendWaypointAckFromHandler(nullptr,ack);return;}
  boat::WaypointSetPayload request{};memcpy(&request,frame.payload,sizeof(request));ack.requestId=request.requestId;ack.revision=request.revision;
  if(request.canonicalCrc!=boat::canonicalCrc(&request,offsetof(boat::WaypointSetPayload,canonicalCrc)))ack.reason=Crc;
  else if(safety!=SafetyState::DISARMED)ack.reason=safety==SafetyState::RUNNING?Running:(safety==SafetyState::E_STOP?Estop:State);
  else if(request.revision==waypointRevision){ack.status=Duplicate;ack.reason=Revision;}
  else if(request.revision<waypointRevision)ack.reason=Revision;
  else if(request.action!=1||request.count==0||request.count>16||!isfinite(request.reachRadiusM)||request.reachRadiusM<.5f||request.reachRadiusM>20.0f)ack.reason=request.count==0?Empty:Range;
  else {
    bool valid=true;for(uint8_t i=0;i<request.count;++i){const auto& point=request.points[i];if(!isfinite(point.latitudeDeg)||!isfinite(point.longitudeDeg)||point.latitudeDeg<-90||point.latitudeDeg>90||point.longitudeDeg<-180||point.longitudeDeg>180){valid=false;break;}}
    if(!valid)ack.reason=Range;
    else {waypointRevision=request.revision;waypointRequestId=request.requestId;waypointGeoCount=request.count;waypointActive=0;waypointReachRadiusM=request.reachRadiusM;memcpy(waypointGeo,request.points,sizeof(boat::WaypointGeo)*request.count);ack.status=Accepted;ack.reason=None;ack.activeIndex=0;ack.count=waypointGeoCount;syncCompetitionWaypoints();}
  }
  ack.canonicalCrc=boat::canonicalCrc(&ack,offsetof(boat::WaypointAckPayload,canonicalCrc));sendWaypointAckFromHandler(nullptr,ack);
}

void sendCompetitionIngressAck(const production_control::CommandIngressResult& result){
  boat::ControlCommandAckPayload ack{};ack.requestId=result.requestId;ack.commandSequence=result.commandSequence;
  ack.commandType=result.commandType;ack.disposition=static_cast<uint8_t>(result.result.ack);
  ack.safetyState=static_cast<uint8_t>(safety);ack.controlMode=static_cast<uint8_t>(competitionController.mode());
  ack.reason=result.result.reason;ack.receivedUs=result.receivedUs;ack.appliedUs=result.appliedUs;
  ack.canonicalCrc=boat::canonicalCrc(&ack,offsetof(boat::ControlCommandAckPayload,canonicalCrc));
  linkSend(boat::Type::ControlCommandAck,reinterpret_cast<uint8_t*>(&ack),sizeof(ack));
}
void handleCompetitionCommand(const boat::Frame& frame){
  const auto result=competitionIngress.process(frame,competitionSafety(safety),competitionController,nowUs());
  if(result.ackGenerated)sendCompetitionIngressAck(result);
}
void printCompetitionCommandDiag(){
  const auto& metrics=competitionIngress.metrics();
  Serial.printf("COMPETITION_CMD applied=%lu rejected=%lu duplicate=%lu malformed=%lu pwm_writes=%lu pwm_errors=%lu\n",
    static_cast<unsigned long>(metrics.appliedCommands),static_cast<unsigned long>(metrics.rejectedCommands),
    static_cast<unsigned long>(metrics.duplicates),static_cast<unsigned long>(metrics.malformed),
    static_cast<unsigned long>(competitionPwmWrites),static_cast<unsigned long>(competitionPwmErrors));
}

bool productionPreflightReady(){
  const uint64_t now=nowUs();
  const auto estimated=controlState.snapshot(now);
  const auto mode=competitionController.mode();
  const bool imuOk=imu.ready()&&estimated.attitudeHealth!=static_cast<uint8_t>(boat::EstimateHealth::Invalid)&&estimated.gyroAgeUs<=kBnoGyroStaleUs;
  const bool powerOk=(latestIna.valid&&lastInaUs&&now-lastInaUs<=kPowerStateStaleUs)||(vescValues.valid&&lastVescRxUs&&now-lastVescRxUs<=kVescStateStaleUs&&isfinite(vescValues.inputVoltageV)&&vescValues.inputVoltageV>0);
  const bool vescOk=vescValues.valid&&lastVescRxUs&&now-lastVescRxUs<=kVescStateStaleUs;
  const bool linkOk=lastHostHeartbeatMs&&millis()-lastHostHeartbeatMs<=kLinkFailSafeTimeoutMs;
  const bool waypointMode=production_control::modeUsesWaypoint(mode);
  const bool navOk=!waypointMode||
                   (navigation.valid&&competitionController.waypointCount()>0);
  const bool manualOk=waypointMode||
                      (competitionController.manualReceivedUs()&&now-competitionController.manualReceivedUs()<=500000ULL);
  const bool actuatorConfigOk=kCompetitionLeftChannel!=kCompetitionRightChannel&&kCompetitionLeftChannel!=kCompetitionRearChannel&&kCompetitionRightChannel!=kCompetitionRearChannel&&
                              kCompetitionServoMinUs<kCompetitionServoNeutralUs&&kCompetitionServoNeutralUs<kCompetitionServoMaxUs;
  const bool baseOk=pcaReady&&linkOk&&manualOk&&actuatorConfigOk;
  if(mode==production_control::ControlMode::Manual)return baseOk&&competitionController.manualOutputMask()!=0;
  if(mode==production_control::ControlMode::AttitudeAssist||mode==production_control::ControlMode::HeadingHold)return baseOk&&imuOk;
  return baseOk&&imuOk&&navOk&&powerOk&&vescOk;
}

void applyCompetitionActuators(const production_control::Output& output,uint64_t now){
  if(!kCompetitionHardwareEnable)return;
  if(lastCompetitionActuatorUs&&now-lastCompetitionActuatorUs<kServoControlUs)return;
  const float dt=lastCompetitionActuatorUs?(now-lastCompetitionActuatorUs)*1.0e-6f:.02f;
  lastCompetitionActuatorUs=now;
  if(safety!=SafetyState::RUNNING||output.safetyRequest!=production_control::SafetyRequest::None||!output.physicalGate){
    if(!competitionOutputsOff||motorRelayEnabled||vescTargetDuty!=0.0f||vescAppliedDuty!=0.0f)safeOutputs();
    return;
  }
  const bool leftEnabled=output.enabledMask&production_control::ManualLeft;
  const bool rightEnabled=output.enabledMask&production_control::ManualRight;
  const bool rearEnabled=output.enabledMask&production_control::ManualRear;
  float leftCommand=output.leftFront,rightCommand=output.rightFront;
  if(production_control::modeUsesAttitude(output.mode)){
    leftCommand=competitionLeftSpikeFilter.apply(leftCommand);
    rightCommand=competitionRightSpikeFilter.apply(rightCommand);
  }else{
    competitionLeftSpikeFilter.reset(leftCommand);competitionRightSpikeFilter.reset(rightCommand);
  }
  const auto left=leftEnabled?competitionLeftServo.map(leftCommand):production_control::ServoResult{};
  const auto right=rightEnabled?competitionRightServo.map(rightCommand):production_control::ServoResult{};
  const auto rear=rearEnabled?competitionRearServo.map(output.rearYaw):production_control::ServoResult{};
  if(!leftEnabled)competitionLeftServo.reset();if(!rightEnabled)competitionRightServo.reset();if(!rearEnabled)competitionRearServo.reset();
  bool ok=pcaReady&&left.finite&&right.finite&&rear.finite;
  if(ok){
    ok=(leftEnabled?pca.setPulse(kCompetitionLeftChannel,left.pulseUs):pca.setFullOff(kCompetitionLeftChannel))&&
       (rightEnabled?pca.setPulse(kCompetitionRightChannel,right.pulseUs):pca.setFullOff(kCompetitionRightChannel))&&
       (rearEnabled?pca.setPulse(kCompetitionRearChannel,rear.pulseUs):pca.setFullOff(kCompetitionRearChannel));
    if(ok){competitionPwmWrites+=3;competitionLeftPulseUs=leftEnabled?left.pulseUs:0;competitionRightPulseUs=rightEnabled?right.pulseUs:0;competitionRearPulseUs=rearEnabled?rear.pulseUs:0;competitionOutputsOff=!(leftEnabled||rightEnabled||rearEnabled);}
  }
  if(!ok){++competitionPwmErrors;safeOutputs();setState(SafetyState::FAULT);return;}
  vescTargetDuty=constrain(output.propulsion,0.0f,1.0f)*kVescMaxDuty;
  vescRamp.setTarget(vescTargetDuty);
  const float applied=vescRamp.step(dt);vescAppliedDuty=applied;vescDutyRaw(applied);
}

uint32_t controlAge(uint64_t stamp,uint64_t now){return stamp&&now>=stamp?static_cast<uint32_t>(min<uint64_t>(now-stamp,UINT32_MAX)):UINT32_MAX;}

void sendProductionTelemetry(const boat::EstimatedStatePayload& estimated,const production_control::SensorInput& input,uint64_t now){
  if(competitionOutput.reason==production_control::StopReason::None)competitionOutput.reason=latchedCompetitionReason;
  boat::ControlOutputPayload output{};output.timestampUs=now;output.leftFrontWing=competitionOutput.leftFront;output.rightFrontWing=competitionOutput.rightFront;output.rearYaw=competitionOutput.rearYaw;output.propulsion=competitionOutput.propulsion;output.leftPrelimit=competitionOutput.leftPrelimit;output.rightPrelimit=competitionOutput.rightPrelimit;output.rearYawPrelimit=competitionOutput.rearPrelimit;output.propulsionPrelimit=competitionOutput.propulsionPrelimit;output.uHeight=competitionOutput.uHeight;output.uPitch=competitionOutput.uPitch;output.uRoll=competitionOutput.uRoll;output.targetCourseRad=competitionOutput.targetYaw;output.courseErrorRad=competitionOutput.courseErrorRad;output.waypointDistanceM=competitionOutput.waypointDistanceM;output.waypointIndex=competitionOutput.activeWaypoint;output.safety=static_cast<uint8_t>(safety);output.stopReason=static_cast<uint8_t>(competitionOutput.reason);output.reservedControl=competitionOutput.enabledMask;output.valid=competitionOutput.physicalGate?1:0;
  linkSend(boat::Type::ControlOutput,reinterpret_cast<uint8_t*>(&output),sizeof(output));
  boat::ControlSnapshotPayload snapshot{};snapshot.timestampUs=now;snapshot.cycle=competitionPwmWrites/3;snapshot.waypointRevision=waypointRevision;snapshot.gnssAgeUs=controlAge(input.gnssUs,now);snapshot.imuAgeUs=controlAge(input.imuUs,now);snapshot.tofAgeUs=controlAge(input.tofUs,now);snapshot.latitudeDeg=navigation.latitudeDeg;snapshot.longitudeDeg=navigation.longitudeDeg;snapshot.speedMps=navigation.speedMps;snapshot.gnssCourseRad=navigation.courseRad;snapshot.localNorthM=navNorthM;snapshot.localEastM=navEastM;snapshot.rollRad=estimated.rollRad;snapshot.pitchRad=estimated.pitchRad;snapshot.yawRad=estimated.yawRad;snapshot.rollRateRadS=estimated.rollRateRadS;snapshot.pitchRateRadS=estimated.pitchRateRadS;snapshot.yawRateRadS=estimated.yawRateRadS;snapshot.tofFilteredM=estimated.waterDistanceM;snapshot.heightErrorM=kCompetitionTargetHeightM-estimated.waterDistanceM;snapshot.uHeight=competitionOutput.uHeight;snapshot.uPitch=competitionOutput.uPitch;snapshot.uRoll=competitionOutput.uRoll;snapshot.uYaw=competitionOutput.uYaw;snapshot.frontCommon=competitionOutput.uHeight+competitionOutput.uPitch;snapshot.frontDifferential=competitionOutput.uRoll;snapshot.leftFrontWing=competitionOutput.leftFront;snapshot.rightFrontWing=competitionOutput.rightFront;snapshot.rearYaw=competitionOutput.rearYaw;snapshot.propulsion=competitionOutput.propulsion;snapshot.leftPrelimit=competitionOutput.leftPrelimit;snapshot.rightPrelimit=competitionOutput.rightPrelimit;snapshot.rearYawPrelimit=competitionOutput.rearPrelimit;snapshot.propulsionPrelimit=competitionOutput.propulsionPrelimit;snapshot.gnssValid=input.gnssValid;snapshot.imuValid=input.imuValid;snapshot.tofValid=input.tofValid;snapshot.heightValid=input.tofValid;snapshot.waypointReached=competitionOutput.waypointReached;snapshot.outputValid=competitionOutput.physicalGate;snapshot.state=static_cast<uint8_t>(safety);snapshot.safetyReason=static_cast<uint8_t>(competitionOutput.reason);snapshot.mode=static_cast<uint8_t>(competitionController.mode());snapshot.activeWaypoint=competitionOutput.activeWaypoint;snapshot.targetBearingRad=competitionOutput.targetYaw;snapshot.courseErrorRad=competitionOutput.courseErrorRad;snapshot.waypointDistanceM=competitionOutput.waypointDistanceM;
  if(waypointGeoCount&&competitionOutput.activeWaypoint<waypointGeoCount){snapshot.targetWaypointLatitudeDeg=waypointGeo[competitionOutput.activeWaypoint].latitudeDeg;snapshot.targetWaypointLongitudeDeg=waypointGeo[competitionOutput.activeWaypoint].longitudeDeg;}
  linkSend(boat::Type::ControlSnapshot,reinterpret_cast<uint8_t*>(&snapshot),sizeof(snapshot));
  boat::InaStatusPayload power{};power.timestampUs=lastInaUs;power.ageUs=controlAge(lastInaUs,now);power.busVoltageV=latestIna.busV;power.shuntVoltageV=latestIna.shuntV;power.currentA=latestIna.currentRegA;power.powerW=latestIna.powerRegW;power.valid=latestIna.valid;power.errorCode=latestIna.valid?0:1;linkSend(boat::Type::InaStatus,reinterpret_cast<uint8_t*>(&power),sizeof(power));
  boat::VescTelemetryPayload motor{};motor.timestampUs=lastVescRxUs;motor.ageUs=controlAge(lastVescRxUs,now);motor.inputVoltageV=vescValues.inputVoltageV;motor.motorCurrentA=vescValues.motorCurrentA;motor.inputCurrentA=vescValues.inputCurrentA;motor.duty=vescValues.duty;motor.erpm=vescValues.erpm;motor.mosTempC=vescValues.mosTempC;motor.motorTempC=vescValues.motorTempC;motor.tachometer=vescValues.tachometer;motor.valid=vescValues.valid;motor.fault=vescValues.fault;linkSend(boat::Type::VescTelemetry,reinterpret_cast<uint8_t*>(&motor),sizeof(motor));
  boat::ActuatorStatePayload actuators{};actuators.timestampUs=now;actuators.pwmWrites=competitionPwmWrites;actuators.pwmErrors=competitionPwmErrors;actuators.leftPulseUs=competitionLeftPulseUs;actuators.rightPulseUs=competitionRightPulseUs;actuators.rearPulseUs=competitionRearPulseUs;actuators.motorRelayEnabled=motorRelayEnabled;actuators.enabledMask=competitionOutput.enabledMask;actuators.targetDuty=vescTargetDuty;actuators.appliedDuty=vescAppliedDuty;actuators.pcaReady=pcaReady;actuators.outputsEnabled=!competitionOutputsOff;actuators.safetyState=static_cast<uint8_t>(safety);actuators.controlMode=static_cast<uint8_t>(competitionController.mode());linkSend(boat::Type::ActuatorState,reinterpret_cast<uint8_t*>(&actuators),sizeof(actuators));
  boat::SystemHealthPayload health{};health.timestampUs=now;health.flags=(input.imuValid?1u:0u)|(input.tofValid?2u:0u)|(input.gnssValid?4u:0u)|(input.powerValid?8u:0u)|(input.vescValid?16u:0u)|(input.heartbeat?32u:0u);health.imuAgeUs=controlAge(input.imuUs,now);health.tofAgeUs=controlAge(input.tofUs,now);health.gnssAgeUs=controlAge(input.gnssUs,now);health.powerAgeUs=controlAge(input.powerUs,now);health.vescAgeUs=controlAge(input.vescUs,now);health.safetyState=static_cast<uint8_t>(safety);health.controlMode=static_cast<uint8_t>(competitionController.mode());health.stopReason=static_cast<uint8_t>(competitionOutput.reason);linkSend(boat::Type::SystemHealth,reinterpret_cast<uint8_t*>(&health),sizeof(health));
}

void runProductionControl(){
  if(!kCompetitionControlEnable)return;
  const uint64_t now=nowUs();const auto estimated=controlState.snapshot(now);
  production_control::SensorInput input{};input.nowUs=now;input.safety=competitionSafety(safety);
  input.heartbeat=lastHostHeartbeatMs&&millis()-lastHostHeartbeatMs<=kLinkFailSafeTimeoutMs;input.heartbeatUs=lastHostHeartbeatMs*1000ULL;
  input.imuValid=imu.ready()&&estimated.attitudeHealth!=static_cast<uint8_t>(boat::EstimateHealth::Invalid)&&estimated.gyroAgeUs<=kBnoGyroStaleUs;input.imuUs=estimated.estimateUs;
  input.tofValid=tofReady&&estimated.heightHealth!=static_cast<uint8_t>(boat::EstimateHealth::Invalid)&&isfinite(estimated.waterDistanceM);input.tofUs=lastTofUs;
  input.gnssValid=navigation.valid&&navOriginSet;input.gnssUs=navigation.timestampUs;
  const bool inaFresh=latestIna.valid&&lastInaUs&&now-lastInaUs<=kPowerStateStaleUs;
  const bool vescPowerValid=vescValues.valid&&lastVescRxUs&&now-lastVescRxUs<=kVescStateStaleUs&&isfinite(vescValues.inputVoltageV)&&vescValues.inputVoltageV>0&&isfinite(vescValues.inputCurrentA);
  input.powerValid=inaFresh||vescPowerValid;input.powerUs=inaFresh?lastInaUs:(vescPowerValid?lastVescRxUs:0);input.vescValid=vescValues.valid;input.vescUs=lastVescRxUs;input.vescFault=vescValues.fault!=0;
  input.rollRad=estimated.rollRad;input.pitchRad=estimated.pitchRad;input.yawRad=estimated.yawRad;input.rollRateRadS=estimated.rollRateRadS;input.pitchRateRadS=estimated.pitchRateRadS;input.yawRateRadS=estimated.yawRateRadS;input.tofM=estimated.waterDistanceM;input.northM=navNorthM;input.eastM=navEastM;input.groundSpeedMps=navigation.speedMps;input.courseRad=navigation.courseRad;input.busVoltageV=inaFresh?latestIna.busV:vescValues.inputVoltageV;input.currentA=inaFresh?latestIna.currentRegA:vescValues.inputCurrentA;input.powerW=inaFresh?latestIna.powerRegW:input.busVoltageV*input.currentA;input.vescErpm=vescValues.erpm;
  competitionOutput=competitionController.step(input);
  if(competitionOutput.reason!=production_control::StopReason::None)latchedCompetitionReason=competitionOutput.reason;
  if(competitionOutput.safetyRequest==production_control::SafetyRequest::Fault&&safety==SafetyState::RUNNING){safeOutputs();setState(SafetyState::FAULT);}
  else if(competitionOutput.safetyRequest==production_control::SafetyRequest::Disarm&&safety==SafetyState::RUNNING)stop();
  applyCompetitionActuators(competitionOutput,now);
  if(millis()-lastCompetitionOutputTxMs>=100){lastCompetitionOutputTxMs=millis();sendProductionTelemetry(estimated,input,now);}
}

void linkRxService(){
  uint16_t bytes=0;
  while(linkUart.available()&&bytes++<kLinkRxByteBudget){
    boat::Frame frame{};if(!linkDecoder.feed(static_cast<uint8_t>(linkUart.read()),frame))continue;++linkRxFrames;
    const boat::Type type=static_cast<boat::Type>(frame.header.type);
    if(type==boat::Type::Heartbeat)lastHostHeartbeatMs=millis();
    else if(type==boat::Type::GnssNav){++gnssNavRx;processNav(frame);}
    else if(type==boat::Type::GnssNavV2&&frame.header.length==sizeof(boat::GnssNavV2Payload)){
      boat::GnssNavV2Payload source{};memcpy(&source,frame.payload,sizeof(source));boat::GnssNavPayload nav{};nav.navSequence=source.navSequence;nav.fixSequence=source.fixSequence;nav.flags=source.flags;nav.utcCentiseconds=source.utcCentiseconds;nav.latitudeE7=source.latitudeE7;nav.longitudeE7=source.longitudeE7;nav.altitudeMm=source.altitudeMm;nav.speedMmPerSec=source.speedMmPerSec;nav.courseE5Deg=source.courseE5Deg;nav.hdopCenti=source.hdopCenti;nav.satellites=source.satellites;nav.fixType=source.fixType;nav.generatedUs=source.generatedUs;nav.canonicalCrc=boat::canonicalCrc(&nav,offsetof(boat::GnssNavPayload,canonicalCrc));boat::Frame legacy{};legacy.header.length=sizeof(nav);memcpy(legacy.payload,&nav,sizeof(nav));++gnssNavRx;processNav(legacy);
    }else if(type==boat::Type::WaypointSet)handleWaypointSet(frame);
    else if(type==boat::Type::ControlModeCommand||type==boat::Type::ManualCommand||type==boat::Type::HeadingTarget)handleCompetitionCommand(frame);
    else if(type==boat::Type::TimeSyncRequest&&frame.header.length==sizeof(boat::TimeSyncRequestPayload)){boat::TimeSyncRequestPayload request{};memcpy(&request,frame.payload,sizeof(request));boat::TimeSyncReplyPayload reply{request.sequence,request.t1Us,nowUs(),nowUs()};linkSend(boat::Type::TimeSyncReply,reinterpret_cast<uint8_t*>(&reply),sizeof(reply));}
    else if(type==boat::Type::Arm){const bool ok=safety==SafetyState::DISARMED&&productionPreflightReady();if(ok){latchedCompetitionReason=production_control::StopReason::None;setState(SafetyState::ARMED_IDLE);}sendCommandAck(frame,ok?0:1,ok?0:1);}
    else if(type==boat::Type::Disarm||type==boat::Type::Stop){stop();sendCommandAck(frame,0,0);}
    else if(type==boat::Type::StartTest){const bool ok=safety==SafetyState::ARMED_IDLE&&productionPreflightReady();if(ok){profile=TestProfile::SensorOnly;testStartMs=millis();setState(SafetyState::RUNNING);}sendCommandAck(frame,ok?0:1,ok?0:2);}
    else if(type==boat::Type::Estop){safeOutputs();profile=TestProfile::None;setState(SafetyState::E_STOP);sendCommandAck(frame,0,0);}
    else if(type==boat::Type::ClearEstop){const bool ok=safety==SafetyState::E_STOP;if(ok)setState(SafetyState::DISARMED);sendCommandAck(frame,ok?0:1,ok?0:3);}
    else if(type==boat::Type::P1Capture&&frame.header.length==sizeof(boat::P1CapturePayload)){boat::P1CapturePayload capture{};memcpy(&capture,frame.payload,sizeof(capture));if(static_cast<boat::P1CaptureAction>(capture.action)==boat::P1CaptureAction::Start)beginP1Capture(capture.captureId);else endP1Capture(capture.captureId);}
  }
}

void linkHealth(){if(safety==SafetyState::RUNNING&&lastHostHeartbeatMs&&millis()-lastHostHeartbeatMs>kLinkFailSafeTimeoutMs){latchedCompetitionReason=production_control::StopReason::Heartbeat;safeOutputs();setState(SafetyState::FAULT);}}

void control(){}
void start(TestProfile){if(safety==SafetyState::ARMED_IDLE&&productionPreflightReady())setState(SafetyState::RUNNING);}
void stop(){safeOutputs();profile=TestProfile::None;if(safety!=SafetyState::E_STOP)setState(SafetyState::DISARMED);}
void command(const char* commandText){
  if(!strcmp(commandText,"status"))Serial.printf("%s BNO=%d TOF=%d INA=%d PCA=%d VESC=%d WP=%u writes=%lu\n",stateName(safety),imu.ready(),tofReady,inaReady,pcaReady,vescValues.valid,competitionController.waypointCount(),static_cast<unsigned long>(competitionPwmWrites));
  else if(!strcmp(commandText,"scan"))scan();
  else if(!strcmp(commandText,"arm")&&safety==SafetyState::DISARMED)setState(SafetyState::ARMED_IDLE);
  else if(!strcmp(commandText,"start")&&productionPreflightReady())setState(SafetyState::RUNNING);
  else if(!strcmp(commandText,"stop")||!strcmp(commandText,"disarm"))stop();
  else if(!strcmp(commandText,"estop")){safeOutputs();setState(SafetyState::E_STOP);}
  else if(!strcmp(commandText,"clear_estop")&&safety==SafetyState::E_STOP)setState(SafetyState::DISARMED);
  else Serial.println("status scan arm start stop disarm estop clear_estop");
}
