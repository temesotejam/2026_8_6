#include <cassert>
#include <cstring>
#include <iostream>

#include "boat_protocol.h"
#include "command_ingress.h"
#include "production_control.h"

namespace {
boat::Frame commandFrame(boat::Type type,const void* payload,size_t size){
  boat::Frame frame{};frame.header.version=boat::kVersion;frame.header.type=static_cast<uint8_t>(type);
  frame.header.length=static_cast<uint16_t>(size);std::memcpy(frame.payload,payload,size);return frame;
}
}

int main(){
  boat::HeartbeatPayload heartbeat{1234,77,1,0,0};
  boat::Header header{boat::kVersion,static_cast<uint8_t>(boat::Type::Heartbeat),sizeof(heartbeat),9,42,123456,0};
  uint8_t encoded[boat::kMaxEncoded]{};const size_t bytes=boat::encode(header,reinterpret_cast<const uint8_t*>(&heartbeat),encoded,sizeof(encoded));
  assert(bytes>0&&encoded[bytes-1]==0);
  boat::Decoder decoder;boat::Frame decoded{};bool complete=false;
  for(size_t i=0;i<bytes;++i)complete=decoder.feed(encoded[i],decoded)||complete;
  assert(complete);assert(decoded.header.sequence==9);assert(decoded.header.type==static_cast<uint8_t>(boat::Type::Heartbeat));
  assert(decoded.header.length==sizeof(heartbeat));assert(std::memcmp(decoded.payload,&heartbeat,sizeof(heartbeat))==0);

  production_control::Controller controller;production_control::CommandIngress ingress;
  boat::ControlModeCommandPayload mode{};mode.protocolVersion=boat::kVersion;mode.mode=static_cast<uint8_t>(production_control::ControlMode::HeadingHold);mode.requestId=10;mode.commandSequence=1;mode.sourceUs=1000;mode.canonicalCrc=boat::canonicalCrc(&mode,offsetof(boat::ControlModeCommandPayload,canonicalCrc));
  const boat::Frame modeFrame=commandFrame(boat::Type::ControlModeCommand,&mode,sizeof(mode));
  auto accepted=ingress.process(modeFrame,production_control::AuthoritativeSafety::Disarmed,controller,2000);
  assert(accepted.ackGenerated&&accepted.result.ack==production_control::Ack::Accepted);
  auto duplicate=ingress.process(modeFrame,production_control::AuthoritativeSafety::Disarmed,controller,2100);
  assert(duplicate.ackGenerated&&duplicate.result.ack==production_control::Ack::Duplicate);

  boat::ManualCommandPayload manual{};manual.protocolVersion=boat::kVersion;manual.reserved[0]=boat::ManualLeft;manual.requestId=11;manual.commandSequence=2;manual.sourceUs=2200;manual.leftFrontWing=.25f;manual.propulsion=.25f;manual.canonicalCrc=boat::canonicalCrc(&manual,offsetof(boat::ManualCommandPayload,canonicalCrc));
  auto manualAccepted=ingress.process(commandFrame(boat::Type::ManualCommand,&manual,sizeof(manual)),production_control::AuthoritativeSafety::Disarmed,controller,2300);
  assert(manualAccepted.ackGenerated&&manualAccepted.result.ack==production_control::Ack::Accepted);
  assert(controller.manualOutputMask()==production_control::ManualLeft);

  boat::ManualCommandPayload fullManual{};fullManual.protocolVersion=boat::kVersion;fullManual.reserved[0]=boat::ManualAll;fullManual.requestId=12;fullManual.commandSequence=3;fullManual.sourceUs=2350;fullManual.leftFrontWing=.25f;fullManual.rightFrontWing=-.2f;fullManual.rearYaw=.1f;fullManual.propulsion=.5f;fullManual.canonicalCrc=boat::canonicalCrc(&fullManual,offsetof(boat::ManualCommandPayload,canonicalCrc));
  auto fullManualAccepted=ingress.process(commandFrame(boat::Type::ManualCommand,&fullManual,sizeof(fullManual)),production_control::AuthoritativeSafety::Disarmed,controller,2375);
  assert(fullManualAccepted.ackGenerated&&fullManualAccepted.result.ack==production_control::Ack::Accepted);
  assert(controller.manualOutputMask()==production_control::ManualAll);

  boat::ControlModeCommandPayload waypointOnly{};waypointOnly.protocolVersion=boat::kVersion;waypointOnly.mode=boat::ControlWaypointOnly;waypointOnly.requestId=13;waypointOnly.commandSequence=4;waypointOnly.sourceUs=2380;waypointOnly.canonicalCrc=boat::canonicalCrc(&waypointOnly,offsetof(boat::ControlModeCommandPayload,canonicalCrc));
  auto waypointAccepted=ingress.process(commandFrame(boat::Type::ControlModeCommand,&waypointOnly,sizeof(waypointOnly)),production_control::AuthoritativeSafety::Disarmed,controller,2390);
  assert(waypointAccepted.ackGenerated&&waypointAccepted.result.ack==production_control::Ack::Accepted);
  assert(controller.mode()==production_control::ControlMode::WaypointOnly);

  manual.canonicalCrc^=1;
  auto rejected=ingress.process(commandFrame(boat::Type::ManualCommand,&manual,sizeof(manual)),production_control::AuthoritativeSafety::Disarmed,controller,2400);
  assert(rejected.ackGenerated&&rejected.result.ack==production_control::Ack::Rejected);
  std::cout<<"protocol tests passed\n";
}
