// GNSS input processing and result acknowledgement.
void stop(); void printP1TxSnapshot();
void sendCommandAck(const boat::Frame&f,uint8_t disposition,uint16_t reason){boat::CommandPayload c{};if(f.header.length==sizeof(c))memcpy(&c,f.payload,sizeof(c));boat::CommandAckPayload ack{c.commandId,c.commandType,disposition,(uint8_t)safety,(uint8_t)kDryRunActuators,nowUs(),nowUs(),reason,0};linkSend(boat::Type::CommandAck,(uint8_t*)&ack,sizeof(ack));}
void processNav(const boat::Frame& f) {
  boat::GnssNavPayload nav{};
  if (f.header.length != sizeof(nav)) { ++navCrcErrors; return; }
  memcpy(&nav, f.payload, sizeof(nav));
  navigation.latitudeDeg=nav.latitudeE7/1e7; navigation.longitudeDeg=nav.longitudeE7/1e7; navigation.speedMps=nav.speedMmPerSec/1000.0f; navigation.courseRad=(nav.courseE5Deg/100000.0f)*PI/180.0f; navigation.timestampUs=nowUs();
  uint32_t flags = 0;
  if (nav.canonicalCrc != boat::canonicalCrc(&nav, offsetof(boat::GnssNavPayload, canonicalCrc))) { ++navCrcErrors; flags |= 8; }
  const bool navValid = (nav.flags & boat::NavFixValid) && !(flags & 8);
  navigation.valid=navValid;
  controlState.updateGnss(nowUs(), nav.latitudeE7 / 1e7,
                          nav.longitudeE7 / 1e7,
                          nav.speedMmPerSec / 1000.0f,
                          (nav.courseE5Deg / 100000.0f) * PI / 180.0f,
                          navValid);
  const bool duplicate = lastNavSequence && nav.navSequence == lastNavSequence;
  if (duplicate) flags |= 2;
  if (lastNavSequence && nav.navSequence != lastNavSequence + 1 && !duplicate) { flags |= 4; ++navSequenceGaps; }
  lastNavSequence = nav.navSequence;
  if (navValid && !navOriginSet) { navOriginSet = true; navOriginLatE7 = nav.latitudeE7; navOriginLonE7 = nav.longitudeE7; syncCompetitionWaypoints(); }
  boat::GnssProcessResultPayload result{};
  result.navSequence = nav.navSequence; result.fixSequence = nav.fixSequence; result.navCanonicalCrc = nav.canonicalCrc; result.flags = flags; result.controlReceivedUs = nowUs();
  result.originLatitudeE7 = navOriginLatE7; result.originLongitudeE7 = navOriginLonE7; result.speedMmPerSec = nav.speedMmPerSec;
  result.courseMilliRad = (int32_t)lround((nav.courseE5Deg / 100000.0) * PI / 180.0 * 1000.0);
  if (navOriginSet) { result.flags |= 16; result.northMm = (int32_t)lround((nav.latitudeE7 - navOriginLatE7) * 11.132); result.eastMm = (int32_t)lround((nav.longitudeE7 - navOriginLonE7) * 11.132 * cos(navOriginLatE7 / 1e7 * PI / 180.0)); navNorthM=result.northMm*.001f;navEastM=result.eastMm*.001f; }
  if (navValid) result.flags |= 1;
  result.errorCode = (flags & 8) ? 1 : 0; result.safetyState = (uint16_t)safety; result.dryRun = kDryRunActuators; result.controlSentUs = nowUs();
  result.canonicalCrc = boat::canonicalCrc(&result, offsetof(boat::GnssProcessResultPayload, canonicalCrc));
  linkSend(boat::Type::GnssProcessResult, (uint8_t*)&result, sizeof(result)); ++gnssResultTx;
}
