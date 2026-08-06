// BNO08X, ToF, INA226, and VESC acquisition.
void setState(SafetyState next) {
  if (next == safety) return;
  safety = next;
  Serial.printf("STATE %s\n", stateName(safety));
}

void printTimestampStats(const char* name, const bno::TimestampStats& t) {
  const double d = t.events > 1 ? static_cast<double>(t.events - 1) : 1.0;
  Serial.printf(
      "BNO_TS %s events=%lu unique=%lu exact=%d overflow=%d dup=%lu "
      "reverse=%lu rollover=%lu sensor_dt_us=%lld/%.2f/%lld "
      "rx_dt_us=%llu/%.2f/%llu pair_first=%llu/%llu "
      "pair_last=%llu/%llu\n",
      name, static_cast<unsigned long>(t.events),
      static_cast<unsigned long>(t.uniqueCount), t.uniqueExact,
      t.uniqueOverflow, static_cast<unsigned long>(t.consecutiveDuplicates),
      static_cast<unsigned long>(t.reversals),
      static_cast<unsigned long>(t.rollovers),
      static_cast<long long>(t.sensorDeltaMin), t.sensorDeltaSum / d,
      static_cast<long long>(t.sensorDeltaMax),
      static_cast<unsigned long long>(t.rxDeltaMin), t.rxDeltaSum / d,
      static_cast<unsigned long long>(t.rxDeltaMax),
      static_cast<unsigned long long>(t.firstSensorUs),
      static_cast<unsigned long long>(t.firstRxUs),
      static_cast<unsigned long long>(t.lastSensorUs),
      static_cast<unsigned long long>(t.lastRxUs));
}

void printVectorStats(const char* name, const bno::VectorStats& v) {
  if (!v.events) {
    Serial.printf("BNO_VEC %s events=0 nonfinite=%lu\n", name,
                  static_cast<unsigned long>(v.nonFinite));
    return;
  }
  const double n = v.events;
  const double mx = v.sum[0] / n, my = v.sum[1] / n, mz = v.sum[2] / n;
  const double mn = v.normSum / n;
  const double sx = sqrt(fmax(0.0, v.sumSquares[0] / n - mx * mx));
  const double sy = sqrt(fmax(0.0, v.sumSquares[1] / n - my * my));
  const double sz = sqrt(fmax(0.0, v.sumSquares[2] / n - mz * mz));
  const double sn = sqrt(fmax(0.0, v.normSquares / n - mn * mn));
  Serial.printf(
      "BNO_VEC %s events=%lu nonfinite=%lu "
      "xyz_mean_sd=[%.6f,%.6f][%.6f,%.6f][%.6f,%.6f] "
      "xyz_minmax=[%.6f,%.6f][%.6f,%.6f][%.6f,%.6f] "
      "norm_mean_sd_minmax=%.6f/%.6f/%.6f/%.6f\n",
      name, static_cast<unsigned long>(v.events),
      static_cast<unsigned long>(v.nonFinite), mx, sx, my, sy, mz, sz,
      v.minimum[0], v.maximum[0], v.minimum[1], v.maximum[1], v.minimum[2],
      v.maximum[2], mn, sn, v.normMin, v.normMax);
}

void bnoEvent(const bno::Sample& sample) {
  if (sample.type == bno::EventType::Accel) {
    controlState.noteAccel(sample.rxUs);
    primaryImu.accelUs = sample.sensorUs;
    primaryImu.accelRxUs = sample.rxUs;
    primaryImu.accelAccuracy = sample.accuracy;
    memcpy(primaryImu.accel, sample.v, sizeof(primaryImu.accel));
  } else if (sample.type == bno::EventType::Gyro) {
    controlState.ingestGyro(sample.sensorUs, sample.rxUs, sample.v[0],
                            sample.v[1], sample.v[2]);
    primaryImu.gyroUs = sample.sensorUs;
    primaryImu.gyroRxUs = sample.rxUs;
    primaryImu.gyroAccuracy = sample.accuracy;
    memcpy(primaryImu.gyro, sample.v, sizeof(primaryImu.gyro));
  } else if (sample.type == bno::EventType::Rotation) {
    controlState.ingestRotation(sample.sensorUs, sample.rxUs, sample.accuracy,
                                sample.v[0], sample.v[1], sample.v[2],
                                sample.v[3]);
  } else if (sample.type == bno::EventType::Magnetic) {
    controlState.noteMagnetic(sample.rxUs);
    primaryImu.magneticUs = sample.sensorUs;
    primaryImu.magneticRxUs = sample.rxUs;
    primaryImu.magneticAccuracy = sample.accuracy;
    memcpy(primaryImu.magnetic, sample.v, sizeof(primaryImu.magnetic));
  }

  if (!bnoBenchmarkActive && !p1CaptureActive) return;
  const uint8_t kind = static_cast<uint8_t>(sample.type);
  if (kind >= 1 && kind <= 5) {
    recordP1BnoTimestamp(kind, sample.sensorUs, sample.rxUs);
    portENTER_CRITICAL(&bnoMetricsMux);
    ++bnoTxDiag[kind].events;
    portEXIT_CRITICAL(&bnoMetricsMux);
  }
  boat::BnoPayload payload{};
  payload.kind = kind;
  payload.accuracy = sample.accuracy;
  payload.sequence = sample.sequence;
  payload.sensorUs = sample.sensorUs;
  payload.callbackUs = sample.callbackUs;
  payload.queuePushUs = sample.queuePushUs;
  memcpy(payload.v, sample.v, sizeof(payload.v));
  const boat::Type type =
      sample.type == bno::EventType::Accel
          ? boat::Type::BnoAccel
          : sample.type == bno::EventType::Gyro
                ? boat::Type::BnoGyro
                : sample.type == bno::EventType::Magnetic
                      ? boat::Type::BnoMagnetic
                      : boat::Type::BnoQuaternion;
  const bool queued =
      linkSend(type, reinterpret_cast<uint8_t*>(&payload), sizeof(payload));
  portENTER_CRITICAL(&bnoMetricsMux);
  if (queued)
    ++bnoLinkFrames;
  else
    ++bnoLinkFrameDrops;
  portEXIT_CRITICAL(&bnoMetricsMux);
}

void sendControlState() {
  const uint32_t now = millis();
  if (now - lastEstimatedStateTxMs < kEstimatedStateTxIntervalMs) return;
  lastEstimatedStateTxMs = now;
  const boat::EstimatedStatePayload state = controlState.snapshot(nowUs());
  linkSend(boat::Type::EstimatedState,
           reinterpret_cast<const uint8_t*>(&state), sizeof(state));
}

void sendPrimaryImuSnapshot() {
  const uint32_t now = millis();
  if (now - lastPrimaryImuSnapshotTxMs < kPrimaryImuSnapshotTxIntervalMs ||
      !primaryImu.accelUs || !primaryImu.gyroUs || !primaryImu.magneticUs)
    return;
  lastPrimaryImuSnapshotTxMs = now;
  boat::PrimaryImuSnapshotPayload payload{};
  payload.accelSensorUs = primaryImu.accelUs;
  payload.gyroSensorUs = primaryImu.gyroUs;
  payload.magneticSensorUs = primaryImu.magneticUs;
  payload.accelAccuracy = primaryImu.accelAccuracy;
  payload.gyroAccuracy = primaryImu.gyroAccuracy;
  payload.magneticAccuracy = primaryImu.magneticAccuracy;
  memcpy(payload.accel, primaryImu.accel, sizeof(payload.accel));
  memcpy(payload.gyro, primaryImu.gyro, sizeof(payload.gyro));
  memcpy(payload.magnetic, primaryImu.magnetic, sizeof(payload.magnetic));
  linkSend(boat::Type::PrimaryImuSnapshot,
           reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
}

void IRAM_ATTR bnoIntIsr() {
  BaseType_t woke = pdFALSE;
  ++bnoIntEdges;
  if (bnoTaskHandle) vTaskNotifyGiveFromISR(bnoTaskHandle, &woke);
  if (woke) portYIELD_FROM_ISR();
}

void bnoTask(void*) {
  for (;;) {
    const bool intLow = digitalRead(kBnoIntPin) == LOW;
    const uint32_t notified =
        intLow ? 0
               : ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBnoTaskFallbackMs));
    const bool service = intLow || notified;
    if (!service) ++bnoTaskTimeouts;
    uint32_t pollUs = 0;
    uint64_t started = nowUs();
    if (service) {
      imu.poll(bnoEvent);
      pollUs = static_cast<uint32_t>(nowUs() - started);
    }
    sendControlState();
    sendPrimaryImuSnapshot();
    started = nowUs();
    imu.recover();
    const uint32_t recoverUs = static_cast<uint32_t>(nowUs() - started);
    if (bnoBenchmarkActive) {
      portENTER_CRITICAL(&bnoMetricsMux);
      if (pollUs > bnoTaskMaxPollUs) bnoTaskMaxPollUs = pollUs;
      if (recoverUs > bnoTaskMaxRecoverUs) bnoTaskMaxRecoverUs = recoverUs;
      portEXIT_CRITICAL(&bnoMetricsMux);
    }
    if (service) vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void scan() {
  char addresses[96] = "";
  size_t used = 0;
  for (uint8_t address = 8; address < 120; ++address) {
    Wire.beginTransmission(address);
    if (!Wire.endTransmission())
      used += snprintf(addresses + used, sizeof(addresses) - used, "%s0x%02X",
                       used ? "," : "", address);
  }
  Serial.printf("PERIPH_SCAN %s\n", used ? addresses : "none");
}

void pollTof() {
  if (!tofReady) return;
  const uint64_t serviceStart = nowUs();
  const bool dataReady = tof.isDataReady();
  const uint32_t readyElapsed =
      static_cast<uint32_t>(nowUs() - serviceStart);
  if (bench.active && readyElapsed > bench.maxTofReadyUs)
    bench.maxTofReadyUs = readyElapsed;
  if (!dataReady) return;

  const uint64_t readStart = nowUs();
  if (!tof.getRangingData(&tofData)) {
    if (bench.active) ++bench.tofIncomplete;
    return;
  }
  const uint32_t readElapsed =
      static_cast<uint32_t>(nowUs() - readStart);
  const uint32_t serviceElapsed =
      static_cast<uint32_t>(nowUs() - serviceStart);
  if (bench.active) {
    ++bench.tofFrames;
    if (readElapsed > bench.maxTofReadUs) bench.maxTofReadUs = readElapsed;
    if (serviceElapsed > bench.maxTofServiceUs)
      bench.maxTofServiceUs = serviceElapsed;
  }

  ++tofFrames;
  lastTofUs = readStart;
  uint8_t validZones = 0;
  float sum = 0.0f, sumSq = 0.0f;
  for (uint8_t zone = 0; zone < tofResolutionActual && zone < 64; ++zone) {
    const uint16_t mm = tofData.distance_mm[zone];
    const uint8_t status = tofData.target_status[zone];
    if (mm >= 200 && mm <= 4000 && (status == 5 || status == 9)) {
      const float meters = mm / 1000.0f;
      sum += meters;
      sumSq += meters * meters;
      ++validZones;
    }
  }
  const float mean = validZones ? sum / validZones : NAN;
  const float spread =
      validZones > 1
          ? sqrtf(fmaxf(0.0f, sumSq / validZones - mean * mean))
          : NAN;
  const boat::EstimatedStatePayload state = controlState.snapshot(readStart);
  const bool attitudeValid =
      state.attitudeHealth != static_cast<uint8_t>(boat::EstimateHealth::Invalid) &&
      isfinite(state.rollRad) && isfinite(state.pitchRad);
  const bool validDistance = validZones >= 4 && isfinite(mean) &&
                             isfinite(spread) && spread <= kTofMaxSpreadM &&
                             attitudeValid;
  const float corrected =
      validDistance ? mean * cosf(state.rollRad) * cosf(state.pitchRad) : NAN;
  controlState.updateWaterDistance(readStart, corrected, validDistance);

  uint8_t payload[196];
  memcpy(payload, &tofFrames, 4);
  memcpy(payload + 4, tofData.distance_mm, 128);
  memcpy(payload + 132, tofData.target_status, 64);
  linkSend(boat::Type::TofFrame, payload, sizeof(payload));
}

void pollIna() {
  const uint64_t now = nowUs();
  if (kEnableIna226 &&
      (!lastInaPollUs || now - lastInaPollUs >= kInaSampleUs)) {
    lastInaPollUs = now;
    InaSample sample{};
    if (inaReady && ina.read(sample) && sample.valid) {
      latestIna = sample;
      lastInaUs = sample.us;
      struct __attribute__((packed)) {
        int16_t shunt, current;
        uint16_t bus, power, mask;
        uint32_t i2cUs;
      } payload{sample.rawShunt, sample.rawCurrent, sample.rawBus,
                sample.rawPower, sample.mask, sample.i2cUs};
      linkSend(boat::Type::InaSample, reinterpret_cast<uint8_t*>(&payload),
               sizeof(payload));
    }
  }
}

void pollVesc() {
  while (vescUart.available()) {
    vesc::Frame frame{};
    vesc::ParseError error;
    if (vescParser.feed(static_cast<uint8_t>(vescUart.read()), nowUs(), frame,
                        error)) {
      if (frame.crcOk && frame.endOk &&
          vesc::Parser::parseCurrentValues(frame, vescValues)) {
        lastVescRxUs = nowUs();
        ++vescResp;
        linkSend(boat::Type::VescStatus,
                 reinterpret_cast<uint8_t*>(&vescValues), sizeof(vescValues));
      } else {
        ++vescErrors;
      }
    }
  }
  if (millis() - lastVescReqMs >= kVescRequestIntervalMs) {
    lastVescReqMs = millis();
    uint8_t request = 4;
    vescSend(&request, 1);
    ++vescReq;
  }
}
