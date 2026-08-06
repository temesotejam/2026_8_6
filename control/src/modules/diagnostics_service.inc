// Runtime diagnostics and Arduino loop orchestration.
void printTxDiag() {
  static uint64_t previousBytes = 0, previousFrames = 0;
  static uint32_t previousMs = 0, previousGeneration = UINT32_MAX;
  const uint32_t now = millis();
  uint8_t queueUsed = 0;
  uint32_t queueHigh = 0, drops = 0;
  portENTER_CRITICAL(&linkMux);
  queueUsed = linkUsed;
  queueHigh = linkQueueHighWater;
  drops = linkDrops;
  portEXIT_CRITICAL(&linkMux);

  BnoTxDiag bnoTx[6]{};
  uint64_t frames = 0, bytes = 0, writes = 0, partials = 0, zeros = 0;
  uint32_t maxWriteUs = 0, minHeap = UINT32_MAX, generation = 0;
  portENTER_CRITICAL(&bnoMetricsMux);
  for (uint8_t kind = 0; kind < 6; ++kind) bnoTx[kind] = bnoTxDiag[kind];
  frames = txFramesCompleted;
  bytes = txBytesSent;
  writes = txWriteCalls;
  partials = txPartialWrites;
  zeros = txZeroWrites;
  maxWriteUs = txMaxWriteUs;
  minHeap = txMinFreeHeap;
  generation = txDiagResetGeneration;
  const uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < txMinFreeHeap) txMinFreeHeap = freeHeap;
  portEXIT_CRITICAL(&bnoMetricsMux);

  if (generation != previousGeneration) {
    previousBytes = previousFrames = 0;
    previousMs = now;
    previousGeneration = generation;
  }
  const uint32_t dt = max<uint32_t>(1, now - previousMs);
  const uint64_t bytesRate =
      (bytes >= previousBytes ? bytes - previousBytes : bytes) * 1000ULL / dt;
  const uint64_t framesRate =
      (frames >= previousFrames ? frames - previousFrames : frames) * 1000ULL /
      dt;
  previousBytes = bytes;
  previousFrames = frames;
  previousMs = now;

  uint32_t bnoDrops = 0;
  for (uint8_t kind = 1; kind <= 5; ++kind)
    bnoDrops += static_cast<uint32_t>(bnoTx[kind].queueDrops);
  Serial.printf(
      "TXDIAG frames_s=%llu bytes_s=%llu total_frames=%llu total_bytes=%llu "
      "queue=%u high=%lu drops=%lu bno_drop=%lu partial=%llu zero=%llu "
      "write_calls=%llu max_write_us=%lu free_heap=%lu min_free_heap=%lu "
      "usage_pct=%.2f\n",
      static_cast<unsigned long long>(framesRate),
      static_cast<unsigned long long>(bytesRate),
      static_cast<unsigned long long>(frames),
      static_cast<unsigned long long>(bytes), queueUsed,
      static_cast<unsigned long>(queueHigh), static_cast<unsigned long>(drops),
      static_cast<unsigned long>(bnoDrops),
      static_cast<unsigned long long>(partials),
      static_cast<unsigned long long>(zeros),
      static_cast<unsigned long long>(writes),
      static_cast<unsigned long>(maxWriteUs),
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(minHeap),
      static_cast<double>(bytesRate) * 10.0 / 921600.0 * 100.0);

  const bno::PipelineMetrics metrics = imu.metrics();
  for (uint8_t kind = 1; kind <= 5; ++kind) {
    const bno::ReportMetrics& report = metrics.report[kind];
    const double effectiveHz =
        report.events > 1 && report.lastRxUs > report.firstRxUs
            ? 1000000.0 * static_cast<double>(report.events - 1) /
                  static_cast<double>(report.lastRxUs - report.firstRxUs)
            : 0.0;
    Serial.printf(
        "BNO_REPORT kind=%u report_id=%u interval_us=%lu requested_hz=%.3f "
        "events=%lu tx_done=%llu tx_drop=%llu event_drop=%lu "
        "effective_hz=%.3f\n",
        kind, report.reportId, static_cast<unsigned long>(report.intervalUs),
        report.intervalUs ? 1000000.0 / report.intervalUs : 0.0,
        static_cast<unsigned long>(report.events),
        static_cast<unsigned long long>(bnoTx[kind].completed),
        static_cast<unsigned long long>(bnoTx[kind].queueDrops),
        static_cast<unsigned long>(report.eventQueueDrops), effectiveHz);
  }
}

void printRuntimeDiagnostics() {
  uint8_t txUsed = 0;
  uint32_t txDrops = 0, txHigh = 0;
  portENTER_CRITICAL(&linkMux);
  txUsed = linkUsed;
  txDrops = linkDrops;
  txHigh = linkQueueHighWater;
  portEXIT_CRITICAL(&linkMux);

  const uint64_t now = nowUs();
  const bno::PipelineMetrics bnoMetrics = imu.metrics();
  const boat::EstimatedStatePayload state = controlState.snapshot(now);
  Serial.printf(
      "DIAG state=%s dry=%d bno_ready=%d addr=0x%02X "
      "event A/G/R/M=%lu/%lu/%lu/%lu age_us attitude/gyro=%lu/%lu "
      "rpy_rad=%.5f/%.5f/%.5f rates=%.5f/%.5f/%.5f "
      "health attitude/yaw/nav/height=%u/%u/%u/%u\n",
      stateName(safety), kDryRunActuators, imu.ready(), imu.address(),
      static_cast<unsigned long>(bnoMetrics.accelEvents),
      static_cast<unsigned long>(bnoMetrics.gyroEvents),
      static_cast<unsigned long>(bnoMetrics.rotationEvents),
      static_cast<unsigned long>(bnoMetrics.magneticEvents),
      static_cast<unsigned long>(
          state.estimateUs && now >= state.estimateUs
              ? min<uint64_t>(now - state.estimateUs, UINT32_MAX)
              : UINT32_MAX),
      static_cast<unsigned long>(state.gyroAgeUs), state.rollRad,
      state.pitchRad, state.yawRad, state.rollRateRadS, state.pitchRateRadS,
      state.yawRateRadS, state.attitudeHealth, state.yawHealth,
      state.navigationHealth, state.heightHealth);
  Serial.printf("LINK queue/drop/high=%u/%lu/%lu tof=%lu\n", txUsed,
                static_cast<unsigned long>(txDrops),
                static_cast<unsigned long>(txHigh),
                static_cast<unsigned long>(tofFrames));
  printTxDiag();
  printCompetitionCommandDiag();
  Serial.printf(
      "ACTUATOR pwm_writes=%lu pwm_errors=%lu vesc_target=%.4f "
      "vesc_applied=%.4f relay=%u vesc_duty=%.4f erpm=%.1f fault=%u state=%s\n",
      static_cast<unsigned long>(competitionPwmWrites),
      static_cast<unsigned long>(competitionPwmErrors), vescTargetDuty,
      vescAppliedDuty, motorRelayEnabled?1:0, vescValues.duty, vescValues.erpm, vescValues.fault,
      stateName(safety));
}

void loop() {
  if (kEnableTemporaryDebugWifi) web.handleClient();
  serialConsole();
  const uint64_t loopStart = nowUs();
  uint64_t taskStart = loopStart;
  linkRxService();
  if (bench.active) {
    const uint32_t elapsed = static_cast<uint32_t>(nowUs() - taskStart);
    if (elapsed > bench.maxLinkRxUs) bench.maxLinkRxUs = elapsed;
  }
  linkHealth();
  pollTof();
  pollIna();

  runProductionControl();
  taskStart = nowUs();
  pollVesc();
  if (bench.active) {
    const uint32_t elapsed = static_cast<uint32_t>(nowUs() - taskStart);
    if (elapsed > bench.maxVescUs) bench.maxVescUs = elapsed;
  }
  control();
  if (bench.active) {
    const uint32_t elapsed = static_cast<uint32_t>(nowUs() - loopStart);
    if (elapsed > bench.maxLoopUs) bench.maxLoopUs = elapsed;
    if (ESP.getFreeHeap() < bench.minFreeHeap)
      bench.minFreeHeap = ESP.getFreeHeap();
  }
  if (millis() - lastDiagMs >= kDiagnosticIntervalMs) {
    lastDiagMs = millis();
    printRuntimeDiagnostics();
  }
  delay(1);
}
