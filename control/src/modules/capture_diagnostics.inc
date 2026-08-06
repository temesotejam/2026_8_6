// P1 capture and transport diagnostics.
void printBnoCaptureReport() {
  const bno::PipelineMetrics pipeline = imu.metrics();
  BnoTxDiag tx[6]{};
  portENTER_CRITICAL(&bnoMetricsMux);
  for (uint8_t kind = 1; kind <= 5; ++kind) tx[kind] = bnoTxDiag[kind];
  portEXIT_CRITICAL(&bnoMetricsMux);
  for (uint8_t kind = 1; kind <= 5; ++kind) {
    const bno::ReportMetrics& report = pipeline.report[kind];
    const double hz =
        report.events > 1 && report.lastRxUs > report.firstRxUs
            ? 1000000.0 * static_cast<double>(report.events - 1) /
                  static_cast<double>(report.lastRxUs - report.firstRxUs)
            : 0.0;
    Serial.printf(
        "BNO_CAPTURE_REPORT kind=%u report_id=%u interval_us=%lu "
        "requested_hz=%.3f enable_ok=%lu enable_fail=%lu callback=%lu "
        "switch=%lu kind_convert=%lu events=%lu tx_event=%llu tx_req=%llu "
        "tx_enq=%llu tx_done=%llu tx_drop=%llu event_drop=%lu "
        "reset_reconfig=%lu effective_hz=%.3f\n",
        kind, report.reportId, static_cast<unsigned long>(report.intervalUs),
        report.intervalUs ? 1000000.0 / report.intervalUs : 0.0,
        static_cast<unsigned long>(report.enableSuccess),
        static_cast<unsigned long>(report.enableFailure),
        static_cast<unsigned long>(report.callbacks),
        static_cast<unsigned long>(report.switchHits),
        static_cast<unsigned long>(report.kindConversions),
        static_cast<unsigned long>(report.events),
        static_cast<unsigned long long>(tx[kind].events),
        static_cast<unsigned long long>(tx[kind].sendRequests),
        static_cast<unsigned long long>(tx[kind].queueEnqueues),
        static_cast<unsigned long long>(tx[kind].completed),
        static_cast<unsigned long long>(tx[kind].queueDrops),
        static_cast<unsigned long>(report.eventQueueDrops),
        static_cast<unsigned long>(report.resetReconfig), hz);
  }
}

void printP1TxSnapshot() {
  uint32_t first = 0, last = 0;
  bool hasSequence = false;
  portENTER_CRITICAL(&bnoMetricsMux);
  first = txTrialFirstSequence;
  last = txTrialLastSequence;
  hasSequence = txTrialHasSequence;
  portEXIT_CRITICAL(&bnoMetricsMux);
  Serial.printf("P1_CAPTURE_SEQUENCE first=%lu last=%lu valid=%d\n",
                static_cast<unsigned long>(first),
                static_cast<unsigned long>(last), hasSequence ? 1 : 0);
  Serial.printf("TX_CAPTURE_SUPPRESSED count=%lu\n",
                static_cast<unsigned long>(txCaptureSuppressed));
  Serial.println("P1_CAPTURE_STATS_STOP");
  printBnoCaptureReport();
  printP1BnoTimestampStats();
  for (uint16_t type = 0; type < 256; ++type) {
    TypeTxDiag diagnostic{};
    portENTER_CRITICAL(&bnoMetricsMux);
    diagnostic = txTypeDiag[type];
    portEXIT_CRITICAL(&bnoMetricsMux);
    if (diagnostic.requested || diagnostic.enqueued || diagnostic.completed ||
        diagnostic.encodedBytes) {
      Serial.printf(
          "TXTYPE type=%u name=%s req=%llu enq=%llu done=%llu bytes=%llu\n",
          type, protocolTypeName(static_cast<uint8_t>(type)),
          static_cast<unsigned long long>(diagnostic.requested),
          static_cast<unsigned long long>(diagnostic.enqueued),
          static_cast<unsigned long long>(diagnostic.completed),
          static_cast<unsigned long long>(diagnostic.encodedBytes));
    }
  }
  txHistoryPrint();
}

void waitLinkSendsIdle() {
  for (;;) {
    portENTER_CRITICAL(&linkMux);
    const uint32_t inFlight = linkSendsInFlight;
    portEXIT_CRITICAL(&linkMux);
    if (!inFlight) return;
    delay(1);
  }
}

void beginP1Capture(uint32_t captureId) {
  portENTER_CRITICAL(&linkMux);
  p1StartGate = true;
  portEXIT_CRITICAL(&linkMux);
  waitLinkSendsIdle();
  p1CaptureActive = false;
  resetBnoTxDiag();
  resetP1BnoTimestampStats();
  imu.resetRunStats();
  imu.requestFeatureSnapshot();
  const uint32_t ackSequence = reserveLinkSequence();
  boat::P1CaptureAckPayload ack{captureId,
                                static_cast<uint8_t>(boat::P1CaptureAction::Start),
                                1, 0, ackSequence + 1, 0};
  const bool queued =
      linkSend(boat::Type::P1Capture, reinterpret_cast<uint8_t*>(&ack),
               sizeof(ack), 0, true, ackSequence);
  portENTER_CRITICAL(&bnoMetricsMux);
  p1BnoTimestampStatsActive = queued;
  portEXIT_CRITICAL(&bnoMetricsMux);
  portENTER_CRITICAL(&linkMux);
  if (queued) p1CaptureActive = true;
  p1StartGate = false;
  portEXIT_CRITICAL(&linkMux);
  Serial.printf("P1_START_ACK sequence=%lu first_sequence=%lu queued=%d\n",
                static_cast<unsigned long>(ackSequence),
                static_cast<unsigned long>(ack.firstSequence), queued ? 1 : 0);
  Serial.println("P1_CAPTURE_STATS_RESET");
  Serial.println("TX_DIAG_RESET");
}

void endP1Capture(uint32_t captureId) {
  portENTER_CRITICAL(&linkMux);
  p1StopGate = true;
  portEXIT_CRITICAL(&linkMux);
  portENTER_CRITICAL(&bnoMetricsMux);
  p1BnoTimestampStatsActive = false;
  portEXIT_CRITICAL(&bnoMetricsMux);
  waitLinkSendsIdle();
  uint32_t lastSequence = 0;
  portENTER_CRITICAL(&bnoMetricsMux);
  lastSequence = txTrialLastSequence;
  p1CaptureActive = false;
  portEXIT_CRITICAL(&bnoMetricsMux);
  const uint32_t ackSequence = reserveLinkSequence();
  boat::P1CaptureAckPayload ack{captureId,
                                static_cast<uint8_t>(boat::P1CaptureAction::Stop),
                                1, 0, 0, lastSequence};
  const bool queued =
      linkSend(boat::Type::P1Capture, reinterpret_cast<uint8_t*>(&ack),
               sizeof(ack), 0, true, ackSequence);
  portENTER_CRITICAL(&linkMux);
  p1StopGate = false;
  portEXIT_CRITICAL(&linkMux);
  Serial.printf("P1_STOP_ACK sequence=%lu last_sequence=%lu queued=%d\n",
                static_cast<unsigned long>(ackSequence),
                static_cast<unsigned long>(ack.lastSequence), queued ? 1 : 0);
  delay(100);
  printP1TxSnapshot();
}

struct WaypointAckTransport {};
