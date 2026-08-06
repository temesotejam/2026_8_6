#pragma once
#include <Arduino.h>
namespace app_config {
constexpr char kFirmwareName[] = "xiao-telemetry-bridge";
constexpr char kFirmwareVersion[] = "4.0.0-xiao-communication";
// GNSS TX -> D6 (RX), GNSS RX <- D7 (TX).
constexpr int kGnssRxPin = D6, kGnssTxPin = D7;
// Communication D0 (TX) -> control D6 (RX).
// Communication D1 (RX) <- control D7 (TX).
constexpr int kControlUartRxPin = D1, kControlUartTxPin = D0;
// External microSD SPI. Logging remains disabled until the logger is restored.
constexpr int kSdSckPin = D8, kSdMisoPin = D9, kSdMosiPin = D10;
constexpr int kSdCsPin = 21;
constexpr uint32_t kSdSpiHz = 10000000UL;
constexpr uint32_t kControlUartBaud = 921600UL;
constexpr uint32_t kGnssBaud = 115200UL;
constexpr uint16_t kGnssUartRxBufferBytes = 2048, kGnssReadBudgetBytes = 512;
constexpr uint16_t kGnssInputLineChars = 127, kGnssMaxSentenceChars = 110;
constexpr uint32_t kGnssSentenceTimeoutMs = 500UL, kGnssNoDataTimeoutMs = 1500UL;
constexpr uint32_t kGnssNavIntervalMs = 100UL;
constexpr char kApSsid[] = "BOAT-CONTROL", kApPassword[] = "12345678";
constexpr uint16_t kHttpPort = 80;
}
