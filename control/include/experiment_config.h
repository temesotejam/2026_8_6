#pragma once

// One firmware image performs exactly one fixed-condition experiment.
// BOAT_EXPERIMENT is set by the matching PlatformIO environment.
#ifndef BOAT_EXPERIMENT
#define BOAT_EXPERIMENT 0
#endif

namespace experiment_config {
#if BOAT_EXPERIMENT == 0
constexpr char kName[] = "P0_bringup_400k"; constexpr uint8_t kPhase = 0; constexpr uint32_t kDurationMs = 60000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 1
constexpr char kName[] = "P1_stability_400k"; constexpr uint8_t kPhase = 0; constexpr uint32_t kDurationMs = 600000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 2
constexpr char kName[] = "P2_i2c_100k"; constexpr uint8_t kPhase = 9; constexpr uint32_t kDurationMs = 600000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 100000UL;
#elif BOAT_EXPERIMENT == 3
constexpr char kName[] = "P4_ina_current"; constexpr uint8_t kPhase = 1; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 4
constexpr char kName[] = "P4_ina_balanced"; constexpr uint8_t kPhase = 2; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 1, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 5
constexpr char kName[] = "P4_ina_fast"; constexpr uint8_t kPhase = 3; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 2, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 6
constexpr char kName[] = "P3_tof_8x8_10"; constexpr uint8_t kPhase = 4; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 7
constexpr char kName[] = "P3_tof_8x8_15"; constexpr uint8_t kPhase = 5; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 1, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 8
constexpr char kName[] = "P3_tof_4x4_15"; constexpr uint8_t kPhase = 6; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 2, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 9
constexpr char kName[] = "P3_tof_4x4_30"; constexpr uint8_t kPhase = 7; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 10
constexpr char kName[] = "P5_uart_base"; constexpr uint8_t kPhase = 11; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 11
constexpr char kName[] = "P5_uart_expected"; constexpr uint8_t kPhase = 12; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 1; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 12
constexpr char kName[] = "P5_uart_double"; constexpr uint8_t kPhase = 13; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 2; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 13
constexpr char kName[] = "P5_uart_target70"; constexpr uint8_t kPhase = 14; constexpr uint32_t kDurationMs = 300000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 3; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 14
constexpr char kName[] = "P6_composite_10min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 600000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 1; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 15
constexpr char kName[] = "P7_endurance_60min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 3600000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 0, kUartProfile = 1; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 16
constexpr char kName[] = "P3_diag_4x4_30"; constexpr uint8_t kPhase = 7; constexpr uint32_t kDurationMs = 120000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 17
constexpr char kName[] = "VESC_passive_3min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 180000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 18
constexpr char kName[] = "BNO_K3_game_rotation_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 19
constexpr char kName[] = "BNO_K5_linear_accel_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 20
constexpr char kName[] = "BNO_ALL_5reports_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 27
constexpr char kName[] = "BNO_K1_accelerometer100_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 28
constexpr char kName[] = "BNO_K2_gyro100_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 29
constexpr char kName[] = "BNO_K4_magnetic20_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 30
constexpr char kName[] = "BNO_ALL_100_100_20_50_50_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 25
constexpr char kName[] = "BNO_K1_accelerometer_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 26
constexpr char kName[] = "BNO_K4_magnetic_10s"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 10000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 21
constexpr char kName[] = "BNO_attitude100_gyro100_3min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 180000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 22
constexpr char kName[] = "BNO_accel100_gyro100_INT_3min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 180000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 23
constexpr char kName[] = "BNO_accel100_gyro100_mag20_INT_3min"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 180000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 24
constexpr char kName[] = "BNO_accel100_gyro100_mag20_INT_60sec"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 60000UL; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#elif BOAT_EXPERIMENT == 31
constexpr char kName[] = "BNO_control_rotation50_gyro100_accel100_mag20"; constexpr uint8_t kPhase = 15; constexpr uint32_t kDurationMs = 0; constexpr uint8_t kInaProfile = 0, kTofProfile = 3, kUartProfile = 0; constexpr uint32_t kPeripheralI2cHz = 400000UL;
#else
#error "BOAT_EXPERIMENT must be 0 through 31"
#endif
}
