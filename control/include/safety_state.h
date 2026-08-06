#pragma once
#include <Arduino.h>
enum class SafetyState:uint8_t { BOOT,DISARMED,ARMED_IDLE,RUNNING,E_STOP,FAULT };
enum class TestProfile:uint8_t { None,SensorOnly,ServoCenter,ServoSlow,VescStatus,VescDuty,ServoVesc };
inline const char* stateName(SafetyState s){switch(s){case SafetyState::BOOT:return "BOOT";case SafetyState::DISARMED:return "DISARMED";case SafetyState::ARMED_IDLE:return "ARMED_IDLE";case SafetyState::RUNNING:return "RUNNING";case SafetyState::E_STOP:return "E_STOP";default:return "FAULT";}}