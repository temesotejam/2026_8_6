#pragma once
#include <Arduino.h>
struct TofFrame { bool valid=false; uint32_t frame=0; uint64_t rxUs=0; uint16_t distanceMm[64]{}; uint8_t status[64]{}; uint8_t validZones=0; uint16_t minMm=0,medianMm=0,maxMm=0; uint32_t readUs=0; };
struct SystemHealth { uint32_t bootId=0; uint32_t linkDrops=0; uint32_t linkCrcErrors=0; uint32_t linkCobsErrors=0; uint32_t linkLengthErrors=0; uint32_t minFreeHeap=UINT32_MAX; };