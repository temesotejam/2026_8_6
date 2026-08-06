#pragma once

#include <stddef.h>
#include <stdint.h>

namespace fixed_waypoints {

struct Point {
  char name;
  double latitudeDeg;
  double longitudeDeg;
};

// Source: temesotejam/waypoint_drift_los_goal_time_sim, main@24868f3.
constexpr Point kPoints[] = {
    {'A', 35.45327, 136.07198},
    {'B', 35.44437, 136.07399},
    {'C', 35.43214, 136.07628},
    {'D', 35.42587, 136.09377},
    {'E', 35.42542, 136.12038},
    {'F', 35.43055, 136.14585},
    {'G', 35.44150, 136.12081},
    {'H', 35.44196, 136.09429},
};

constexpr size_t kCount = sizeof(kPoints) / sizeof(kPoints[0]);

inline const Point* byIndex(uint8_t index) {
  return index < kCount ? &kPoints[index] : nullptr;
}

}  // namespace fixed_waypoints
