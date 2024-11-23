#ifndef _SENSOR_UTILITY_VISUAL_OBSERVE_H_
#define _SENSOR_UTILITY_VISUAL_OBSERVE_H_

#include "basic_type.h"
#include "unordered_map"

namespace SLAM_UTILITY {

// Define feature point to be [x, y, 1], and only store [x, y].
using VisualPointFeatureObserve = Vec2;
using VisualPointFeatureObserveMultiView = std::unordered_map<int32_t, Vec2>;

// [Plucker] Define feature line to be [l, m] (Vec3, Vec3).
using VisualLineFeatureObserve = Vec6;
using VisualLineFeatureObserveMultiView = std::unordered_map<int32_t, Vec6>;

}

#endif // end of _SENSOR_UTILITY_VISUAL_OBSERVE_H_
