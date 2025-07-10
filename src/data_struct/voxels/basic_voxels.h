#ifndef _SLAM_UTILITY_BASIC_VOXELS_H_
#define _SLAM_UTILITY_BASIC_VOXELS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

/* Class BasicVoxels Declaration. */
template <typename T>
class BasicVoxels {

public:
    struct Options {
        float kRadius = 0.0f;
        float kStep = 0.0f;
    };

public:
    BasicVoxels() = default;
    virtual ~BasicVoxels() = default;

    void InitializeBuffer(const float radius, const float step);
    void RefreshBuffer(const T &value);

    bool TryToOccupy(const Vec3 &position, const T &value);
    bool IsOccupied(const Vec3 &position, const T &value);

    uint32_t size() { return buffer_.size(); }
    T &voxel(int32_t x, int32_t y, int32_t z) { return buffer_[z * sq_cube_length_ + y * cube_length_ + x]; }

private:
    Options options_;

    std::vector<T> buffer_;
    int32_t cube_length_ = 0;
    int32_t half_cube_length_ = 0;
    int32_t sq_cube_length_ = 0;
};

/* Class BasicVoxels Definition. */
template <typename T>
void BasicVoxels<T>::InitializeBuffer(const float radius, const float step) {
    options_.kRadius = radius;
    options_.kStep = step;
    cube_length_ = static_cast<int32_t>(options_.kRadius * 2.0f / options_.kStep) + 1;
    half_cube_length_ = cube_length_ >> 1;
    sq_cube_length_ = cube_length_ * cube_length_;
    buffer_.resize(sq_cube_length_ * cube_length_);
}

template <typename T>
void BasicVoxels<T>::RefreshBuffer(const T &value) {
    std::fill_n(buffer_.begin(), buffer_.size(), value);
}

template <typename T>
bool BasicVoxels<T>::TryToOccupy(const Vec3 &position, const T &value) {
    const std::array<int32_t, 3> indices = std::array<int32_t, 3>{
          static_cast<int32_t>(position.x() / options_.kStep) + half_cube_length_,
          static_cast<int32_t>(position.y() / options_.kStep) + half_cube_length_,
          static_cast<int32_t>(position.z() / options_.kStep) + half_cube_length_,
    };
    for (uint32_t i = 0; i < 3; ++i) {
        RETURN_FALSE_IF(indices[i] < 0 || indices[i] >= cube_length_);
    }

    auto &v = voxel(indices[0], indices[1], indices[2]);
    if (v == value) {
        return false;
    }
    v = value;
    return true;
}

template <typename T>
bool BasicVoxels<T>::IsOccupied(const Vec3 &position, const T &value) {
    const std::array<int32_t, 3> indices = std::array<int32_t, 3>{
          static_cast<int32_t>(position.x() / options_.kStep) + half_cube_length_,
          static_cast<int32_t>(position.y() / options_.kStep) + half_cube_length_,
          static_cast<int32_t>(position.z() / options_.kStep) + half_cube_length_,
    };
    for (uint32_t i = 0; i < 3; ++i) {
        RETURN_FALSE_IF(indices[i] < 0 || indices[i] >= cube_length_);
    }

    auto &v = voxel(indices[0], indices[1], indices[2]);
    if (v == value) {
        return true;
    }
    return false;
}

}

#endif // end of _SLAM_UTILITY_BASIC_VOXELS_H_
