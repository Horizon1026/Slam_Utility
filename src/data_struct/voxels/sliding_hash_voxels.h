#ifndef _SLAM_UTILITY_SLIDING_HASH_VOXELS_H_
#define _SLAM_UTILITY_SLIDING_HASH_VOXELS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "voxels.h"

namespace SLAM_UTILITY {

/* Class SlidingHashVoxels Declaration. */
template <typename T>
class SlidingHashVoxels: public Voxels<T> {

public:
    struct Options {
        Vec3 kRadius = Vec3::Zero();
        Vec3 kStep = Vec3::Zero();
    };

public:
    SlidingHashVoxels() = default;
    virtual ~SlidingHashVoxels() = default;

    void InitializeBuffer(const Vec3 &radius, const Vec3 &step);
    void InitializeBuffer();
    virtual void ResetBuffer() override;

    using Voxels<T>::TryToOccupy;
    using Voxels<T>::IsOccupied;
    using Voxels<T>::ClearVoxel;
    virtual bool TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) override;
    virtual bool IsOccupied(const std::array<int32_t, 3> &indices, const T &value) override;
    virtual void ClearVoxel(const std::array<int32_t, 3> &indices) override;

    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) override;

    using Voxels<T>::GetBufferIndex;
    virtual uint32_t GetBufferIndex(const std::array<int32_t, 3> &indices) const override;

    using Voxels<T>::GetVoxel;
    virtual T &GetVoxel(uint32_t index) override { return buffer_[index]; }

private:
    Options options_;

    T default_value_;
    std::unordered_map<uint32_t, T> buffer_;
    TVec3<int32_t> voxel_length_ = TVec3<int32_t>::Zero();
    TVec3<int32_t> half_voxel_length_ = TVec3<int32_t>::Zero();
};

/* Class SlidingHashVoxels Definition. */
template <typename T>
void SlidingHashVoxels<T>::InitializeBuffer(const Vec3 &radius, const Vec3 &step) {
    options_.kRadius = radius;
    options_.kStep = step;
    InitializeBuffer();
}

template <typename T>
void SlidingHashVoxels<T>::InitializeBuffer() {
    voxel_length_ = (options_.kRadius * 2.0f / options_.kStep).cast<int32_t>() + 1;
    half_voxel_length_ = voxel_length_ >> 1;
    buffer_.clear();
}

template <typename T>
void SlidingHashVoxels<T>::ResetBuffer() {
    buffer_.clear();
}

template <typename T>
bool SlidingHashVoxels<T>::ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) {
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int32_t>(position[i] / options_.kStep[i]) + half_voxel_length_[i];
        // Move index into [0, voxel_length_[i] - 1] with step = voxel_length_[i].
        indices[i] = (indices[i] % voxel_length_[i] + voxel_length_[i]) % voxel_length_[i];
    }
    return true;
}

template <typename T>
uint32_t SlidingHashVoxels<T>::GetBufferIndex(const std::array<int32_t, 3> &indices) const {
    const int32_t index = indices[0] + indices[1] * voxel_length_[0] + indices[2] * voxel_length_[0] * voxel_length_[1];
    return static_cast<uint32_t>(index);
}

template <typename T>
bool SlidingHashVoxels<T>::TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = GetBufferIndex(indices);
    auto &v = GetVoxel(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    return true;
}

template <typename T>
bool SlidingHashVoxels<T>::IsOccupied(const std::array<int32_t, 3> &indices, const T &value) {
    auto &v = GetVoxel(indices);
    return v == value;
}

template <typename T>
void SlidingHashVoxels<T>::ClearVoxel(const std::array<int32_t, 3> &indices) {
    GetVoxel(indices) = default_value_;
}


}

#endif // end of _SLAM_UTILITY_SLIDING_HASH_VOXELS_H_
