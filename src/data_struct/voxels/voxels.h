#ifndef _SLAM_UTILITY_VOXELS_H_
#define _SLAM_UTILITY_VOXELS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

/* Class Voxels Declaration. */
template <typename T>
class Voxels {

public:
    struct Options {
        Vec3 kRadius = Vec3::Zero();
        Vec3 kStep = Vec3::Zero();
    };

public:
    Voxels() = default;
    virtual ~Voxels() = default;

    void InitializeBuffer(const Vec3 &radius, const Vec3 &step);
    void InitializeBuffer(const float radius, const float step);
    virtual void InitializeBuffer() = 0;
    virtual void ResetBuffer() = 0;

    bool TryToOccupy(const Vec3 &position, const T &value);
    bool IsOccupied(const Vec3 &position, const T &value);
    void ClearVoxel(const Vec3 &position);
    virtual bool TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) = 0;
    virtual bool IsOccupied(const std::array<int32_t, 3> &indices, const T &value) = 0;
    virtual void ClearVoxel(const std::array<int32_t, 3> &indices) = 0;

    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) const = 0;
    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices, std::array<int32_t, 3> &map_indices) const = 0;

    uint32_t GetBufferIndex(const std::array<int32_t, 3> &indices) const { return this->GetBufferIndex(indices[0], indices[1], indices[2]); }
    uint32_t GetBufferIndex(int32_t x, int32_t y, int32_t z) const { return static_cast<uint32_t>(z * voxel_length_[0] * voxel_length_[1] + y * voxel_length_[0] + x); }

    T &GetVoxel(int32_t x, int32_t y, int32_t z) { return this->GetVoxel(this->GetBufferIndex(x, y, z)); }
    T &GetVoxel(const std::array<int32_t, 3> &indices) { return this->GetVoxel(this->GetBufferIndex(indices)); }
    virtual T &GetVoxel(uint32_t index) = 0;

    // Reference for member variables.
    Options &options() { return options_; }
    T &default_value() { return default_value_; }
    TVec3<int32_t> &voxel_length() { return voxel_length_; }
    TVec3<int32_t> &half_voxel_length() { return half_voxel_length_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }
    const T &default_value() const { return default_value_; }
    const TVec3<int32_t> &voxel_length() const { return voxel_length_; }
    const TVec3<int32_t> &half_voxel_length() const { return half_voxel_length_; }

private:
    Options options_;
    T default_value_;
    TVec3<int32_t> voxel_length_ = TVec3<int32_t>::Zero();
    TVec3<int32_t> half_voxel_length_ = TVec3<int32_t>::Zero();

};

template <typename T>
void Voxels<T>::InitializeBuffer(const Vec3 &radius, const Vec3 &step) {
    options_.kRadius = radius;
    options_.kStep = step;
    InitializeBuffer();
}

template <typename T>
void Voxels<T>::InitializeBuffer(const float radius, const float step) {
    InitializeBuffer(Vec3(radius, radius, radius), Vec3(step, step, step));
}

template <typename T>
bool Voxels<T>::TryToOccupy(const Vec3 &position, const T &value) {
    std::array<int32_t, 3> indices;
    RETURN_FALSE_IF(!this->ConvertPositionTo3DofIndices(position, indices));
    return this->TryToOccupy(indices, value);
}

template <typename T>
bool Voxels<T>::IsOccupied(const Vec3 &position, const T &value) {
    std::array<int32_t, 3> indices;
    RETURN_FALSE_IF(!this->ConvertPositionTo3DofIndices(position, indices));
    return this->IsOccupied(indices, value);
}

template <typename T>
void Voxels<T>::ClearVoxel(const Vec3 &position) {
    std::array<int32_t, 3> indices;
    RETURN_IF(!this->ConvertPositionTo3DofIndices(position, indices));
    this->ClearVoxel(indices);
}

}
#endif // end of _SLAM_UTILITY_VOXELS_H_
