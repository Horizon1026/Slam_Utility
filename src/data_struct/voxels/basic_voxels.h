#ifndef _SLAM_UTILITY_BASIC_VOXELS_H_
#define _SLAM_UTILITY_BASIC_VOXELS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "voxels.h"

namespace SLAM_UTILITY {

/* Class BasicVoxels Declaration. */
template <typename T>
class BasicVoxels: public Voxels<T> {

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
    virtual void ResetBuffer() override;

    using Voxels<T>::TryToOccupy;
    using Voxels<T>::IsOccupied;
    using Voxels<T>::ClearVoxel;
    virtual bool TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) override;
    virtual bool IsOccupied(const std::array<int32_t, 3> &indices, const T &value) override;
    virtual void ClearVoxel(const std::array<int32_t, 3> &indices) override;

    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) override;

    using Voxels<T>::GetBufferIndex;
    virtual uint32_t GetBufferIndex(int32_t x, int32_t y, int32_t z) const override { return static_cast<uint32_t>(z * sq_cube_length_ + y * cube_length_ + x); }

    using Voxels<T>::GetVoxel;
    virtual T &GetVoxel(uint32_t index) override { return buffer_[index]; }

    uint32_t GetNumberOfProcessedVoxels() const { return changed_items_indices_.size(); }

private:
    Options options_;

    T default_value_;
    std::vector<T> buffer_;
    std::vector<uint32_t> changed_items_indices_;
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
    changed_items_indices_.reserve(buffer_.size());
    changed_items_indices_.clear();
}

template <typename T>
void BasicVoxels<T>::RefreshBuffer(const T &value) {
    std::fill_n(buffer_.begin(), buffer_.size(), value);
    default_value_ = value;
    changed_items_indices_.clear();
}

template <typename T>
void BasicVoxels<T>::ResetBuffer() {
    for (const uint32_t &index: changed_items_indices_) {
        buffer_[index] = default_value_;
    }
    changed_items_indices_.clear();
}

template <typename T>
bool BasicVoxels<T>::ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) {
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int32_t>(position[i] / options_.kStep) + half_cube_length_;
        RETURN_FALSE_IF(indices[i] < 0 || indices[i] >= cube_length_);
    }
    return true;
}

template <typename T>
bool BasicVoxels<T>::TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = GetBufferIndex(indices[0], indices[1], indices[2]);
    auto &v = GetVoxel(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    changed_items_indices_.emplace_back(index);
    return true;
}

template <typename T>
bool BasicVoxels<T>::IsOccupied(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = GetBufferIndex(indices[0], indices[1], indices[2]);
    auto &v = GetVoxel(index);
    return v == value;
}

template <typename T>
void BasicVoxels<T>::ClearVoxel(const std::array<int32_t, 3> &indices) {
    const uint32_t index = GetBufferIndex(indices[0], indices[1], indices[2]);
    GetVoxel(index) = default_value_;
}

}

#endif // end of _SLAM_UTILITY_BASIC_VOXELS_H_
