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
        Vec3 kRadius = Vec3::Zero();
        Vec3 kStep = Vec3::Zero();
    };

public:
    BasicVoxels() = default;
    virtual ~BasicVoxels() = default;

    using Voxels<T>::InitializeBuffer;
    virtual void InitializeBuffer() override;
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
    using Voxels<T>::GetVoxel;
    virtual T &GetVoxel(uint32_t index) override { return buffer_[index]; }

    uint32_t GetNumberOfProcessedVoxels() const { return changed_items_indices_.size(); }

private:
    std::vector<T> buffer_;
    std::vector<uint32_t> changed_items_indices_;
};

/* Class BasicVoxels Definition. */
template <typename T>
void BasicVoxels<T>::InitializeBuffer() {
    for (uint32_t i = 0; i < 3; ++i) {
        this->voxel_length()[i] = static_cast<int32_t>(this->options().kRadius[i] * 2.0f / this->options().kStep[i]) + 1;
        this->half_voxel_length()[i] = this->voxel_length()[i] / 2;
    }
    buffer_.resize(this->voxel_length()[0] * this->voxel_length()[1] * this->voxel_length()[2]);
    changed_items_indices_.reserve(buffer_.size());
    changed_items_indices_.clear();
}

template <typename T>
void BasicVoxels<T>::RefreshBuffer(const T &value) {
    std::fill_n(buffer_.begin(), buffer_.size(), value);
    this->default_value() = value;
    changed_items_indices_.clear();
}

template <typename T>
void BasicVoxels<T>::ResetBuffer() {
    for (const uint32_t &index: changed_items_indices_) {
        buffer_[index] = this->default_value();
    }
    changed_items_indices_.clear();
}

template <typename T>
bool BasicVoxels<T>::ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) {
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int32_t>(position[i] / this->options().kStep[i]) + this->half_voxel_length()[i];
        RETURN_FALSE_IF(indices[i] < 0 || indices[i] >= this->voxel_length()[i]);
    }
    return true;
}

template <typename T>
bool BasicVoxels<T>::TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(indices[0], indices[1], indices[2]);
    auto &v = GetVoxel(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    changed_items_indices_.emplace_back(index);
    return true;
}

template <typename T>
bool BasicVoxels<T>::IsOccupied(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(indices[0], indices[1], indices[2]);
    auto &v = GetVoxel(index);
    return v == value;
}

template <typename T>
void BasicVoxels<T>::ClearVoxel(const std::array<int32_t, 3> &indices) {
    const uint32_t index = this->GetBufferIndex(indices[0], indices[1], indices[2]);
    GetVoxel(index) = this->default_value();
}

}

#endif // end of _SLAM_UTILITY_BASIC_VOXELS_H_
