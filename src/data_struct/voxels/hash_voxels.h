#ifndef _SLAM_UTILITY_HASH_VOXELS_H_
#define _SLAM_UTILITY_HASH_VOXELS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "voxels.h"

namespace slam_utility {

/* Class HashVoxels Declaration. */
template <typename T>
class HashVoxels: public Voxels<T> {

public:
    struct Options {
        Vec3 kRadius = Vec3::Zero();
        Vec3 kStep = Vec3::Zero();
    };

public:
    HashVoxels() = default;
    virtual ~HashVoxels() = default;

    using Voxels<T>::InitializeBuffer;
    virtual void InitializeBuffer() override;
    void RefreshBuffer(const T &value);
    virtual void ResetBuffer() override;

    using Voxels<T>::TryToOccupy;
    using Voxels<T>::IsOccupied;
    using Voxels<T>::ClearVoxel;
    virtual bool TryToOccupy(const typename Voxels<T>::Index &voxel_indices, const T &value) override;
    virtual bool IsOccupied(const typename Voxels<T>::Index &voxel_indices, const T &value) override;
    virtual void ClearVoxel(const typename Voxels<T>::Index &voxel_indices) override;

    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, typename Voxels<T>::Index &voxel_indices) const override;
    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, typename Voxels<T>::Index &voxel_indices, typename Voxels<T>::Index &map_indices) const override;

    using Voxels<T>::GetBufferIndex;
    using Voxels<T>::GetVoxel;
    virtual T &GetVoxel(uint32_t index) override { return buffer_[index]; }

    uint32_t GetNumberOfProcessedVoxels() const { return buffer_.size(); }

private:
    std::unordered_map<uint32_t, T> buffer_;
};

/* Class HashVoxels Definition. */
template <typename T>
void HashVoxels<T>::InitializeBuffer() {
    for (uint32_t i = 0; i < 3; ++i) {
        this->voxel_length()[i] = static_cast<int32_t>(this->options().kRadius[i] * 2.0f / this->options().kStep[i]) + 1;
        this->half_voxel_length()[i] = this->voxel_length()[i] / 2;
    }
    buffer_.clear();
}

template <typename T>
void HashVoxels<T>::RefreshBuffer(const T &value) {
    this->default_value() = value;
    buffer_.clear();
}

template <typename T>
void HashVoxels<T>::ResetBuffer() {
    buffer_.clear();
}

template <typename T>
bool HashVoxels<T>::ConvertPositionTo3DofIndices(const Vec3 &position, typename Voxels<T>::Index &voxel_indices) const {
    for (uint32_t i = 0; i < voxel_indices.size(); ++i) {
        voxel_indices[i] = static_cast<int32_t>(position[i] / this->options().kStep[i]) + this->half_voxel_length()[i];
        RETURN_FALSE_IF(voxel_indices[i] < 0 || voxel_indices[i] >= this->voxel_length()[i]);
    }
    return true;
}

template <typename T>
bool HashVoxels<T>::ConvertPositionTo3DofIndices(const Vec3 &position, typename Voxels<T>::Index &voxel_indices, typename Voxels<T>::Index &map_indices) const {
    for (uint32_t i = 0; i < voxel_indices.size(); ++i) {
        voxel_indices[i] = static_cast<int32_t>(position[i] / this->options().kStep[i]) + this->half_voxel_length()[i];
        const int32_t voxel_length = this->voxel_length()[i];
        map_indices[i] = voxel_indices[i] / voxel_length;
        if (voxel_indices[i] < 0) {
            --map_indices[i];
        }
        voxel_indices[i] = (voxel_indices[i] % voxel_length + voxel_length) % voxel_length;
    }
    return true;
}

template <typename T>
bool HashVoxels<T>::TryToOccupy(const typename Voxels<T>::Index &voxel_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(voxel_indices[0], voxel_indices[1], voxel_indices[2]);
    auto &v = GetVoxel(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    return true;
}

template <typename T>
bool HashVoxels<T>::IsOccupied(const typename Voxels<T>::Index &voxel_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(voxel_indices[0], voxel_indices[1], voxel_indices[2]);
    auto &v = GetVoxel(index);
    return v == value;
}

template <typename T>
void HashVoxels<T>::ClearVoxel(const typename Voxels<T>::Index &voxel_indices) {
    const uint32_t index = this->GetBufferIndex(voxel_indices[0], voxel_indices[1], voxel_indices[2]);
    GetVoxel(index) = this->default_value();
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_HASH_VOXELS_H_