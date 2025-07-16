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
    void ResetBuffer();

    bool TryToOccupy(const Vec3 &position, const T &value);
    bool IsOccupied(const Vec3 &position, const T &value);
    bool TryToOccupy(const std::array<int32_t, 3> &indices, const T &value);
    bool IsOccupied(const std::array<int32_t, 3> &indices, const T &value);

    void GeneratePointCloud(const Vec3 &mid_p_w, std::vector<Vec3> &all_p_w);
    void GeneratePointCloud(const Vec3 &mid_p_w, const T &value, std::vector<Vec3> &all_p_w);

    bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices);
    uint32_t GetBufferIndex(int32_t x, int32_t y, int32_t z) const { return static_cast<uint32_t>(z * sq_cube_length_ + y * cube_length_ + x); }
    uint32_t GetBufferIndex(const std::array<int32_t, 3> &indices) const { return GetBufferIndex(indices[0], indices[1], indices[2]); }
    T &GetVoxel(int32_t x, int32_t y, int32_t z) { return buffer_[GetBufferIndex(x, y, z)]; }
    T &GetVoxel(uint32_t index) { return buffer_[index]; }

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
bool BasicVoxels<T>::TryToOccupy(const Vec3 &position, const T &value) {
    std::array<int32_t, 3> indices;
    RETURN_FALSE_IF(!ConvertPositionTo3DofIndices(position, indices));
    return TryToOccupy(indices, value);
}

template <typename T>
bool BasicVoxels<T>::IsOccupied(const Vec3 &position, const T &value) {
    std::array<int32_t, 3> indices;
    RETURN_FALSE_IF(!ConvertPositionTo3DofIndices(position, indices));
    return IsOccupied(indices, value);
}

template <typename T>
bool BasicVoxels<T>::TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) {
    const uint32_t index = GetBufferIndex(indices);
    auto &v = GetVoxel(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    changed_items_indices_.emplace_back(index);
    return true;
}

template <typename T>
bool BasicVoxels<T>::IsOccupied(const std::array<int32_t, 3> &indices, const T &value) {
    auto &v = GetVoxel(indices[0], indices[1], indices[2]);
    return v == value;
}

template <typename T>
void BasicVoxels<T>::GeneratePointCloud(const Vec3 &mid_p_w, std::vector<Vec3> &all_p_w) {
    all_p_w.clear();
    all_p_w.reserve(buffer_.size());
    for (int32_t x = 0; x < cube_length_; ++x) {
        for (int32_t y = 0; y < cube_length_; ++y) {
            for (int32_t z = 0; z < cube_length_; ++z) {
                const Vec3 position = Vec3(
                    (x - half_cube_length_) * options_.kStep,
                    (y - half_cube_length_) * options_.kStep,
                    (z - half_cube_length_) * options_.kStep);
                all_p_w.emplace_back(position + mid_p_w);
            }
        }
    }
}

template <typename T>
void BasicVoxels<T>::GeneratePointCloud(const Vec3 &mid_p_w, const T &value, std::vector<Vec3> &all_p_w) {
    all_p_w.clear();
    all_p_w.reserve(buffer_.size());
    for (int32_t x = 0; x < cube_length_; ++x) {
        for (int32_t y = 0; y < cube_length_; ++y) {
            for (int32_t z = 0; z < cube_length_; ++z) {
                CONTINUE_IF(GetVoxel(x, y, z) != value);
                const Vec3 position = Vec3(
                    (x - half_cube_length_) * options_.kStep,
                    (y - half_cube_length_) * options_.kStep,
                    (z - half_cube_length_) * options_.kStep);
                all_p_w.emplace_back(position + mid_p_w);
            }
        }
    }
}

}

#endif // end of _SLAM_UTILITY_BASIC_VOXELS_H_
