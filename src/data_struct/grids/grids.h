#ifndef _SLAM_UTILITY_GRIDS_H_
#define _SLAM_UTILITY_GRIDS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace slam_utility {

/* Class Grids Declaration. */
template <typename T>
class Grids {

public:
    using Index = std::array<int32_t, 2>;
    struct Options {
        Vec2 kRadius = Vec2::Zero();
        Vec2 kStep = Vec2::Zero();
    };

public:
    Grids() = default;
    virtual ~Grids() = default;

    void InitializeBuffer(const Vec2 &radius, const Vec2 &step);
    void InitializeBuffer(const float radius, const float step);
    virtual void InitializeBuffer() = 0;
    virtual void ResetBuffer() = 0;

    bool TryToOccupy(const Vec2 &position, const T &value);
    bool IsOccupied(const Vec2 &position, const T &value);
    void ClearGrid(const Vec2 &position);
    virtual bool TryToOccupy(const Index &grid_indices, const T &value) = 0;
    virtual bool IsOccupied(const Index &grid_indices, const T &value) = 0;
    virtual void ClearGrid(const Index &grid_indices) = 0;

    virtual bool ConvertPositionTo2DofIndices(const Vec2 &position, Index &grid_indices) const = 0;
    virtual bool ConvertPositionTo2DofIndices(const Vec2 &position, Index &grid_indices, Index &map_indices) const = 0;
    void ConvertLocalIndicesToGlobalIndices(const Index &grid_indices, const Index &map_index,
                                            Index &global_grid_indices) const;

    uint32_t GetBufferIndex(const Index &grid_indices) const {
        return this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    }
    uint32_t GetBufferIndex(int32_t x, int32_t y) const {
        return static_cast<uint32_t>(y * grid_length_[0] + x);
    }

    T &GetGrid(int32_t x, int32_t y) { return this->GetGrid(this->GetBufferIndex(x, y)); }
    T &GetGrid(const Index &grid_indices) { return this->GetGrid(this->GetBufferIndex(grid_indices)); }
    virtual T &GetGrid(uint32_t index) = 0;

    static bool IsSameIndices(const Index &indices1, const Index &indices2) {
        return indices1[0] == indices2[0] && indices1[1] == indices2[1];
    }

    // Reference for member variables.
    Options &options() { return options_; }
    T &default_value() { return default_value_; }
    TVec2<int32_t> &grid_length() { return grid_length_; }
    TVec2<int32_t> &half_grid_length() { return half_grid_length_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }
    const T &default_value() const { return default_value_; }
    const TVec2<int32_t> &grid_length() const { return grid_length_; }
    const TVec2<int32_t> &half_grid_length() const { return half_grid_length_; }

private:
    Options options_;
    T default_value_;
    TVec2<int32_t> grid_length_ = TVec2<int32_t>::Zero();
    TVec2<int32_t> half_grid_length_ = TVec2<int32_t>::Zero();
};

template <typename T>
void Grids<T>::InitializeBuffer(const Vec2 &radius, const Vec2 &step) {
    options_.kRadius = radius;
    options_.kStep = step;
    InitializeBuffer();
}

template <typename T>
void Grids<T>::InitializeBuffer(const float radius, const float step) {
    InitializeBuffer(Vec2(radius, radius), Vec2(step, step));
}

template <typename T>
bool Grids<T>::TryToOccupy(const Vec2 &position, const T &value) {
    Index grid_indices;
    RETURN_FALSE_IF(!this->ConvertPositionTo2DofIndices(position, grid_indices));
    return this->TryToOccupy(grid_indices, value);
}

template <typename T>
bool Grids<T>::IsOccupied(const Vec2 &position, const T &value) {
    Index grid_indices;
    RETURN_FALSE_IF(!this->ConvertPositionTo2DofIndices(position, grid_indices));
    return this->IsOccupied(grid_indices, value);
}

template <typename T>
void Grids<T>::ClearGrid(const Vec2 &position) {
    Index grid_indices;
    RETURN_IF(!this->ConvertPositionTo2DofIndices(position, grid_indices));
    this->ClearGrid(grid_indices);
}

template <typename T>
void Grids<T>::ConvertLocalIndicesToGlobalIndices(const Index &grid_indices, const Index &map_index,
                                                  Index &global_grid_indices) const {
    for (uint32_t i = 0; i < grid_indices.size(); ++i) {
        global_grid_indices[i] = map_index[i] * grid_length_[i] + grid_indices[i];
    }
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_GRIDS_H_
