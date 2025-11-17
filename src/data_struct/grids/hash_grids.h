#ifndef _SLAM_UTILITY_HASH_GRIDS_H_
#define _SLAM_UTILITY_HASH_GRIDS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "grids.h"

namespace slam_utility {

/* Class HashGrids Declaration. */
template <typename T>
class HashGrids: public Grids<T> {

public:
    struct Options {
        Vec2 kRadius = Vec2::Zero();
        Vec2 kStep = Vec2::Zero();
    };

public:
    HashGrids() = default;
    virtual ~HashGrids() = default;

    using Grids<T>::InitializeBuffer;
    virtual void InitializeBuffer() override;
    void RefreshBuffer(const T &value);
    virtual void ResetBuffer() override;

    using Grids<T>::TryToOccupy;
    using Grids<T>::IsOccupied;
    using Grids<T>::ClearGrid;
    virtual bool TryToOccupy(const typename Grids<T>::Index &grid_indices, const T &value) override;
    virtual bool IsOccupied(const typename Grids<T>::Index &grid_indices, const T &value) override;
    virtual void ClearGrid(const typename Grids<T>::Index &grid_indices) override;

    virtual bool ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices) const override;
    virtual bool ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices, typename Grids<T>::Index &map_indices) const override;

    using Grids<T>::GetBufferIndex;
    using Grids<T>::GetGrid;
    virtual T &GetGrid(uint32_t index) override { return buffer_[index]; }

    uint32_t GetNumberOfProcessedGrids() const { return buffer_.size(); }

private:
    std::unordered_map<uint32_t, T> buffer_;
};

/* Class HashGrids Definition. */
template <typename T>
void HashGrids<T>::InitializeBuffer() {
    for (uint32_t i = 0; i < 2; ++i) {
        this->grid_length()[i] = static_cast<int32_t>(this->options().kRadius[i] * 2.0f / this->options().kStep[i]) + 1;
        this->half_grid_length()[i] = this->grid_length()[i] / 2;
    }
    buffer_.clear();
}

template <typename T>
void HashGrids<T>::RefreshBuffer(const T &value) {
    this->default_value() = value;
    buffer_.clear();
}

template <typename T>
void HashGrids<T>::ResetBuffer() {
    buffer_.clear();
}

template <typename T>
bool HashGrids<T>::ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices) const {
    for (uint32_t i = 0; i < grid_indices.size(); ++i) {
        grid_indices[i] = static_cast<int32_t>(position[i] / this->options().kStep[i]) + this->half_grid_length()[i];
        RETURN_FALSE_IF(grid_indices[i] < 0 || grid_indices[i] >= this->grid_length()[i]);
    }
    return true;
}

template <typename T>
bool HashGrids<T>::ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices, typename Grids<T>::Index &map_indices) const {
    for (uint32_t i = 0; i < grid_indices.size(); ++i) {
        grid_indices[i] = static_cast<int32_t>(position[i] / this->options().kStep[i]) + this->half_grid_length()[i];
        const int32_t grid_length = this->grid_length()[i];
        map_indices[i] = grid_indices[i] / grid_length;
        if (grid_indices[i] < 0) {
            --map_indices[i];
        }
        grid_indices[i] = (grid_indices[i] % grid_length + grid_length) % grid_length;
    }
    return true;
}

template <typename T>
bool HashGrids<T>::TryToOccupy(const typename Grids<T>::Index &grid_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    auto &v = GetGrid(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    return true;
}

template <typename T>
bool HashGrids<T>::IsOccupied(const typename Grids<T>::Index &grid_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    auto &v = GetGrid(index);
    return v == value;
}

template <typename T>
void HashGrids<T>::ClearGrid(const typename Grids<T>::Index &grid_indices) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    GetGrid(index) = this->default_value();
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_HASH_GRIDS_H_
