#ifndef _SLAM_UTILITY_BASIC_GRIDS_H_
#define _SLAM_UTILITY_BASIC_GRIDS_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "unordered_set"
#include "grids.h"

namespace slam_utility {

/* Class BasicGrids Declaration. */
template <typename T>
class BasicGrids: public Grids<T> {

public:
    struct Options {
        Vec2 kRadius = Vec2::Zero();
        Vec2 kStep = Vec2::Zero();
    };

public:
    BasicGrids() = default;
    virtual ~BasicGrids() = default;

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

    uint32_t GetNumberOfProcessedGrids() const { return changed_items_indices_.size(); }

    // Reference for member variables.
    std::vector<T> &buffer() { return buffer_; }
    std::unordered_set<uint32_t> &changed_items_indices() { return changed_items_indices_; }
    // Const reference for member variables.
    const std::vector<T> &buffer() const { return buffer_; }
    const std::unordered_set<uint32_t> &changed_items_indices() const { return changed_items_indices_; }

private:
    std::vector<T> buffer_;
    std::unordered_set<uint32_t> changed_items_indices_;
};

/* Class BasicGrids Definition. */
template <typename T>
void BasicGrids<T>::InitializeBuffer() {
    for (uint32_t i = 0; i < 2; ++i) {
        this->grid_length()[i] = static_cast<int32_t>(this->options().kRadius[i] * 2.0f / this->options().kStep[i]) + 1;
        this->half_grid_length()[i] = this->grid_length()[i] / 2;
    }
    buffer_.resize(this->grid_length()[0] * this->grid_length()[1]);
    changed_items_indices_.clear();
}

template <typename T>
void BasicGrids<T>::RefreshBuffer(const T &value) {
    std::fill_n(buffer_.begin(), buffer_.size(), value);
    this->default_value() = value;
    changed_items_indices_.clear();
}

template <typename T>
void BasicGrids<T>::ResetBuffer() {
    for (const uint32_t &index: changed_items_indices_) {
        buffer_[index] = this->default_value();
    }
    changed_items_indices_.clear();
}

template <typename T>
bool BasicGrids<T>::ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices) const {
    for (uint32_t i = 0; i < grid_indices.size(); ++i) {
        grid_indices[i] = static_cast<int32_t>(std::floor(position[i] / this->options().kStep[i])) + this->half_grid_length()[i];
        RETURN_FALSE_IF(grid_indices[i] < 0 || grid_indices[i] >= this->grid_length()[i]);
    }
    return true;
}

template <typename T>
bool BasicGrids<T>::ConvertPositionTo2DofIndices(const Vec2 &position, typename Grids<T>::Index &grid_indices, typename Grids<T>::Index &map_indices) const {
    for (uint32_t i = 0; i < grid_indices.size(); ++i) {
        grid_indices[i] = static_cast<int32_t>(std::floor(position[i] / this->options().kStep[i])) + this->half_grid_length()[i];
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
bool BasicGrids<T>::TryToOccupy(const typename Grids<T>::Index &grid_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    auto &v = GetGrid(index);
    RETURN_FALSE_IF(v == value);
    v = value;
    changed_items_indices_.insert(index);
    return true;
}

template <typename T>
bool BasicGrids<T>::IsOccupied(const typename Grids<T>::Index &grid_indices, const T &value) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    auto &v = GetGrid(index);
    return v == value;
}

template <typename T>
void BasicGrids<T>::ClearGrid(const typename Grids<T>::Index &grid_indices) {
    const uint32_t index = this->GetBufferIndex(grid_indices[0], grid_indices[1]);
    GetGrid(index) = this->default_value();
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_BASIC_GRIDS_H_
