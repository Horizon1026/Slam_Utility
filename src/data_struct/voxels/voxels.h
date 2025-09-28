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
    Voxels() = default;
    virtual ~Voxels() = default;

    virtual void ResetBuffer() = 0;

    bool TryToOccupy(const Vec3 &position, const T &value);
    bool IsOccupied(const Vec3 &position, const T &value);
    void ClearVoxel(const Vec3 &position);
    virtual bool TryToOccupy(const std::array<int32_t, 3> &indices, const T &value) = 0;
    virtual bool IsOccupied(const std::array<int32_t, 3> &indices, const T &value) = 0;
    virtual void ClearVoxel(const std::array<int32_t, 3> &indices) = 0;

    virtual bool ConvertPositionTo3DofIndices(const Vec3 &position, std::array<int32_t, 3> &indices) = 0;

    uint32_t GetBufferIndex(const std::array<int32_t, 3> &indices) const { return this->GetBufferIndex(indices[0], indices[1], indices[2]); }
    virtual uint32_t GetBufferIndex(int32_t x, int32_t y, int32_t z) const = 0;

    T &GetVoxel(int32_t x, int32_t y, int32_t z) { return this->GetVoxel(this->GetBufferIndex(x, y, z)); }
    T &GetVoxel(const std::array<int32_t, 3> &indices) { return this->GetVoxel(this->GetBufferIndex(indices)); }
    virtual T &GetVoxel(uint32_t index) = 0;

};

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
