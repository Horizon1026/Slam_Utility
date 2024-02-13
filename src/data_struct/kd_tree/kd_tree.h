#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "datatype_basic.h"
#include "memory"

namespace SLAM_UTILITY {

/* Options of KD-Tree. */
struct OptionsOfKdTree {
    int32_t kNumOfPointsInLeaf = 1;
};

/* Class KD-Tree Declaration. */
template <typename Scalar, int32_t Dimension>
class KdTreeNode {

public:
    KdTreeNode() = default;
    virtual ~KdTreeNode() = default;

    bool IsLeaf() const { return axis_ < 0; }

    std::unique_ptr<KdTreeNode<Scalar, Dimension>> Construct(KdTreeNode<Scalar, Dimension> *root_ptr,
        const std::vector<Scalar> &points,
        const std::vector<int32_t> &sorted_point_indices,
        const int32_t axis);

    // Reference for member variables.
    int32_t &axis() { return axis_; }
    Scalar &value() { return value_; }
    KdTreeNode<Scalar, Dimension> &left_ptr() { return left_ptr_.get(); }
    KdTreeNode<Scalar, Dimension> &right_ptr() { return right_ptr_.get(); }
    std::vector<int32_t> &point_indices() { return point_indices_; }

    // Const reference for member variables.
    const int32_t &axis() const { return axis_; }
    const Scalar &value() const { return value_; }
    const std::vector<int32_t> &point_indices() const { return point_indices_; }

private:
    OptionsOfKdTree options_;

    int32_t axis_ = 0;
    Scalar value_ = static_cast<Scalar>(0);
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> left_ptr_ = nullptr;
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> right_ptr_ = nullptr;
    std::vector<int32_t> point_indices_;
};

/* Class KD-Tree Definition. */

}

#endif // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
