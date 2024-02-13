#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "datatype_basic.h"
#include "memory"

namespace SLAM_UTILITY {

/* Class KD-Tree Declaration. */
template <typename Scalar, int32_t Dimension>
class KdTreeNode {

public:
    using ValueType = Eigen::Matrix<Scalar, Dimension, 1>;

public:
    KdTreeNode() = default;
    virtual ~KdTreeNode() = default;

    bool IsLeaf() const { return axis_ < 0; }

    // Reference for member variables.
    int32_t &axis() { return axis_; }
    ValueType &value() { return value_; }
    KdTreeNode<Scalar, Dimension> &left_ptr() { return left_ptr_.get(); }
    KdTreeNode<Scalar, Dimension> &right_ptr() { return right_ptr_.get(); }
    std::vector<int32_t> &points_index() { return points_index_; }

    // Const reference for member variables.
    const int32_t &axis() const { return axis_; }
    const ValueType &value() const { return value_; }
    const std::vector<int32_t> &points_index() const { return points_index_; }

private:
    int32_t axis_ = 0;
    ValueType value_ = ValueType::Zero();
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> left_ptr_ = nullptr;
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> right_ptr_ = nullptr;
    std::vector<int32_t> points_index_;
};

/* Class KD-Tree Declaration. */
template <typename T>
class KdTree {

};

/* Class KD-Tree Definition. */

}

#endif // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
