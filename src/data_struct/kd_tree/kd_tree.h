#ifndef _SLAM_UTILITY_CIRCULAR_BUFFER_H_
#define _SLAM_UTILITY_CIRCULAR_BUFFER_H_

#include "datatype_basic.h"
#include "log_report.h"
#include "slam_operations.h"

#include "memory"
#include "vector"
#include "map"

namespace SLAM_UTILITY {

/* Options of KD-Tree. */
struct OptionsOfKdTree {
    int32_t kMaxNumberOfPointsInLeafNode = 1;
};

/* Class KD-Tree Declaration. */
template <typename Scalar, int32_t Dimension>
class KdTreeNode {

public:
    KdTreeNode() = default;
    virtual ~KdTreeNode() = default;

    void Construct(const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                   const std::vector<int32_t> &point_indices,
                   const int32_t specified_axis,
                   std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr);
    void ExtractAllPoints(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                          const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                          std::vector<int32_t> &point_indices);
    void SearchKnn(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                   const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                   const Eigen::Matrix<Scalar, Dimension, 1> &target_point,
                   const uint32_t target_number,
                   std::map<float, int32_t> &residual_index_of_points);
    void SearchRadius(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                      const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                      const Eigen::Matrix<Scalar, Dimension, 1> &target_point,
                      const Scalar max_radius,
                      std::map<float, int32_t> &residual_index_of_points);

    int32_t GetAxisWithMaxRange(const std::vector<int32_t> &point_indices,
                                const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points);

    bool IsLeafNode() const { return !point_indices_.empty(); }

    void Information();
    void InformationRecursion();

    // Reference for member variables.
    int32_t &axis() { return axis_; }
    Scalar &divider() { return divider_; }
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> &left_ptr() { return left_ptr_; }
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> &right_ptr() { return right_ptr_; }
    std::vector<int32_t> &point_indices() { return point_indices_; }

    // Const reference for member variables.
    const int32_t &axis() const { return axis_; }
    const Scalar &divider() const { return divider_; }
    const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &left_ptr() const { return left_ptr_; }
    const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &right_ptr() const { return right_ptr_; }
    const std::vector<int32_t> &point_indices() const { return point_indices_; }

private:
    OptionsOfKdTree options_;

    int32_t axis_ = 0;
    Scalar divider_ = static_cast<Scalar>(0);
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> left_ptr_ = nullptr;
    std::unique_ptr<KdTreeNode<Scalar, Dimension>> right_ptr_ = nullptr;
    std::vector<int32_t> point_indices_;
};

/* Class KD-Tree Definition. */
template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::Construct(const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                                              const std::vector<int32_t> &point_indices,
                                              const int32_t specified_axis,
                                              std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr) {
    RETURN_IF(points.empty() || point_indices.empty());

    if (node_ptr == nullptr) {
        node_ptr = std::make_unique<KdTreeNode<Scalar, Dimension>>();
        node_ptr->axis() = specified_axis;
        node_ptr->divider() = points[point_indices.front()][specified_axis];
        node_ptr->left_ptr() = nullptr;
        node_ptr->right_ptr() = nullptr;
        node_ptr->point_indices() = point_indices;
    }

    RETURN_IF(static_cast<int32_t>(point_indices.size()) <= options_.kMaxNumberOfPointsInLeafNode);

    // If this node is not leaf, clear its points.
    node_ptr->point_indices().clear();

    // Sort indices.
    std::vector<int32_t> sorted_indices = point_indices;
    SlamOperation::ArgSortVector(points, specified_axis, sorted_indices);

    // Compute divider value in current node.
    const uint32_t left_index = (sorted_indices.size() / 2) - 1;
    const uint32_t right_index = left_index + 1;
    const Scalar left_value = points[sorted_indices[left_index]][specified_axis];
    const Scalar right_value = points[sorted_indices[right_index]][specified_axis];
    node_ptr->divider() = static_cast<Scalar>(0.5) * (left_value + right_value);


    // Divide point indices. Move them to left and right child node.
    std::vector<int32_t> left_indices;
    std::vector<int32_t> right_indices;
    for (uint32_t i = 0; i < right_index; ++i) {
        left_indices.emplace_back(sorted_indices[i]);
    }
    for (uint32_t i = right_index; i < sorted_indices.size(); ++i) {
        right_indices.emplace_back(sorted_indices[i]);
    }

    // Decide next dimension.
    const int32_t next_axis = GetAxisWithMaxRange(sorted_indices, points);

    Construct(points, left_indices, next_axis, node_ptr->left_ptr());
    Construct(points, right_indices, next_axis, node_ptr->right_ptr());
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::ExtractAllPoints(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                                                     const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                                                     std::vector<int32_t> &point_indices) {
    RETURN_IF(node_ptr == nullptr || points.empty());

    if (node_ptr->IsLeafNode()) {
        // Add all points in this node.
        for (const auto &index : node_ptr->point_indices()) {
            point_indices.emplace_back(index);
        }
    }

    ExtractAllPoints(node_ptr->left_ptr(), points, point_indices);
    ExtractAllPoints(node_ptr->right_ptr(), points, point_indices);
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::SearchKnn(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                                              const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                                              const Eigen::Matrix<Scalar, Dimension, 1> &target_point,
                                              const uint32_t target_number,
                                              std::map<float, int32_t> &residual_index_of_points) {
    RETURN_IF(node_ptr == nullptr || points.empty() || target_number == 0);

    if (node_ptr->IsLeafNode()) {
        // Add all points in this node.
        for (const auto &index : node_ptr->point_indices()) {
            const Scalar distance = (target_point - points[index]).norm();
            residual_index_of_points.insert(std::make_pair(distance, index));

            // Control num of searched points.
            while (residual_index_of_points.size() > target_number) {
                residual_index_of_points.erase(std::prev(residual_index_of_points.end()));
            }
        }
    }

    // Recursion search.
    // if (target_point(node_ptr->axis()) < node_ptr->divider()) {
        SearchKnn(node_ptr->left_ptr(), points, target_point, target_number, residual_index_of_points);
        // if (std::abs(target_point(node_ptr->axis()) - node_ptr->divider()) < residual_index_of_points.rbegin()->first) {
            SearchKnn(node_ptr->right_ptr(), points, target_point, target_number, residual_index_of_points);
        // }
    // } else {
    //     SearchKnn(node_ptr->right_ptr(), points, target_point, target_number, residual_index_of_points);
    //     if (std::abs(target_point(node_ptr->axis()) - node_ptr->divider()) < residual_index_of_points.rbegin()->first) {
    //         SearchKnn(node_ptr->left_ptr(), points, target_point, target_number, residual_index_of_points);
    //     }
    // }
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::SearchRadius(const std::unique_ptr<KdTreeNode<Scalar, Dimension>> &node_ptr,
                                                 const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points,
                                                 const Eigen::Matrix<Scalar, Dimension, 1> &target_point,
                                                 const Scalar max_radius,
                                                 std::map<float, int32_t> &residual_index_of_points) {
    RETURN_IF(node_ptr == nullptr || points.empty());

    if (node_ptr->IsLeafNode()) {
        // Add all points in this node.
        for (const auto &index : node_ptr->point_indices()) {
            const Scalar distance = (target_point - points[index]).norm();
            residual_index_of_points.insert(std::make_pair(distance, index));

            // Control num of searched points.
            while (!residual_index_of_points.empty()) {
                if (residual_index_of_points.rbegin()->first > max_radius) {
                    residual_index_of_points.erase(std::prev(residual_index_of_points.end()));
                } else {
                    break;
                }
            }
        }
    }

    // Recursion search.
    // if (target_point(node_ptr->axis()) < node_ptr->divider()) {
        SearchRadius(node_ptr->left_ptr(), points, target_point, max_radius, residual_index_of_points);
        // if (std::abs(target_point(node_ptr->axis()) - node_ptr->divider()) < residual_index_of_points.rbegin()->first) {
            SearchRadius(node_ptr->right_ptr(), points, target_point, max_radius, residual_index_of_points);
        // }
    // } else {
    //     SearchRadius(node_ptr->right_ptr(), points, target_point, max_radius, residual_index_of_points);
    //     if (std::abs(target_point(node_ptr->axis()) - node_ptr->divider()) < residual_index_of_points.rbegin()->first) {
    //         SearchRadius(node_ptr->left_ptr(), points, target_point, max_radius, residual_index_of_points);
    //     }
    // }
}

template <typename Scalar, int32_t Dimension>
int32_t KdTreeNode<Scalar, Dimension>::GetAxisWithMaxRange(const std::vector<int32_t> &point_indices,
                                                           const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> &points) {
    if (point_indices.empty() || points.empty()) {
        return 0;
    }

    const int32_t axis_bound = static_cast<int32_t>(points.front().rows());
    int32_t axis = 0;
    Scalar max_range = 0.0f;

    for (int32_t i = 0; i < axis_bound; ++i) {
        Scalar max_value = points[point_indices.front()](i);
        Scalar min_value = max_value;

        for (const auto &index : point_indices) {
            max_value = std::max(points[index](i), max_value);
            min_value = std::min(points[index](i), min_value);
        }

        const Scalar range = max_value - min_value;
        if (range > max_range) {
            max_range = range;
            axis = i;
        }
    }

    return axis;
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::Information() {
    ReportInfo("[KdTreeNode] Axis [" << axis_ << "], Value [" << divider_ << "], [" << point_indices_.size() << "] points." <<
        " Left child " << LogPtr(left_ptr_.get()) << ", Right child " << LogPtr(right_ptr_.get()));
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::InformationRecursion() {
    Information();
    if (left_ptr_ != nullptr) {
        left_ptr_->InformationRecursion();
    }
    if (right_ptr_ != nullptr) {
        right_ptr_->InformationRecursion();
    }
}

}

#endif // end of _SLAM_UTILITY_CIRCULAR_BUFFER_H_
