#ifndef _SLAM_UTILITY_KD_TREE_H_
#define _SLAM_UTILITY_KD_TREE_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "algorithm"
#include "cmath"
#include "memory"
#include "map"
#include "vector"
#include "type_traits"

namespace slam_utility {

/**
 * @brief Configuration options for KD-Tree
 */
struct OptionsOfKdTree {
    // Optimal leaf node size (balance between recursion depth and search speed)
    int32_t kMaxNumberOfPointsInLeafNode = 16;
};

/**
 * @brief KD-Tree implementation for efficient nearest neighbor search
 * @tparam Scalar Floating point type (float/double)
 * @tparam Dimension Point dimension (e.g., 3 for 3D points)
 */
template <typename Scalar, int32_t Dimension>
class KdTreeNode {
public:
    using Ptr = std::unique_ptr<KdTreeNode>;
    using PointType = Eigen::Matrix<Scalar, Dimension, 1>;

    // Compile-time validation for template parameters
    static_assert(Dimension > 0, "Dimension must be a positive integer");
    static_assert(std::is_floating_point_v<Scalar>, "Scalar must be floating point type (float/double)");

    KdTreeNode() = default;
    virtual ~KdTreeNode() = default;

    /**
     * @brief Build KD-Tree recursively (root node entry point)
     * @param points Input point cloud data
     * @param point_indices Indices of points to include in this tree
     */
    void Construct(const std::vector<PointType>& points, const std::vector<int32_t>& point_indices);

    /**
     * @brief Extract all point indices stored in the tree (recursive)
     * @param point_indices Output vector to store all point indices
     */
    void ExtractAllPoints(std::vector<int32_t>& point_indices) const;

    /**
     * @brief Search K nearest neighbors for target point
     * @param points Input point cloud data
     * @param target_point Query point for nearest neighbor search
     * @param target_number Number of nearest neighbors to find
     * @param residual_index_of_points Output: map of (squared distance, point index)
     */
    void SearchKnn(const std::vector<PointType>& points,
                   const PointType& target_point,
                   const uint32_t target_number,
                   std::multimap<float, int32_t>& residual_index_of_points) const;

    /**
     * @brief Search all points within specified radius from target point
     * @param points Input point cloud data
     * @param target_point Query point for radius search
     * @param max_radius Maximum search radius (linear distance)
     * @param residual_index_of_points Output: map of (squared distance, point index)
     */
    void SearchRadius(const std::vector<PointType>& points,
                      const PointType& target_point,
                      const Scalar max_radius,
                      std::multimap<float, int32_t>& residual_index_of_points) const;

    /**
     * @brief Search all points within axis-aligned bounding box (cube)
     * @param points Input point cloud data
     * @param min_value Minimum bounds of cube (per dimension)
     * @param max_value Maximum bounds of cube (per dimension)
     * @param residual_index_of_points Output: map of (squared distance to cube center, point index)
     */
    void SearchCube(const std::vector<PointType>& points,
                    const PointType& min_value,
                    const PointType& max_value,
                    std::multimap<float, int32_t>& residual_index_of_points) const;

    /**
     * @brief Get tree depth (maximum depth of left/right subtrees + 1)
     * @return Actual depth of the tree
     */
    int32_t GetDepth() const;

    /**
     * @brief Print single node information (for debugging)
     */
    void Information() const;

    /**
     * @brief Print all node information recursively (for debugging)
     */
    void InformationRecursion() const;

    // Const accessors (encapsulation - prevent external modification)
    int32_t dimension() const { return dimension_; }
    Scalar divider() const { return divider_; }
    const std::vector<int32_t>& point_indices() const { return point_indices_; }
    const Ptr& left_ptr() const { return left_ptr_; }
    const Ptr& right_ptr() const { return right_ptr_; }

private:
    /**
     * @brief Find axis with maximum value range (for optimal split)
     * @param point_indices Indices of points to analyze
     * @param points Input point cloud data
     * @return Axis index with maximum range
     */
    int32_t GetAxisWithMaxRange(const std::vector<int32_t>& point_indices,
                                const std::vector<PointType>& points) const;

    /**
     * @brief Check if current node is leaf node (contains points)
     * @return True if leaf node, false otherwise
     */
    bool IsLeafNode() const { return !point_indices_.empty(); }

    // Member variables
    OptionsOfKdTree options_;
    int32_t dimension_ = 0;                // Split axis for current node
    Scalar divider_ = static_cast<Scalar>(0); // Split value on current axis
    Ptr left_ptr_ = nullptr;               // Left subtree (points <= divider)
    Ptr right_ptr_ = nullptr;              // Right subtree (points > divider)
    std::vector<int32_t> point_indices_;   // Point indices (only for leaf nodes)
};

// -----------------------------------------------------------------------------
// KD-Tree Implementation
// -----------------------------------------------------------------------------

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::Construct(const std::vector<PointType>& points,
                                              const std::vector<int32_t>& point_indices) {
    // Input validation
    RETURN_IF(points.empty() || point_indices.empty());

    // Validate point indices are within bounds
    for (const auto& idx : point_indices) {
        RETURN_IF(idx < 0 || static_cast<size_t>(idx) >= points.size());
    }

    // Initialize current node with point indices
    this->point_indices_ = point_indices;
    const int32_t axis = GetAxisWithMaxRange(point_indices, points);
    this->dimension_ = axis;
    this->divider_ = points[point_indices.front()][axis];

    // Termination condition: leaf node (contains <= max leaf points)
    if (static_cast<int32_t>(point_indices.size()) <= options_.kMaxNumberOfPointsInLeafNode) {
        return;
    }

    // Non-leaf node: clear point indices (only leaf nodes store points)
    this->point_indices_.clear();

    // Sort indices by split axis (using external sorting function)
    std::vector<int32_t> sorted_indices = point_indices;
    SlamOperation::ArgSortVector(points, axis, sorted_indices);

    // Calculate optimal split value (midpoint of two middle points)
    const uint32_t left_index = (sorted_indices.size() / 2) - 1;
    const uint32_t right_index = left_index + 1;
    const Scalar left_value = points[sorted_indices[left_index]][axis];
    const Scalar right_value = points[sorted_indices[right_index]][axis];
    this->divider_ = static_cast<Scalar>(0.5) * (left_value + right_value);

    // Split indices into left/right subsets
    std::vector<int32_t> left_indices, right_indices;
    for (uint32_t i = 0; i < right_index; ++i) {
        left_indices.emplace_back(sorted_indices[i]);
    }
    for (uint32_t i = right_index; i < sorted_indices.size(); ++i) {
        right_indices.emplace_back(sorted_indices[i]);
    }

    // Recursively build left subtree
    if (!left_indices.empty()) {
        this->left_ptr_ = std::make_unique<KdTreeNode<Scalar, Dimension>>();
        this->left_ptr_->Construct(points, left_indices);
    }

    // Recursively build right subtree
    if (!right_indices.empty()) {
        this->right_ptr_ = std::make_unique<KdTreeNode<Scalar, Dimension>>();
        this->right_ptr_->Construct(points, right_indices);
    }
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::ExtractAllPoints(std::vector<int32_t>& point_indices) const {
    // Add points from current leaf node
    if (IsLeafNode()) {
        for (const auto& index : point_indices_) {
            point_indices.emplace_back(index);
        }
    }

    // Recursively extract points from subtrees
    if (left_ptr_) {
        left_ptr_->ExtractAllPoints(point_indices);
    }
    if (right_ptr_) {
        right_ptr_->ExtractAllPoints(point_indices);
    }
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::SearchKnn(const std::vector<PointType>& points,
                                              const PointType& target_point,
                                              const uint32_t target_number,
                                              std::multimap<float, int32_t>& residual_index_of_points) const {
    // Input validation
    RETURN_IF(points.empty() || target_number == 0);

    // Leaf node: process all points in node
    if (IsLeafNode()) {
        for (const auto& index : point_indices_) {
            // Skip invalid indices
            CONTINUE_IF(index < 0 || static_cast<size_t>(index) >= points.size());

            // Calculate squared distance (avoids expensive sqrt for comparison)
            const Scalar dist_sq = (target_point - points[index]).squaredNorm();
            residual_index_of_points.insert({static_cast<float>(dist_sq), index});
        }

        // Keep only K nearest neighbors (smallest squared distances)
        while (residual_index_of_points.size() > target_number) {
            residual_index_of_points.erase(std::prev(residual_index_of_points.end()));
        }
        return;
    }

    // Recursive search strategy: first search the closer subtree
    if (target_point(dimension_) < divider_) {
        // Search left subtree first
        if (left_ptr_) {
            left_ptr_->SearchKnn(points, target_point, target_number, residual_index_of_points);
        }

        // Pruning check: search right subtree if necessary
        if (!residual_index_of_points.empty()) {
            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (axis_dist_sq < residual_index_of_points.rbegin()->first) {
                if (right_ptr_) {
                    right_ptr_->SearchKnn(points, target_point, target_number, residual_index_of_points);
                }
            }
        }
    } else {
        // Search right subtree first
        if (right_ptr_) {
            right_ptr_->SearchKnn(points, target_point, target_number, residual_index_of_points);
        }

        // Pruning check: search left subtree if necessary
        if (!residual_index_of_points.empty()) {
            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (axis_dist_sq < residual_index_of_points.rbegin()->first) {
                if (left_ptr_) {
                    left_ptr_->SearchKnn(points, target_point, target_number, residual_index_of_points);
                }
            }
        }
    }
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::SearchRadius(const std::vector<PointType>& points,
                                                 const PointType& target_point,
                                                 const Scalar max_radius,
                                                 std::multimap<float, int32_t>& residual_index_of_points) const {
    // Input validation
    RETURN_IF(points.empty() || max_radius < static_cast<Scalar>(0));

    // Calculate squared radius (for consistent distance comparison)
    const Scalar max_radius_sq = max_radius * max_radius;

    // Leaf node: process all points in node
    if (IsLeafNode()) {
        for (const auto& index : point_indices_) {
            // Skip invalid indices
            CONTINUE_IF(index < 0 || static_cast<size_t>(index) >= points.size());

            // Calculate squared distance to avoid sqrt
            const Scalar dist_sq = (target_point - points[index]).squaredNorm();

            // Check if point is within radius
            CONTINUE_IF(dist_sq > max_radius_sq);

            residual_index_of_points.insert({static_cast<float>(dist_sq), index});
        }
        return;
    }

    // Recursive search strategy with pruning
    if (target_point(dimension_) < divider_) {
        // Search left subtree first
        if (left_ptr_) {
            left_ptr_->SearchRadius(points, target_point, max_radius, residual_index_of_points);
        }

        // Pruning check: search right subtree if boundary intersects with search radius
        const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
        if (axis_dist_sq < max_radius_sq) {
            if (right_ptr_) {
                right_ptr_->SearchRadius(points, target_point, max_radius, residual_index_of_points);
            }
        }
    } else {
        // Search right subtree first
        if (right_ptr_) {
            right_ptr_->SearchRadius(points, target_point, max_radius, residual_index_of_points);
        }

        // Pruning check: search left subtree if boundary intersects with search radius
        const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
        if (axis_dist_sq < max_radius_sq) {
            if (left_ptr_) {
                left_ptr_->SearchRadius(points, target_point, max_radius, residual_index_of_points);
            }
        }
    }
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::SearchCube(const std::vector<PointType>& points,
                                               const PointType& min_value,
                                               const PointType& max_value,
                                               std::multimap<float, int32_t>& residual_index_of_points) const {
    // Input validation
    RETURN_IF(points.empty());

    // Validate cube bounds (min <= max for all dimensions)
    for (uint32_t dim = 0; dim < Dimension; ++dim) {
        RETURN_IF(min_value(dim) > max_value(dim));
    }

    // Calculate cube center (for distance calculation)
    const PointType cube_center = static_cast<Scalar>(0.5) * (min_value + max_value);

    // Leaf node: process all points in node
    if (IsLeafNode()) {
        for (const auto& index : point_indices_) {
            // Skip invalid indices
            CONTINUE_IF(index < 0 || static_cast<size_t>(index) >= points.size());

            // Check if point is inside axis-aligned cube
            bool is_inside = true;
            for (uint32_t dim = 0; dim < Dimension; ++dim) {
                if (points[index](dim) < min_value(dim) || points[index](dim) > max_value(dim)) {
                    is_inside = false;
                    break;
                }
            }
            CONTINUE_IF(!is_inside);

            // Calculate squared distance to cube center
            const Scalar dist_sq = (points[index] - cube_center).squaredNorm();
            residual_index_of_points.insert({static_cast<float>(dist_sq), index});
        }
        return;
    }

    // Recursive search strategy: check intersection with cube
    if (min_value(dimension_) < divider_) {
        // Search left subtree if cube intersects left partition
        if (left_ptr_) {
            left_ptr_->SearchCube(points, min_value, max_value, residual_index_of_points);
        }
    }

    if (max_value(dimension_) > divider_) {
        // Search right subtree if cube intersects right partition
        if (right_ptr_) {
            right_ptr_->SearchCube(points, min_value, max_value, residual_index_of_points);
        }
    }
}

template <typename Scalar, int32_t Dimension>
int32_t KdTreeNode<Scalar, Dimension>::GetAxisWithMaxRange(const std::vector<int32_t>& point_indices,
                                                           const std::vector<PointType>& points) const {
    // Return default axis if no points to analyze
    if (point_indices.empty() || points.empty()) {
        return 0;
    }

    const int32_t axis_bound = static_cast<int32_t>(points.front().rows());
    int32_t best_axis = 0;
    Scalar max_range = static_cast<Scalar>(0);

    // Find axis with maximum value range (for optimal splitting)
    for (int32_t i = 0; i < axis_bound; ++i) {
        Scalar max_val = points[point_indices.front()](i);
        Scalar min_val = max_val;

        // Calculate min/max values for current axis
        for (const auto& index : point_indices) {
            // Skip invalid indices
            if (index < 0 || static_cast<size_t>(index) >= points.size()) {
                continue;
            }
            max_val = std::max(points[index](i), max_val);
            min_val = std::min(points[index](i), min_val);
        }

        // Update best axis if current axis has larger range
        const Scalar range = max_val - min_val;
        if (range > max_range) {
            max_range = range;
            best_axis = i;
        }
    }

    return best_axis;
}

template <typename Scalar, int32_t Dimension>
int32_t KdTreeNode<Scalar, Dimension>::GetDepth() const {
    // Empty node has depth 0
    if (!left_ptr_ && !right_ptr_) {
        return IsLeafNode() ? 1 : 0;
    }

    // Calculate depth of subtrees
    const int32_t left_depth = left_ptr_ ? left_ptr_->GetDepth() : 0;
    const int32_t right_depth = right_ptr_ ? right_ptr_->GetDepth() : 0;

    // Tree depth = max subtree depth + 1 (current node)
    return std::max(left_depth, right_depth) + 1;
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::Information() const {
    // Debug information (keep original logging logic)
    ReportInfo("[KdTreeNode] Axis [" << dimension_ << "], Value [" << divider_ << "], ["
              << point_indices_.size() << "] points." << " Left child "
              << LogPtr(left_ptr_.get()) << ", Right child " << LogPtr(right_ptr_.get()));
}

template <typename Scalar, int32_t Dimension>
void KdTreeNode<Scalar, Dimension>::InformationRecursion() const {
    // Recursive debug information
    Information();
    if (left_ptr_) {
        left_ptr_->InformationRecursion();
    }
    if (right_ptr_) {
        right_ptr_->InformationRecursion();
    }
}

} // namespace slam_utility

#endif // _SLAM_UTILITY_KD_TREE_H_
