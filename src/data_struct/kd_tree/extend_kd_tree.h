#ifndef _SLAM_UTILITY_EXTEND_KD_TREE_H_
#define _SLAM_UTILITY_EXTEND_KD_TREE_H_

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "algorithm"
#include "cmath"
#include "memory"
#include "map"
#include "vector"
#include "functional"

namespace slam_utility {

/**
 * @brief Configuration options for Extend KD-Tree
 */
struct OptionsOfExtendKdTree {
    int32_t kMaxNumberOfPointsInLeafNode = 16;
};

/**
 * @brief Extend KD-Tree node implementation that supports arbitrary data types
 * through a lambda accessor.
 * @tparam Scalar Floating point type (float/double)
 * @tparam Dimension Point dimension
 */
template <typename Scalar, int32_t Dimension>
class ExtendKdTreeNode {
public:
    using Ptr = std::unique_ptr<ExtendKdTreeNode>;
    using PointType = Eigen::Matrix<Scalar, Dimension, 1>;
    // Lambda accessor: given an index, return the coordinate point
    using PointAccessor = std::function<PointType(int32_t)>;

    ExtendKdTreeNode() = default;
    virtual ~ExtendKdTreeNode() = default;

    /**
     * @brief Build KD-Tree recursively
     * @param point_indices Indices of points to include
     * @param accessor Lambda to get point from index
     */
    void Construct(const std::vector<int32_t>& point_indices, const PointAccessor& accessor) {
        RETURN_IF(point_indices.empty());

        this->point_indices_ = point_indices;
        const int32_t axis = GetAxisWithMaxRange(point_indices, accessor);
        this->dimension_ = axis;
        this->divider_ = accessor(point_indices.front())[axis];

        if (static_cast<int32_t>(point_indices.size()) <= options_.kMaxNumberOfPointsInLeafNode) {
            return;
        }

        this->point_indices_.clear();

        std::vector<int32_t> sorted_indices = point_indices;
        std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int32_t idx_1, int32_t idx_2) {
            return accessor(idx_1)[axis] < accessor(idx_2)[axis];
        });

        const uint32_t left_index = (sorted_indices.size() / 2) - 1;
        const uint32_t right_index = left_index + 1;
        const Scalar left_value = accessor(sorted_indices[left_index])[axis];
        const Scalar right_value = accessor(sorted_indices[right_index])[axis];
        this->divider_ = static_cast<Scalar>(0.5) * (left_value + right_value);

        std::vector<int32_t> left_indices, right_indices;
        for (uint32_t i = 0; i < right_index; ++i) {
            left_indices.emplace_back(sorted_indices[i]);
        }
        for (uint32_t i = right_index; i < sorted_indices.size(); ++i) {
            right_indices.emplace_back(sorted_indices[i]);
        }

        if (!left_indices.empty()) {
            this->left_ptr_ = std::make_unique<ExtendKdTreeNode<Scalar, Dimension>>();
            this->left_ptr_->Construct(left_indices, accessor);
        }

        if (!right_indices.empty()) {
            this->right_ptr_ = std::make_unique<ExtendKdTreeNode<Scalar, Dimension>>();
            this->right_ptr_->Construct(right_indices, accessor);
        }
    }

    /**
     * @brief Search K nearest neighbors
     */
    void SearchKnn(const PointType& target_point,
                   const uint32_t target_number,
                   const PointAccessor& accessor,
                   std::multimap<float, int32_t>& residual_index_of_points) const {
        RETURN_IF(target_number == 0);

        if (IsLeafNode()) {
            for (const auto& index : point_indices_) {
                const Scalar dist_sq = (target_point - accessor(index)).squaredNorm();
                residual_index_of_points.insert({static_cast<float>(dist_sq), index});
            }

            while (residual_index_of_points.size() > target_number) {
                residual_index_of_points.erase(std::prev(residual_index_of_points.end()));
            }
            return;
        }

        if (target_point(dimension_) < divider_) {
            if (left_ptr_) {
                left_ptr_->SearchKnn(target_point, target_number, accessor, residual_index_of_points);
            }

            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (residual_index_of_points.size() < target_number || axis_dist_sq < residual_index_of_points.rbegin()->first) {
                if (right_ptr_) {
                    right_ptr_->SearchKnn(target_point, target_number, accessor, residual_index_of_points);
                }
            }
        } else {
            if (right_ptr_) {
                right_ptr_->SearchKnn(target_point, target_number, accessor, residual_index_of_points);
            }

            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (residual_index_of_points.size() < target_number || axis_dist_sq < residual_index_of_points.rbegin()->first) {
                if (left_ptr_) {
                    left_ptr_->SearchKnn(target_point, target_number, accessor, residual_index_of_points);
                }
            }
        }
    }

    /**
     * @brief Search points within radius
     */
    void SearchRadius(const PointType& target_point,
                      const Scalar max_radius,
                      const PointAccessor& accessor,
                      std::multimap<float, int32_t>& residual_index_of_points) const {
        RETURN_IF(max_radius < static_cast<Scalar>(0));
        const Scalar max_radius_sq = max_radius * max_radius;

        if (IsLeafNode()) {
            for (const auto& index : point_indices_) {
                const Scalar dist_sq = (target_point - accessor(index)).squaredNorm();
                CONTINUE_IF(dist_sq > max_radius_sq);
                residual_index_of_points.insert({static_cast<float>(dist_sq), index});
            }
            return;
        }

        if (target_point(dimension_) < divider_) {
            if (left_ptr_) {
                left_ptr_->SearchRadius(target_point, max_radius, accessor, residual_index_of_points);
            }
            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (axis_dist_sq < max_radius_sq) {
                if (right_ptr_) {
                    right_ptr_->SearchRadius(target_point, max_radius, accessor, residual_index_of_points);
                }
            }
        } else {
            if (right_ptr_) {
                right_ptr_->SearchRadius(target_point, max_radius, accessor, residual_index_of_points);
            }
            const Scalar axis_dist_sq = std::pow(target_point(dimension_) - divider_, 2);
            if (axis_dist_sq < max_radius_sq) {
                if (left_ptr_) {
                    left_ptr_->SearchRadius(target_point, max_radius, accessor, residual_index_of_points);
                }
            }
        }
    }

    /**
     * @brief Search points within cube
     */
    void SearchCube(const PointType& min_value,
                    const PointType& max_value,
                    const PointAccessor& accessor,
                    std::multimap<float, int32_t>& residual_index_of_points) const {
        for (uint32_t dim = 0; dim < Dimension; ++dim) {
            RETURN_IF(min_value(dim) > max_value(dim));
        }

        const PointType cube_center = static_cast<Scalar>(0.5) * (min_value + max_value);

        if (IsLeafNode()) {
            for (const auto& index : point_indices_) {
                const PointType p = accessor(index);
                bool is_inside = true;
                for (uint32_t dim = 0; dim < Dimension; ++dim) {
                    if (p(dim) < min_value(dim) || p(dim) > max_value(dim)) {
                        is_inside = false;
                        break;
                    }
                }
                CONTINUE_IF(!is_inside);

                const Scalar dist_sq = (p - cube_center).squaredNorm();
                residual_index_of_points.insert({static_cast<float>(dist_sq), index});
            }
            return;
        }

        if (min_value(dimension_) < divider_) {
            if (left_ptr_) left_ptr_->SearchCube(min_value, max_value, accessor, residual_index_of_points);
        }
        if (max_value(dimension_) > divider_) {
            if (right_ptr_) right_ptr_->SearchCube(min_value, max_value, accessor, residual_index_of_points);
        }
    }

    void ExtractAllPoints(std::vector<int32_t>& point_indices) const {
        if (IsLeafNode()) {
            for (const auto& index : point_indices_) {
                point_indices.emplace_back(index);
            }
        }
        if (left_ptr_) left_ptr_->ExtractAllPoints(point_indices);
        if (right_ptr_) right_ptr_->ExtractAllPoints(point_indices);
    }

    int32_t GetDepth() const {
        if (!left_ptr_ && !right_ptr_) return IsLeafNode() ? 1 : 0;
        const int32_t left_depth = left_ptr_ ? left_ptr_->GetDepth() : 0;
        const int32_t right_depth = right_ptr_ ? right_ptr_->GetDepth() : 0;
        return std::max(left_depth, right_depth) + 1;
    }

    void Information() const {
        ReportInfo("[ExtendKdTreeNode] Axis [" << dimension_ << "], Value [" << divider_ << "], ["
                  << point_indices_.size() << "] points." << " Left child "
                  << LogPtr(left_ptr_.get()) << ", Right child " << LogPtr(right_ptr_.get()));
    }

    void InformationRecursion() const {
        Information();
        if (left_ptr_) left_ptr_->InformationRecursion();
        if (right_ptr_) right_ptr_->InformationRecursion();
    }

private:
    int32_t GetAxisWithMaxRange(const std::vector<int32_t>& point_indices,
                                const PointAccessor& accessor) const {
        if (point_indices.empty()) return 0;

        int32_t best_axis = 0;
        Scalar max_range = static_cast<Scalar>(0);

        for (int32_t i = 0; i < Dimension; ++i) {
            Scalar max_val = accessor(point_indices.front())[i];
            Scalar min_val = max_val;

            for (const auto& index : point_indices) {
                const Scalar val = accessor(index)[i];
                max_val = std::max(val, max_val);
                min_val = std::min(val, min_val);
            }

            const Scalar range = max_val - min_val;
            if (range > max_range) {
                max_range = range;
                best_axis = i;
            }
        }
        return best_axis;
    }

    bool IsLeafNode() const { return !point_indices_.empty(); }

    OptionsOfExtendKdTree options_;
    int32_t dimension_ = 0;
    Scalar divider_ = static_cast<Scalar>(0);
    Ptr left_ptr_ = nullptr;
    Ptr right_ptr_ = nullptr;
    std::vector<int32_t> point_indices_;
};

} // namespace slam_utility

#endif
