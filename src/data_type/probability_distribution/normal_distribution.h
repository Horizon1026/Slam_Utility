#ifndef _NORMAL_DISTRIBUTION_H_
#define _NORMAL_DISTRIBUTION_H_

#include "basic_type.h"
#include "slam_basic_math.h"
#include "slam_operations.h"

namespace slam_utility {

/* Class NormalDistribution Declaration. */
template <int32_t kDimension>
class NormalDistribution {

public:
    struct AdvancedParameters {
        TMat<float, kDimension, kDimension> sqrt_inv_covariance_ = TMat<float, kDimension, kDimension>::Zero();
        TMat<float, kDimension, kDimension> inv_covariance_ = TMat<float, kDimension, kDimension>::Zero();
    };

public:
    NormalDistribution() = default;
    virtual ~NormalDistribution() = default;

    // Operations.
    void Reset();
    bool IncrementallyFitDistribution(const TVec<float, kDimension> &point);
    bool DirectlyFitDistribution(const std::vector<TVec<float, kDimension>> &points);
    void SynchronizeToAdvancedParameters();
    float ProbabilityDensityFunction(const TVec<float, kDimension> &point) const;

    // Define operators.
    bool operator==(const NormalDistribution &other) const { return mid_point_ == other.mid_point_ && covariance_ == other.covariance_; }
    bool operator!=(const NormalDistribution &other) const { return mid_point_ != other.mid_point_ || covariance_ != other.covariance_; }

    // Reference for member variables.
    TVec<float, kDimension> &mid_point() { return mid_point_; }
    TMat<float, kDimension, kDimension> &covariance() { return covariance_; }
    uint32_t &num_of_points() { return num_of_points_; }
    // Const reference for member variables.
    const TVec<float, kDimension> &mid_point() const { return mid_point_; }
    const TMat<float, kDimension, kDimension> &covariance() const { return covariance_; }
    const TMat<float, kDimension, kDimension> &sqrt_inv_covariance() const { return advanced_params_.sqrt_inv_covariance_; }
    const TMat<float, kDimension, kDimension> &inv_covariance() const { return advanced_params_.inv_covariance_; }
    const uint32_t &num_of_points() const { return num_of_points_; }

private:
    TVec<float, kDimension> mid_point_ = TVec<float, kDimension>::Zero();
    TMat<float, kDimension, kDimension> covariance_ = TMat<float, kDimension, kDimension>::Zero();
    uint32_t num_of_points_ = 0;

    AdvancedParameters advanced_params_;
};

/* Class NormalDistribution Definition. */
template <int32_t kDimension>
void NormalDistribution<kDimension>::Reset() {
    mid_point_.setZero();
    covariance_.setZero();
    num_of_points_ = 0;
}

template <int32_t kDimension>
bool NormalDistribution<kDimension>::IncrementallyFitDistribution(const TVec<float, kDimension> &point) {
    // Initialize normal distribution.
    if (num_of_points_ == 0) {
        mid_point_ = point;
        covariance_.setZero();
        ++num_of_points_;
        return true;
    }
    // Incremental update of plane model.
    const float old_weight = static_cast<float>(num_of_points_);
    const float new_weight = old_weight + 1.0f;
    // Incremental update of mid point and covariance matrix.
    // new_miu = N / (N + 1) * miu + 1 / (N + 1) * x = miu + 1 / (N + 1) * (x - miu)
    // new_sigma = N / (N + 1) * sigma + 1 / (N + 1) * (x - miu) * (x - new_miu).T
    const Vec3 new_mid_point = mid_point_ + (point - mid_point_) / new_weight;
    const Mat3 new_covariance = covariance_ * old_weight / new_weight + (point - mid_point_) * (point - new_mid_point).transpose() / new_weight;
    // Update parameters of normal distribution.
    mid_point_ = new_mid_point;
    covariance_ = new_covariance;
    ++num_of_points_;
    return true;
}

template <int32_t kDimension>
bool NormalDistribution<kDimension>::DirectlyFitDistribution(const std::vector<TVec<float, kDimension>> &points) {
    RETURN_FALSE_IF(points.empty());
    num_of_points_ = points.size();

    // Compute mid point.
    mid_point_.setZero();
    Vec3 summary = Vec3::Zero();
    for (const Vec3 &point: points) {
        summary += point;
    }
    mid_point_ = summary / static_cast<float>(num_of_points_);

    // Compute covariance matrix.
    covariance_.setZero();
    for (const Vec3 &point: points) {
        const Vec3 p = point - mid_point_;
        covariance_ += p * p.transpose();
    }
    covariance_ /= static_cast<float>(num_of_points_);
    return true;
}

template <int32_t kDimension>
void NormalDistribution<kDimension>::SynchronizeToAdvancedParameters() {
    const Eigen::LLT<TMat<float, kDimension, kDimension>> cov_llt(covariance_ + TMat<float, kDimension, kDimension>::Identity() * 1e-4f);
    advanced_params_.sqrt_inv_covariance_ = cov_llt.matrixL().solve(TMat<float, kDimension, kDimension>::Identity());
    advanced_params_.inv_covariance_ = advanced_params_.sqrt_inv_covariance_ * advanced_params_.sqrt_inv_covariance_.transpose();
}

template <int32_t kDimension>
float NormalDistribution<kDimension>::ProbabilityDensityFunction(const TVec<float, kDimension> &point) const {
    const TVec<float, kDimension> diff = point - mid_point_;
    return std::exp(-0.5 * diff.transpose() * advanced_params_.inv_covariance_ * diff) / std::sqrt(2.0f * kPai * covariance_.determinant());
}

}  // namespace slam_utility

#endif  // end of _NORMAL_DISTRIBUTION_H_
