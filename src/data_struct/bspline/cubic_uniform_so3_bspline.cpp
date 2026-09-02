#include "cubic_uniform_so3_bspline.h"
#include "algorithm"
#include "cmath"
#include "slam_operations.h"

namespace slam_utility {

bool CubicUniformSO3BSpline::Fit(const std::vector<double> &all_time_stamps_s, const std::vector<TQuat<double>> &all_orientations) {
    RETURN_FALSE_IF(all_time_stamps_s.size() != all_orientations.size() || all_orientations.size() < 4);

    std::vector<TQuat<double>> normalized_orientations;
    normalized_orientations.reserve(all_orientations.size());
    for (uint32_t i = 0; i < all_orientations.size(); ++i) {
        if (!std::isfinite(all_orientations[i].w()) || !all_orientations[i].vec().allFinite() || all_orientations[i].norm() < 1e-12) {
            return false;
        }
        TQuat<double> orientation = all_orientations[i].normalized();
        // q and -q represent the same rotation. Keep consecutive samples in the
        // same hemisphere so their logarithms vary continuously.
        if (!normalized_orientations.empty() && normalized_orientations.back().coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() = -orientation.coeffs();
        }
        normalized_orientations.emplace_back(orientation);
    }

    const double time_interval_s = all_time_stamps_s[1] - all_time_stamps_s[0];
    if (!std::isfinite(time_interval_s) || time_interval_s <= 0.0) {
        return false;
    }
    for (uint32_t i = 2; i < all_time_stamps_s.size(); ++i) {
        const double interval_s = all_time_stamps_s[i] - all_time_stamps_s[i - 1];
        const double tolerance = 1e-9 * std::max(1.0, std::fabs(time_interval_s));
        RETURN_FALSE_IF(!std::isfinite(all_time_stamps_s[i]) || std::fabs(interval_s - time_interval_s) > tolerance);
    }

    CubicUniformSO3BSpline candidate;
    candidate.start_time_stamp_s_ = all_time_stamps_s.front();
    candidate.end_time_stamp_s_ = all_time_stamps_s.back();
    candidate.time_interval_s_ = time_interval_s;
    candidate.control_orientations_ = normalized_orientations;
    candidate.knots_.insert(candidate.knots_.end(), 4, candidate.start_time_stamp_s_);
    const double knot_interval_s = (candidate.end_time_stamp_s_ - candidate.start_time_stamp_s_) / (normalized_orientations.size() - 3);
    for (uint32_t i = 1; i + 3 < normalized_orientations.size(); ++i) {
        candidate.knots_.push_back(candidate.start_time_stamp_s_ + i * knot_interval_s);
    }
    candidate.knots_.insert(candidate.knots_.end(), 4, candidate.end_time_stamp_s_);

    // Solve for Lie-group control orientations so Fit retains its interpolation
    // semantics. Perturbations are applied in each control orientation's local frame.
    const uint32_t num_orientations = static_cast<uint32_t>(normalized_orientations.size());
    const uint32_t num_parameters = 3 * num_orientations;
    constexpr uint32_t kMaximumIterations = 30;
    constexpr double kResidualTolerance = 1e-10;
    constexpr double kDifferenceStep = 1e-7;
    for (uint32_t iteration = 0; iteration < kMaximumIterations; ++iteration) {
        TVec<double> residual(3 * num_orientations);
        for (uint32_t i = 0; i < num_orientations; ++i) {
            TQuat<double> estimated_orientation;
            TVec3<double> estimated_angular_velocity;
            RETURN_FALSE_IF(!candidate.Evaluate(all_time_stamps_s[i], estimated_orientation, estimated_angular_velocity));
            TQuat<double> error = estimated_orientation.inverse() * normalized_orientations[i];
            if (error.w() < 0.0) {
                error.coeffs() = -error.coeffs();
            }
            residual.template segment<3>(3 * i) = Utility::Logarithm(error);
        }
        if (residual.norm() < kResidualTolerance) {
            *this = std::move(candidate);
            return true;
        }

        TMat<double> jacobian(3 * num_orientations, num_parameters);
        for (uint32_t j = 0; j < num_orientations; ++j) {
            const TQuat<double> original_control = candidate.control_orientations_[j];
            for (uint32_t axis = 0; axis < 3; ++axis) {
                TVec3<double> perturbation = TVec3<double>::Zero();
                perturbation[axis] = kDifferenceStep;
                candidate.control_orientations_[j] = (original_control * Utility::Exponent(perturbation)).normalized();
                for (uint32_t i = 0; i < num_orientations; ++i) {
                    TQuat<double> estimated_orientation;
                    TVec3<double> estimated_angular_velocity;
                    candidate.Evaluate(all_time_stamps_s[i], estimated_orientation, estimated_angular_velocity);
                    TQuat<double> error = estimated_orientation.inverse() * normalized_orientations[i];
                    if (error.w() < 0.0) {
                        error.coeffs() = -error.coeffs();
                    }
                    jacobian.template block<3, 1>(3 * i, 3 * j + axis) = (Utility::Logarithm(error) - residual.template segment<3>(3 * i)) / kDifferenceStep;
                }
            }
            candidate.control_orientations_[j] = original_control;
        }
        const TVec<double> increment = jacobian.colPivHouseholderQr().solve(-residual);
        RETURN_FALSE_IF(!increment.allFinite());
        for (uint32_t i = 0; i < num_orientations; ++i) {
            candidate.control_orientations_[i] = (candidate.control_orientations_[i] * Utility::Exponent(increment.template segment<3>(3 * i))).normalized();
        }
    }
    return false;
}

bool CubicUniformSO3BSpline::GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity,
                                      TVec3<double> &angular_acceleration) const {
    RETURN_FALSE_IF(!IsFitted() || !GetAngularVelocity(time_stamp_s, angular_velocity));

    TVec3<double> evaluated_angular_velocity;
    RETURN_FALSE_IF(!Evaluate(time_stamp_s, orientation, evaluated_angular_velocity));

    const double derivative_interval_s = std::min(1e-4 * time_interval_s(), 1e-5);
    RETURN_FALSE_IF(derivative_interval_s <= 0.0);
    const double left_time_stamp_s = std::max(start_time_stamp_s(), time_stamp_s - derivative_interval_s);
    const double right_time_stamp_s = std::min(end_time_stamp_s(), time_stamp_s + derivative_interval_s);
    TVec3<double> left_angular_velocity = TVec3<double>::Zero();
    TVec3<double> right_angular_velocity = TVec3<double>::Zero();
    RETURN_FALSE_IF(!GetAngularVelocity(left_time_stamp_s, left_angular_velocity) || !GetAngularVelocity(right_time_stamp_s, right_angular_velocity));
    const double time_difference_s = right_time_stamp_s - left_time_stamp_s;
    RETURN_FALSE_IF(time_difference_s <= 0.0);
    angular_acceleration = (right_angular_velocity - left_angular_velocity) / time_difference_s;
    return true;
}

bool CubicUniformSO3BSpline::GetValue(const double time_stamp_s, TQuat<double> &orientation) const {
    TVec3<double> angular_velocity = TVec3<double>::Zero();
    TVec3<double> angular_acceleration = TVec3<double>::Zero();
    return GetValue(time_stamp_s, orientation, angular_velocity, angular_acceleration);
}

bool CubicUniformSO3BSpline::GetAngularVelocity(const double time_stamp_s, TVec3<double> &angular_velocity) const {
    TQuat<double> orientation;
    return Evaluate(time_stamp_s, orientation, angular_velocity);
}

bool CubicUniformSO3BSpline::Evaluate(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity) const {
    RETURN_FALSE_IF(!IsFitted() || time_stamp_s < start_time_stamp_s_ || time_stamp_s > end_time_stamp_s_);
    std::vector<double> basis;
    std::vector<double> first_basis;
    CalculateBasis(time_stamp_s, basis, first_basis);
    std::vector<double> cumulative_basis(basis.size(), 0.0);
    std::vector<double> cumulative_first_basis(basis.size(), 0.0);
    double basis_sum = 0.0;
    double first_basis_sum = 0.0;
    for (uint32_t i = static_cast<uint32_t>(basis.size()); i-- > 1;) {
        basis_sum += basis[i];
        first_basis_sum += first_basis[i];
        cumulative_basis[i] = basis_sum;
        cumulative_first_basis[i] = first_basis_sum;
    }

    orientation = control_orientations_.front();
    TVec3<double> velocity = TVec3<double>::Zero();
    for (uint32_t i = 1; i < control_orientations_.size(); ++i) {
        TQuat<double> relative_orientation = control_orientations_[i - 1].inverse() * control_orientations_[i];
        if (relative_orientation.w() < 0.0) {
            relative_orientation.coeffs() = -relative_orientation.coeffs();
        }
        const TVec3<double> increment = Utility::Logarithm(relative_orientation);
        const TQuat<double> factor = Utility::Exponent(cumulative_basis[i] * increment);
        orientation = (orientation * factor).normalized();
        velocity = factor.inverse() * velocity + cumulative_first_basis[i] * increment;
    }
    angular_velocity = velocity;
    return orientation.coeffs().allFinite() && velocity.allFinite();
}

void CubicUniformSO3BSpline::CalculateBasis(const double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis) const {
    const double evaluation_time_stamp_s = time_stamp_s == end_time_stamp_s_ ? std::nextafter(time_stamp_s, start_time_stamp_s_) : time_stamp_s;
    std::vector<std::vector<double>> basis_by_degree(4);
    basis_by_degree[0].resize(knots_.size() - 1, 0.0);
    for (uint32_t i = 0; i + 1 < knots_.size(); ++i) {
        if (evaluation_time_stamp_s >= knots_[i] && evaluation_time_stamp_s < knots_[i + 1]) {
            basis_by_degree[0][i] = 1.0;
        }
    }
    for (uint32_t degree = 1; degree <= 3; ++degree) {
        basis_by_degree[degree].resize(knots_.size() - degree - 1, 0.0);
        for (uint32_t i = 0; i < basis_by_degree[degree].size(); ++i) {
            const double left_denominator = knots_[i + degree] - knots_[i];
            const double right_denominator = knots_[i + degree + 1] - knots_[i + 1];
            if (left_denominator > 0.0) {
                basis_by_degree[degree][i] += (evaluation_time_stamp_s - knots_[i]) / left_denominator * basis_by_degree[degree - 1][i];
            }
            if (right_denominator > 0.0) {
                basis_by_degree[degree][i] += (knots_[i + degree + 1] - evaluation_time_stamp_s) / right_denominator * basis_by_degree[degree - 1][i + 1];
            }
        }
    }
    basis.assign(basis_by_degree[3].begin(), basis_by_degree[3].begin() + control_orientations_.size());
    first_basis.assign(control_orientations_.size(), 0.0);
    for (uint32_t i = 0; i < control_orientations_.size(); ++i) {
        const double left_denominator = knots_[i + 3] - knots_[i];
        const double right_denominator = knots_[i + 4] - knots_[i + 1];
        first_basis[i] = (left_denominator > 0.0 ? 3.0 / left_denominator * basis_by_degree[2][i] : 0.0) -
                         (right_denominator > 0.0 ? 3.0 / right_denominator * basis_by_degree[2][i + 1] : 0.0);
    }
}

}  // namespace slam_utility
