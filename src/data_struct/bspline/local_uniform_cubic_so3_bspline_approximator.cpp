#include "local_uniform_cubic_so3_bspline_approximator.h"

#include "slam_basic_math.h"
#include "slam_operations.h"

#include "algorithm"
#include "cmath"

namespace slam_utility {

// Standard uniform cubic B-spline basis and its first derivative with respect
// to the local parameter u in [0, 1], shared by the SO(3) cumulative weights.
void LocalUniformCubicSO3BSplineApproximator::CalculateBasisAndFirstDerivative(const double u, double (&basis)[4], double (&first_basis)[4]) {
    const double u2 = u * u;
    const double u3 = u2 * u;
    basis[0] = (1.0 - u) * (1.0 - u) * (1.0 - u) / 6.0;
    basis[1] = (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0;
    basis[2] = (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0;
    basis[3] = u3 / 6.0;
    first_basis[0] = -(1.0 - u) * (1.0 - u) / 2.0;
    first_basis[1] = (3.0 * u2 - 4.0 * u) / 2.0;
    first_basis[2] = (-3.0 * u2 + 2.0 * u + 1.0) / 2.0;
    first_basis[3] = u2 / 2.0;
}

bool LocalUniformCubicSO3BSplineApproximator::Fit(const double start_time_s, const double interval_s, const std::vector<TQuat<double>> &controls) {
    RETURN_FALSE_IF(!std::isfinite(start_time_s) || !std::isfinite(interval_s) || interval_s <= 0.0);
    RETURN_FALSE_IF(controls.size() < 4);
    for (const TQuat<double> &control: controls) {
        RETURN_FALSE_IF(!std::isfinite(control.w()) || !control.vec().allFinite() || control.norm() < kZeroDouble);
    }

    // Normalize and keep consecutive samples on the same hemisphere so their
    // logarithms vary continuously.
    LocalUniformCubicSO3BSplineApproximator candidate;
    candidate.start_time_s_ = start_time_s;
    candidate.interval_s_ = interval_s;
    candidate.controls_.reserve(controls.size());
    for (const TQuat<double> &control: controls) {
        TQuat<double> orientation = control.normalized();
        if (!candidate.controls_.empty() && candidate.controls_.back().coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() = -orientation.coeffs();
        }
        candidate.controls_.emplace_back(orientation);
    }
    *this = std::move(candidate);
    return true;
}

bool LocalUniformCubicSO3BSplineApproximator::Evaluate(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity) const {
    RETURN_FALSE_IF(!IsFitted());
    const double end_time_s = end_time_stamp_s();
    const double tolerance = 1e-9 * std::max(1.0, std::fabs(end_time_s - start_time_s_));
    RETURN_FALSE_IF(time_stamp_s < start_time_s_ - tolerance || time_stamp_s > end_time_s + tolerance);
    const double time_s = std::clamp(time_stamp_s, start_time_s_, end_time_s);

    const uint32_t num_controls = static_cast<uint32_t>(controls_.size());
    const double raw_span = (time_s - start_time_s_) / interval_s_;
    uint32_t span = 0;
    double u = 0.0;
    if (raw_span >= static_cast<double>(num_controls - 1)) {
        span = num_controls - 2;
        u = 1.0;
    } else {
        span = static_cast<uint32_t>(raw_span);
        u = raw_span - static_cast<double>(span);
    }

    // Active control orientations of this span, with one duplicated ghost
    // control at each end.
    const uint32_t index[4] = {
        span > 0 ? span - 1 : 0,
        span,
        std::min(span + 1, num_controls - 1),
        std::min(span + 2, num_controls - 1),
    };
    const TQuat<double> c0 = controls_[index[0]];
    const TQuat<double> c1 = controls_[index[1]];
    const TQuat<double> c2 = controls_[index[2]];
    const TQuat<double> c3 = controls_[index[3]];

    // Consecutive Lie-algebra increments between the four active orientations.
    TQuat<double> relative_orientations[3] = {c0.inverse() * c1, c1.inverse() * c2, c2.inverse() * c3};
    TVec3<double> increments[3];
    for (uint32_t i = 0; i < 3; ++i) {
        if (relative_orientations[i].w() < 0.0) {
            relative_orientations[i].coeffs() = -relative_orientations[i].coeffs();
        }
        increments[i] = Utility::Logarithm(relative_orientations[i]);
    }

    // Cumulative weights of the three increments derive from the scalar cubic
    // basis: w1 = 1 - b0, w2 = b2 + b3, w3 = b3.
    double basis[4];
    double first_basis[4];
    CalculateBasisAndFirstDerivative(u, basis, first_basis);
    const double weight[3] = {1.0 - basis[0], basis[2] + basis[3], basis[3]};
    const double first_weight[3] = {-first_basis[0], first_basis[2] + first_basis[3], first_basis[3]};

    // Cumulative Lie-group spline value and right-trivialized (body-frame)
    // angular velocity with respect to u, using the same recurrence as
    // ClampedCubicSO3BSplineInterpolator restricted to the active span.
    const TQuat<double> factor[3] = {Utility::Exponent(weight[0] * increments[0]), Utility::Exponent(weight[1] * increments[1]),
                                     Utility::Exponent(weight[2] * increments[2])};
    orientation = (c0 * factor[0] * factor[1] * factor[2]).normalized();
    TVec3<double> velocity = TVec3<double>::Zero();
    for (uint32_t i = 0; i < 3; ++i) {
        velocity = factor[i].inverse() * velocity + first_weight[i] * increments[i];
    }
    // Convert the derivative with respect to u into one with respect to time.
    angular_velocity = velocity / interval_s_;
    return orientation.coeffs().allFinite() && angular_velocity.allFinite();
}

bool LocalUniformCubicSO3BSplineApproximator::GetAngularAcceleration(const double time_stamp_s, TVec3<double> &angular_acceleration) const {
    RETURN_FALSE_IF(!IsFitted());
    const double end_time_s = end_time_stamp_s();
    const double derivative_interval_s = std::min(1e-4 * interval_s_, 1e-5);
    RETURN_FALSE_IF(derivative_interval_s <= 0.0);
    const double left_time_s = std::max(start_time_s_, time_stamp_s - derivative_interval_s);
    const double right_time_s = std::min(end_time_s, time_stamp_s + derivative_interval_s);
    TQuat<double> left_orientation;
    TVec3<double> left_angular_velocity = TVec3<double>::Zero();
    TQuat<double> right_orientation;
    TVec3<double> right_angular_velocity = TVec3<double>::Zero();
    RETURN_FALSE_IF(!Evaluate(left_time_s, left_orientation, left_angular_velocity) ||
                    !Evaluate(right_time_s, right_orientation, right_angular_velocity));
    const double time_difference_s = right_time_s - left_time_s;
    RETURN_FALSE_IF(time_difference_s <= 0.0);
    angular_acceleration = (right_angular_velocity - left_angular_velocity) / time_difference_s;
    return true;
}

bool LocalUniformCubicSO3BSplineApproximator::GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity,
                                      TVec3<double> &angular_acceleration) const {
    RETURN_FALSE_IF(!Evaluate(time_stamp_s, orientation, angular_velocity));
    return GetAngularAcceleration(time_stamp_s, angular_acceleration);
}

bool LocalUniformCubicSO3BSplineApproximator::GetValue(const double time_stamp_s, TQuat<double> &orientation) const {
    TVec3<double> angular_velocity = TVec3<double>::Zero();
    TVec3<double> angular_acceleration = TVec3<double>::Zero();
    return GetValue(time_stamp_s, orientation, angular_velocity, angular_acceleration);
}

}  // namespace slam_utility
