#include "cubic_uniform_so3_bspline.h"
#include "algorithm"
#include "cmath"

namespace slam_utility {

bool CubicUniformSO3BSpline::Fit(const std::vector<double> &all_time_stamps_s, const std::vector<TQuat<double>> &all_orientations) {
    if (all_time_stamps_s.size() != all_orientations.size() || all_orientations.size() < 4) {
        return false;
    }

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

    const TQuat<double> reference_orientation = normalized_orientations.front();
    std::vector<TVec3<double>> rotation_vectors;
    rotation_vectors.reserve(normalized_orientations.size());
    for (const TQuat<double> &orientation: normalized_orientations) {
        rotation_vectors.emplace_back(Utility::Logarithm(reference_orientation.inverse() * orientation));
    }

    CubicUniformBSpline<TVec3<double>> rotation_vector_spline;
    if (!rotation_vector_spline.Fit(all_time_stamps_s, rotation_vectors)) {
        return false;
    }
    reference_orientation_ = reference_orientation;
    rotation_vector_spline_ = rotation_vector_spline;
    return true;
}

bool CubicUniformSO3BSpline::GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity,
                                      TVec3<double> &angular_acceleration) const {
    if (!IsFitted() || !GetAngularVelocity(time_stamp_s, angular_velocity)) {
        return false;
    }

    TVec3<double> rotation_vector = TVec3<double>::Zero();
    TVec3<double> rotation_vector_derivative = TVec3<double>::Zero();
    TVec3<double> rotation_vector_second_derivative = TVec3<double>::Zero();
    if (!rotation_vector_spline_.GetValue(time_stamp_s, rotation_vector, rotation_vector_derivative, rotation_vector_second_derivative)) {
        return false;
    }
    orientation = (reference_orientation_ * Utility::Exponent(rotation_vector)).normalized();

    const double derivative_interval_s = std::min(1e-4 * time_interval_s(), 1e-5);
    if (derivative_interval_s <= 0.0) {
        return false;
    }
    const double left_time_stamp_s = std::max(start_time_stamp_s(), time_stamp_s - derivative_interval_s);
    const double right_time_stamp_s = std::min(end_time_stamp_s(), time_stamp_s + derivative_interval_s);
    TVec3<double> left_angular_velocity = TVec3<double>::Zero();
    TVec3<double> right_angular_velocity = TVec3<double>::Zero();
    if (!GetAngularVelocity(left_time_stamp_s, left_angular_velocity) || !GetAngularVelocity(right_time_stamp_s, right_angular_velocity)) {
        return false;
    }
    const double time_difference_s = right_time_stamp_s - left_time_stamp_s;
    if (time_difference_s <= 0.0) {
        return false;
    }
    angular_acceleration = (right_angular_velocity - left_angular_velocity) / time_difference_s;
    return true;
}

bool CubicUniformSO3BSpline::GetValue(const double time_stamp_s, TQuat<double> &orientation) const {
    TVec3<double> angular_velocity = TVec3<double>::Zero();
    TVec3<double> angular_acceleration = TVec3<double>::Zero();
    return GetValue(time_stamp_s, orientation, angular_velocity, angular_acceleration);
}

bool CubicUniformSO3BSpline::GetAngularVelocity(const double time_stamp_s, TVec3<double> &angular_velocity) const {
    TVec3<double> rotation_vector = TVec3<double>::Zero();
    TVec3<double> rotation_vector_derivative = TVec3<double>::Zero();
    if (!rotation_vector_spline_.GetValue(time_stamp_s, rotation_vector, rotation_vector_derivative)) {
        return false;
    }
    angular_velocity = Utility::RightJacobian(rotation_vector) * rotation_vector_derivative;
    return angular_velocity.allFinite();
}

}  // namespace slam_utility
