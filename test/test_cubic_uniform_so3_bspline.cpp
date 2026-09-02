#include "cubic_uniform_so3_bspline.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "algorithm"
#include "cassert"
#include "cmath"
#include "iomanip"
#include "vector"

using namespace slam_utility;

// Reproduce the former fixed-reference tangent-space implementation so the
// regression test can compare both formulations with exactly the same input.
class FixedTangentSO3BSpline {

public:
    bool Fit(const std::vector<double> &time_stamps_s, const std::vector<TQuat<double>> &orientations) {
        reference_orientation_ = orientations.front().normalized();
        std::vector<TVec3<double>> rotation_vectors;
        rotation_vectors.reserve(orientations.size());
        TQuat<double> previous_orientation = reference_orientation_;
        for (const TQuat<double> &input_orientation: orientations) {
            TQuat<double> orientation = input_orientation.normalized();
            if (previous_orientation.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() = -orientation.coeffs();
            }
            rotation_vectors.emplace_back(Utility::Logarithm(reference_orientation_.inverse() * orientation));
            previous_orientation = orientation;
        }
        return rotation_vector_spline_.Fit(time_stamps_s, rotation_vectors);
    }

    bool GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity) const {
        TVec3<double> rotation_vector;
        TVec3<double> rotation_vector_derivative;
        if (!rotation_vector_spline_.GetValue(time_stamp_s, rotation_vector, rotation_vector_derivative)) {
            return false;
        }
        orientation = (reference_orientation_ * Utility::Exponent(rotation_vector)).normalized();
        angular_velocity = Utility::RightJacobian(rotation_vector) * rotation_vector_derivative;
        return true;
    }

private:
    TQuat<double> reference_orientation_ = TQuat<double>::Identity();
    CubicUniformBSpline<TVec3<double>> rotation_vector_spline_;
};

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test cubic uniform SO(3) B-spline." RESET_COLOR);
    const std::vector<double> time_stamp_s {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<TQuat<double>> orientations;
    for (const double time_stamp: time_stamp_s) {
        orientations.emplace_back(Utility::Exponent(TVec3<double>(0.0, 0.0, 0.4 * time_stamp)));
    }

    CubicUniformSO3BSpline spline;
    assert(spline.Fit(time_stamp_s, orientations));
    for (uint32_t i = 0; i < time_stamp_s.size(); ++i) {
        TQuat<double> orientation = TQuat<double>::Identity();
        TVec3<double> angular_velocity = TVec3<double>::Zero();
        TVec3<double> angular_acceleration = TVec3<double>::Zero();
        assert(spline.GetValue(time_stamp_s[i], orientation, angular_velocity, angular_acceleration));
        assert(2.0 * (orientations[i].inverse() * orientation).vec().norm() < 1e-9);
        assert((angular_velocity - TVec3<double>(0.0, 0.0, 0.4)).norm() < 1e-9);
        assert(angular_acceleration.norm() < 1e-7);
    }

    // Non-commuting rotations exercise the cumulative Lie-group product. Check
    // both interpolation and the body-frame angular velocity returned by it.
    std::vector<TQuat<double>> non_commuting_orientations {
        TQuat<double>::Identity(), Utility::Exponent(TVec3<double>(0.20, -0.10, 0.05)), Utility::Exponent(TVec3<double>(-0.15, 0.25, 0.20)),
        Utility::Exponent(TVec3<double>(0.30, 0.10, -0.20)), Utility::Exponent(TVec3<double>(-0.05, -0.20, 0.35))};
    CubicUniformSO3BSpline non_commuting_spline;
    assert(non_commuting_spline.Fit(time_stamp_s, non_commuting_orientations));
    for (uint32_t i = 0; i < time_stamp_s.size(); ++i) {
        TQuat<double> orientation;
        assert(non_commuting_spline.GetValue(time_stamp_s[i], orientation));
        assert(2.0 * (non_commuting_orientations[i].inverse() * orientation).vec().norm() < 1e-8);
    }
    constexpr double kEvaluationTimeStamp = 1.7;
    constexpr double kDifferenceInterval = 1e-6;
    TQuat<double> orientation;
    TQuat<double> left_orientation;
    TQuat<double> right_orientation;
    TVec3<double> angular_velocity;
    TVec3<double> angular_acceleration;
    assert(non_commuting_spline.GetValue(kEvaluationTimeStamp, orientation, angular_velocity, angular_acceleration));
    assert(non_commuting_spline.GetValue(kEvaluationTimeStamp - kDifferenceInterval, left_orientation));
    assert(non_commuting_spline.GetValue(kEvaluationTimeStamp + kDifferenceInterval, right_orientation));
    TQuat<double> orientation_difference = left_orientation.inverse() * right_orientation;
    if (orientation_difference.w() < 0.0) {
        orientation_difference.coeffs() = -orientation_difference.coeffs();
    }
    const TVec3<double> numerical_angular_velocity = Utility::Logarithm(orientation_difference) / (2.0 * kDifferenceInterval);
    assert((angular_velocity - numerical_angular_velocity).norm() < 1e-7);

    // A fixed reference logarithm crosses its principal branch when the total
    // rotation exceeds pi. The cumulative spline only logs adjacent rotations,
    // so a continuously sampled multi-turn motion remains smooth and keeps the
    // correct angular velocity.
    std::vector<double> large_rotation_time_stamps_s;
    std::vector<TQuat<double>> large_rotation_orientations;
    for (uint32_t i = 0; i <= 8; ++i) {
        large_rotation_time_stamps_s.emplace_back(static_cast<double>(i));
        large_rotation_orientations.emplace_back(Utility::Exponent(TVec3<double>(0.0, 0.0, static_cast<double>(i))));
    }
    CubicUniformSO3BSpline cumulative_spline;
    FixedTangentSO3BSpline fixed_tangent_spline;
    assert(cumulative_spline.Fit(large_rotation_time_stamps_s, large_rotation_orientations));
    assert(fixed_tangent_spline.Fit(large_rotation_time_stamps_s, large_rotation_orientations));

    double maximum_cumulative_velocity_error = 0.0;
    double maximum_cumulative_acceleration = 0.0;
    double maximum_fixed_tangent_velocity_error = 0.0;
    double maximum_cumulative_orientation_error = 0.0;
    double maximum_fixed_tangent_orientation_error = 0.0;
    for (uint32_t i = 0; i <= 160; ++i) {
        const double evaluation_time_stamp_s = 0.05 * i;
        const TQuat<double> expected_orientation = Utility::Exponent(TVec3<double>(0.0, 0.0, evaluation_time_stamp_s));
        TQuat<double> cumulative_orientation;
        TQuat<double> fixed_tangent_orientation;
        TVec3<double> cumulative_angular_velocity;
        TVec3<double> cumulative_angular_acceleration;
        TVec3<double> fixed_tangent_angular_velocity;
        assert(cumulative_spline.GetValue(evaluation_time_stamp_s, cumulative_orientation, cumulative_angular_velocity, cumulative_angular_acceleration));
        assert(fixed_tangent_spline.GetValue(evaluation_time_stamp_s, fixed_tangent_orientation, fixed_tangent_angular_velocity));
        maximum_cumulative_velocity_error = std::max(maximum_cumulative_velocity_error, (cumulative_angular_velocity - TVec3<double>(0.0, 0.0, 1.0)).norm());
        maximum_cumulative_acceleration = std::max(maximum_cumulative_acceleration, cumulative_angular_acceleration.norm());
        maximum_fixed_tangent_velocity_error =
            std::max(maximum_fixed_tangent_velocity_error, (fixed_tangent_angular_velocity - TVec3<double>(0.0, 0.0, 1.0)).norm());
        maximum_cumulative_orientation_error =
            std::max(maximum_cumulative_orientation_error, 2.0 * (expected_orientation.inverse() * cumulative_orientation).vec().norm());
        maximum_fixed_tangent_orientation_error =
            std::max(maximum_fixed_tangent_orientation_error, 2.0 * (expected_orientation.inverse() * fixed_tangent_orientation).vec().norm());
    }
    assert(maximum_cumulative_velocity_error < 1e-8);
    assert(maximum_cumulative_acceleration < 1e-7);
    assert(maximum_cumulative_orientation_error < 1e-8);
    assert(maximum_fixed_tangent_velocity_error > 1.0);
    assert(maximum_fixed_tangent_orientation_error > 0.1);
    ReportInfo("\n>> Large-rotation comparison (constant 1 rad/s rotation over 8 rad):");
    ReportInfo("   Method                         Max orientation error [rad]   Max angular velocity error [rad/s]");
    ReportInfo("   Cumulative Lie-group spline    " << std::scientific << std::setprecision(6) << maximum_cumulative_orientation_error << "              "
                                                    << maximum_cumulative_velocity_error);
    ReportInfo("   Fixed-reference tangent spline " << std::scientific << std::setprecision(6) << maximum_fixed_tangent_orientation_error << "              "
                                                    << maximum_fixed_tangent_velocity_error);
    ReportInfo("   Cumulative spline max angular acceleration [rad/s^2]: " << std::scientific << std::setprecision(6) << maximum_cumulative_acceleration);
    ReportInfo(GREEN ">> All cubic uniform SO(3) B-spline tests passed." RESET_COLOR);
    return 0;
}
