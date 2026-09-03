#include "local_uniform_cubic_so3_bspline_approximator.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "algorithm"
#include "cassert"
#include "chrono"
#include "cmath"
#include "random"
#include "vector"

using namespace slam_utility;

// Minimal rotation angle between two orientations, robust to the q == -q alias.
double OrientationError(const TQuat<double> &lhs, const TQuat<double> &rhs) {
    TQuat<double> difference = lhs.inverse() * rhs;
    if (difference.w() < 0.0) {
        difference.coeffs() = -difference.coeffs();
    }
    return 2.0 * std::atan2(difference.vec().norm(), difference.w());
}

// Body-frame angular velocity by geodesic central difference of two spline
// orientations, matching the convention validated by the existing spline test.
bool NumericalAngularVelocity(const LocalUniformCubicSO3BSplineApproximator &spline, const double time_stamp_s, const double interval_s,
                              TVec3<double> &angular_velocity) {
    TQuat<double> left_orientation;
    TQuat<double> right_orientation;
    if (!spline.GetValue(time_stamp_s - interval_s, left_orientation) || !spline.GetValue(time_stamp_s + interval_s, right_orientation)) {
        return false;
    }
    TQuat<double> difference = left_orientation.inverse() * right_orientation;
    if (difference.w() < 0.0) {
        difference.coeffs() = -difference.coeffs();
    }
    angular_velocity = Utility::Logarithm(difference) / (2.0 * interval_s);
    return true;
}

// Constant-rate rotation about a fixed axis is the SO(3) analog of a straight
// line. The cumulative spline must reproduce it exactly: geodesic-line
// precision. Interior knots only (the outermost spans use a duplicated ghost).
void TestConstantRotationPrecision() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.05;
    constexpr double kAngularSpeed = 0.7;
    constexpr uint32_t kNumControls = 60;
    const TVec3<double> axis = TVec3<double>(0.2, -0.3, 0.8).normalized();

    std::vector<TQuat<double>> controls;
    for (uint32_t i = 0; i < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        controls.emplace_back(Utility::Exponent(axis * kAngularSpeed * time_s));
    }
    LocalUniformCubicSO3BSplineApproximator spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));

    double max_orientation_error = 0.0;
    double max_angular_velocity_error = 0.0;
    double max_angular_acceleration = 0.0;
    const TVec3<double> expected_angular_velocity = axis * kAngularSpeed;
    for (uint32_t i = 3; i + 3 < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        const TQuat<double> expected_orientation = Utility::Exponent(axis * kAngularSpeed * time_s);
        TQuat<double> orientation;
        TVec3<double> angular_velocity;
        TVec3<double> angular_acceleration;
        assert(spline.GetValue(time_s, orientation, angular_velocity, angular_acceleration));
        max_orientation_error = std::max(max_orientation_error, OrientationError(expected_orientation, orientation));
        max_angular_velocity_error = std::max(max_angular_velocity_error, (angular_velocity - expected_angular_velocity).norm());
        max_angular_acceleration = std::max(max_angular_acceleration, angular_acceleration.norm());
    }
    ReportInfo("Constant rotation errors: " << max_orientation_error << " / " << max_angular_velocity_error << " / alpha " << max_angular_acceleration);
    assert(max_orientation_error < 1e-9);
    assert(max_angular_velocity_error < 1e-8);
    assert(max_angular_acceleration < 1e-7);
}

// The analytic angular velocity must equal a geodesic central difference of the
// returned orientation even under non-commuting (two-axis) motion, and the curve
// stays C^1: angular velocity is continuous across interior knots.
void TestAngularVelocityConsistency() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.01;
    constexpr uint32_t kNumControls = 1200;
    const TVec3<double> axis_x = TVec3<double>(1.0, 0.0, 0.0);
    const TVec3<double> axis_z = TVec3<double>(0.0, 0.0, 1.0);
    const auto OrientationAt = [&axis_x, &axis_z](const double time_s) {
        return (Utility::Exponent(axis_z * (0.3 * std::sin(0.5 * time_s))) * Utility::Exponent(axis_x * (0.5 * time_s))).normalized();
    };

    std::vector<TQuat<double>> controls;
    for (uint32_t i = 0; i < kNumControls; ++i) {
        controls.emplace_back(OrientationAt(kStartTimeS + i * kIntervalS));
    }
    LocalUniformCubicSO3BSplineApproximator spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));

    constexpr double kDifferenceInterval = 1e-5;
    double max_velocity_error = 0.0;
    double max_orientation_error = 0.0;
    for (uint32_t i = 3; i + 3 < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        TQuat<double> orientation;
        TVec3<double> angular_velocity;
        TVec3<double> angular_acceleration;
        assert(spline.GetValue(time_s, orientation, angular_velocity, angular_acceleration));
        TVec3<double> numerical_angular_velocity;
        assert(NumericalAngularVelocity(spline, time_s, kDifferenceInterval, numerical_angular_velocity));
        max_velocity_error = std::max(max_velocity_error, (angular_velocity - numerical_angular_velocity).norm());
        max_orientation_error = std::max(max_orientation_error, OrientationError(OrientationAt(time_s), orientation));
    }
    ReportInfo("Angular velocity FD error: " << max_velocity_error << ", orientation error vs truth: " << max_orientation_error);
    assert(max_velocity_error < 1e-5);
    assert(max_orientation_error < 1e-4);

    // Angular velocity is continuous across an interior knot (the orientation
    // curve is C^1, and is in fact C^2 with a smooth angular acceleration).
    double max_velocity_jump = 0.0;
    for (uint32_t i = 4; i + 4 < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        TQuat<double> left_orientation;
        TQuat<double> right_orientation;
        TVec3<double> left_velocity;
        TVec3<double> right_velocity;
        TVec3<double> left_acceleration;
        TVec3<double> right_acceleration;
        assert(spline.GetValue(time_s - 1e-9, left_orientation, left_velocity, left_acceleration));
        assert(spline.GetValue(time_s + 1e-9, right_orientation, right_velocity, right_acceleration));
        max_velocity_jump = std::max(max_velocity_jump, (left_velocity - right_velocity).norm());
    }
    ReportInfo("Angular velocity jump across knots: " << max_velocity_jump);
    assert(max_velocity_jump < 1e-6);
}

// Halving the grid interval must shrink the approximation error of a smooth
// closed-form orientation trajectory.
void TestApproximationConvergence() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kDurationS = 10.0;
    const TVec3<double> axis_y = TVec3<double>(0.0, 1.0, 0.0);
    const TVec3<double> axis_z = TVec3<double>(0.0, 0.0, 1.0);
    const auto OrientationAt = [&axis_y, &axis_z](const double time_s) {
        return (Utility::Exponent(axis_y * (0.4 * time_s)) * Utility::Exponent(axis_z * (0.3 * std::sin(0.5 * time_s)))).normalized();
    };

    const auto MeasureMaxOrientationError = [&](const double interval_s, const uint32_t num_controls) {
        std::vector<TQuat<double>> controls;
        for (uint32_t i = 0; i < num_controls; ++i) {
            controls.emplace_back(OrientationAt(kStartTimeS + i * interval_s));
        }
        LocalUniformCubicSO3BSplineApproximator spline;
        assert(spline.Fit(kStartTimeS, interval_s, controls));
        double max_orientation_error = 0.0;
        for (uint32_t i = 3; i + 3 < num_controls; ++i) {
            const double time_s = kStartTimeS + i * interval_s;
            TQuat<double> orientation;
            assert(spline.GetValue(time_s, orientation));
            max_orientation_error = std::max(max_orientation_error, OrientationError(OrientationAt(time_s), orientation));
        }
        return max_orientation_error;
    };

    const auto coarse_error = MeasureMaxOrientationError(0.02, static_cast<uint32_t>(kDurationS / 0.02) + 1);
    const auto fine_error = MeasureMaxOrientationError(0.01, static_cast<uint32_t>(kDurationS / 0.01) + 1);
    ReportInfo("Coarse orientation error: " << coarse_error << ", fine: " << fine_error);
    assert(fine_error < 0.5 * coarse_error);
    assert(fine_error < 1e-3);
}

// The spline must scale to very long trajectories: O(N) build and O(1) queries.
void TestLongTrajectoryScale() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.01;
    constexpr uint32_t kNumControls = 100000;
    const TVec3<double> axis = TVec3<double>(0.0, 0.0, 1.0);
    std::vector<TQuat<double>> controls;
    controls.reserve(kNumControls);
    for (uint32_t i = 0; i < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        // A rotating and nodding motion so hemisphere flips are exercised.
        const TQuat<double> tilt = Utility::Exponent(TVec3<double>(0.4 * std::sin(0.01 * time_s), 0.0, 0.0));
        controls.emplace_back((tilt * Utility::Exponent(axis * time_s)).normalized());
    }

    const auto begin_fit = std::chrono::steady_clock::now();
    LocalUniformCubicSO3BSplineApproximator spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));
    const double fit_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin_fit).count();

    std::mt19937 random_engine(2);
    std::uniform_real_distribution<double> distribution(kStartTimeS + 1.0, spline.end_time_stamp_s() - 1.0);
    const auto begin_query = std::chrono::steady_clock::now();
    TQuat<double> orientation;
    TVec3<double> angular_velocity;
    TVec3<double> angular_acceleration;
    for (uint32_t i = 0; i < 100000; ++i) {
        assert(spline.GetValue(distribution(random_engine), orientation, angular_velocity, angular_acceleration));
        assert(orientation.coeffs().allFinite() && angular_velocity.allFinite() && angular_acceleration.allFinite());
    }
    const double query_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin_query).count();
    ReportInfo("Long trajectory: " << kNumControls << " controls, fit " << fit_time_s << " s, 100k queries " << query_time_s << " s.");
    assert(fit_time_s < 5.0);
    assert(query_time_s < 5.0);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test local uniform cubic SO(3) B-spline approximator." RESET_COLOR);
    TestConstantRotationPrecision();
    TestAngularVelocityConsistency();
    TestApproximationConvergence();
    TestLongTrajectoryScale();
    ReportInfo(GREEN ">> Local uniform cubic SO(3) B-spline approximator test passed." RESET_COLOR);
    return 0;
}
