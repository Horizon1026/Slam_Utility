#include "local_uniform_cubic_bspline_approximator.h"
#include "slam_log_reporter.h"

#include "cassert"
#include "chrono"
#include "cmath"
#include "random"
#include "vector"

using namespace slam_utility;

// Linearly spaced control points on a straight line are reproduced exactly by
// the uniform cubic B-spline (linear precision). Checks value/velocity/acceleration.
void TestLinearPrecision() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.1;
    constexpr double kVelocity = 3.0;
    constexpr uint32_t kNumControls = 8;
    std::vector<TVec3<double>> controls;
    for (uint32_t i = 0; i < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        controls.emplace_back(2.0 + kVelocity * time_s, -1.0 + kVelocity * time_s, 0.5 - 2.0 * kVelocity * time_s);
    }

    LocalUniformCubicBSplineApproximator<TVec3<double>> spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));
    assert(spline.IsFitted());
    // Keep away from the outermost biased spans; interior is exact.
    for (uint32_t i = 1; i + 1 < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        TVec3<double> position;
        TVec3<double> velocity;
        TVec3<double> acceleration;
        assert(spline.GetValue(time_s, position, velocity, acceleration));
        const TVec3<double> expect_position(2.0 + kVelocity * time_s, -1.0 + kVelocity * time_s, 0.5 - 2.0 * kVelocity * time_s);
        const TVec3<double> expect_velocity(kVelocity, kVelocity, -2.0 * kVelocity);
        assert((position - expect_position).norm() < 1e-9);
        assert((velocity - expect_velocity).norm() < 1e-9);
        assert(acceleration.norm() < 1e-9);
    }

    // Scalar variant.
    LocalUniformCubicBSplineApproximator<double> scalar_spline;
    std::vector<double> scalar_controls;
    for (uint32_t i = 0; i < kNumControls; ++i) {
        scalar_controls.emplace_back(1.0 + 2.0 * (kStartTimeS + i * kIntervalS));
    }
    assert(scalar_spline.Fit(kStartTimeS, kIntervalS, scalar_controls));
    double scalar_value;
    double scalar_velocity;
    double scalar_acceleration;
    assert(scalar_spline.GetValue(kStartTimeS + 3 * kIntervalS, scalar_value, scalar_velocity, scalar_acceleration));
    assert(std::fabs(scalar_value - (1.0 + 2.0 * (kStartTimeS + 3 * kIntervalS))) < 1e-9);
    assert(std::fabs(scalar_velocity - 2.0) < 1e-9);
    assert(std::fabs(scalar_acceleration) < 1e-9);

    ReportInfo("Linear precision passed.");
}

// The fitted spline is a smooth approximation of its samples: sampling a smooth
// closed-form curve with a finer grid must reduce the approximation error.
void TestApproximationConvergence() {
    const auto Value = [](const double time_s, TVec3<double> &position, TVec3<double> &velocity, TVec3<double> &acceleration) {
        // A smooth non-linear closed-form trajectory.
        position = TVec3<double>(std::sin(0.5 * time_s), std::cos(0.5 * time_s), 0.3 * time_s * time_s);
        velocity = TVec3<double>(0.5 * std::cos(0.5 * time_s), -0.5 * std::sin(0.5 * time_s), 0.6 * time_s);
        acceleration = TVec3<double>(-0.25 * std::sin(0.5 * time_s), -0.25 * std::cos(0.5 * time_s), 0.6);
    };

    const auto MeasureMaxError = [&Value](const double interval_s) {
        constexpr double kStartTimeS = 0.0;
        constexpr uint32_t kNumControls = 201;
        std::vector<TVec3<double>> controls;
        for (uint32_t i = 0; i < kNumControls; ++i) {
            TVec3<double> position;
            TVec3<double> velocity;
            TVec3<double> acceleration;
            Value(kStartTimeS + i * interval_s, position, velocity, acceleration);
            controls.emplace_back(position);
        }
        LocalUniformCubicBSplineApproximator<TVec3<double>> spline;
        assert(spline.Fit(kStartTimeS, interval_s, controls));
        double max_position_error = 0.0;
        double max_velocity_error = 0.0;
        // Dense interior query set.
        for (uint32_t i = 2; i + 2 < kNumControls; ++i) {
            TVec3<double> expect_position;
            TVec3<double> expect_velocity;
            TVec3<double> expect_acceleration;
            Value(kStartTimeS + i * interval_s, expect_position, expect_velocity, expect_acceleration);
            TVec3<double> position;
            TVec3<double> velocity;
            TVec3<double> acceleration;
            assert(spline.GetValue(kStartTimeS + i * interval_s, position, velocity, acceleration));
            max_position_error = std::max(max_position_error, (position - expect_position).norm());
            max_velocity_error = std::max(max_velocity_error, (velocity - expect_velocity).norm());
        }
        return std::make_pair(max_position_error, max_velocity_error);
    };

    const auto coarse_error = MeasureMaxError(0.01);
    const auto fine_error = MeasureMaxError(0.005);
    ReportInfo("Coarse error: " << coarse_error.first << " / " << coarse_error.second);
    ReportInfo("Fine error:   " << fine_error.first << " / " << fine_error.second);
    // Halving the grid interval must at least halve the error (it shrinks as O(dt^2)).
    assert(fine_error.first < 0.5 * coarse_error.first);
    assert(fine_error.second < 0.5 * coarse_error.second);
    // And the absolute error must be small on the fine grid.
    assert(fine_error.first < 1e-5);
    assert(fine_error.second < 1e-4);
}

// Uniform circular motion: check that the analytic velocity and acceleration
// match the physical values (R w, R w^2 scaling is handled correctly).
void TestCircularMotion() {
    constexpr double kRadius = 2.0;
    constexpr double kAngularSpeed = 1.5;
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.01;
    constexpr uint32_t kNumControls = 200;
    std::vector<TVec3<double>> controls;
    for (uint32_t i = 0; i < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        controls.emplace_back(kRadius * std::cos(kAngularSpeed * time_s), kRadius * std::sin(kAngularSpeed * time_s), 0.0);
    }

    LocalUniformCubicBSplineApproximator<TVec3<double>> spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));
    double max_position_error = 0.0;
    double max_velocity_error = 0.0;
    double max_acceleration_error = 0.0;
    for (uint32_t i = 3; i + 3 < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        TVec3<double> position;
        TVec3<double> velocity;
        TVec3<double> acceleration;
        assert(spline.GetValue(time_s, position, velocity, acceleration));
        const TVec3<double> expect_position = TVec3<double>(kRadius * std::cos(kAngularSpeed * time_s), kRadius * std::sin(kAngularSpeed * time_s), 0.0);
        const TVec3<double> expect_velocity =
            TVec3<double>(-kRadius * kAngularSpeed * std::sin(kAngularSpeed * time_s), kRadius * kAngularSpeed * std::cos(kAngularSpeed * time_s), 0.0);
        const TVec3<double> expect_acceleration =
            TVec3<double>(-kRadius * kAngularSpeed * kAngularSpeed * std::cos(kAngularSpeed * time_s),
                          -kRadius * kAngularSpeed * kAngularSpeed * std::sin(kAngularSpeed * time_s), 0.0);
        max_position_error = std::max(max_position_error, (position - expect_position).norm());
        max_velocity_error = std::max(max_velocity_error, (velocity - expect_velocity).norm());
        max_acceleration_error = std::max(max_acceleration_error, (acceleration - expect_acceleration).norm());
    }
    ReportInfo("Circular motion errors: " << max_position_error << " / " << max_velocity_error << " / " << max_acceleration_error);
    assert(max_position_error < 1e-4);
    assert(max_velocity_error < 1e-3);
    assert(max_acceleration_error < 0.1);
}

// The curve is C^2 at every interior knot: there must be no seam in value,
// velocity or acceleration between the spans on either side of a knot. A
// seam (span-indexing bug) would leave an O(1) jump that does not shrink as the
// sampling window h -> 0, whereas a genuinely C^2 curve shows jumps that vanish
// with h (value ~ O(h*|v|), velocity ~ O(h*|a|), acceleration ~ O(h*|da/dt|)).
// The window h is therefore taken tiny enough that the smooth variation falls
// far below any seam, and the tolerances are set between those two scales.
void TestC2Continuity() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.1;
    std::vector<TVec3<double>> controls;
    std::mt19937 random_engine(0);
    std::uniform_real_distribution<double> distribution(-5.0, 5.0);
    for (uint32_t i = 0; i < 30; ++i) {
        controls.emplace_back(distribution(random_engine), distribution(random_engine), distribution(random_engine));
    }

    LocalUniformCubicBSplineApproximator<TVec3<double>> spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));
    constexpr double kTinyWindowS = 1e-8;
    double max_value_jump = 0.0;
    double max_velocity_jump = 0.0;
    double max_acceleration_jump = 0.0;
    for (uint32_t i = 2; i + 2 < controls.size(); ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        TVec3<double> left_position;
        TVec3<double> left_velocity;
        TVec3<double> left_acceleration;
        TVec3<double> right_position;
        TVec3<double> right_velocity;
        TVec3<double> right_acceleration;
        assert(spline.GetValue(time_s - kTinyWindowS, left_position, left_velocity, left_acceleration));
        assert(spline.GetValue(time_s + kTinyWindowS, right_position, right_velocity, right_acceleration));
        // Smooth variation over 2*h: value ~ 2h*|v| (<= 1e-6 here), velocity ~
        // 2h*|a| (<= 1e-4 here), acceleration ~ 2h*|da/dt| (<= 1e-3 here).
        max_value_jump = std::max(max_value_jump, (left_position - right_position).norm());
        max_velocity_jump = std::max(max_velocity_jump, (left_velocity - right_velocity).norm());
        max_acceleration_jump = std::max(max_acceleration_jump, (left_acceleration - right_acceleration).norm());
    }
    ReportInfo("C2 seam checks over +-" << kTinyWindowS << " s: value " << max_value_jump << " / velocity " << max_velocity_jump
                                        << " / acceleration " << max_acceleration_jump);
    // A seam would be O(1); all smooth variation here is O(1e-4) or below.
    assert(max_value_jump < 1e-4);
    assert(max_velocity_jump < 1e-3);
    assert(max_acceleration_jump < 1e-1);
}

// The spline must scale to very long trajectories: O(N) build and O(1) queries.
void TestLongTrajectoryScale() {
    constexpr double kStartTimeS = 0.0;
    constexpr double kIntervalS = 0.01;
    constexpr uint32_t kNumControls = 100000;
    std::vector<TVec3<double>> controls;
    controls.reserve(kNumControls);
    for (uint32_t i = 0; i < kNumControls; ++i) {
        const double time_s = kStartTimeS + i * kIntervalS;
        controls.emplace_back(std::sin(0.1 * time_s), std::cos(0.1 * time_s), 0.05 * time_s);
    }

    const auto begin_fit = std::chrono::steady_clock::now();
    LocalUniformCubicBSplineApproximator<TVec3<double>> spline;
    assert(spline.Fit(kStartTimeS, kIntervalS, controls));
    const double fit_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin_fit).count();

    std::mt19937 random_engine(1);
    std::uniform_real_distribution<double> distribution(kStartTimeS + 1.0, spline.end_time_stamp_s() - 1.0);
    const auto begin_query = std::chrono::steady_clock::now();
    TVec3<double> value;
    TVec3<double> first;
    TVec3<double> second;
    for (uint32_t i = 0; i < 100000; ++i) {
        assert(spline.GetValue(distribution(random_engine), value, first, second));
        assert(value.allFinite() && first.allFinite() && second.allFinite());
    }
    const double query_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin_query).count();
    ReportInfo("Long trajectory: " << kNumControls << " controls, fit " << fit_time_s << " s, 100k queries " << query_time_s << " s.");
    assert(fit_time_s < 5.0);
    assert(query_time_s < 5.0);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test local uniform cubic B-spline approximator." RESET_COLOR);
    TestLinearPrecision();
    TestApproximationConvergence();
    TestCircularMotion();
    TestC2Continuity();
    TestLongTrajectoryScale();
    ReportInfo(GREEN ">> Local uniform cubic B-spline approximator test passed." RESET_COLOR);
    return 0;
}
