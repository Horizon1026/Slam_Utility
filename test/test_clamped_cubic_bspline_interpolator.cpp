#include "clamped_cubic_bspline_interpolator.h"
#include "slam_log_reporter.h"

#include "cassert"
#include "cmath"
#include "limits"
#include "vector"

using namespace slam_utility;

double CubicFunction(double time_stamp_s) { return time_stamp_s * time_stamp_s * time_stamp_s - 2.0 * time_stamp_s * time_stamp_s + 3.0 * time_stamp_s + 1.0; }

void TestScalarSpline() {
    const std::vector<double> time_stamp_s {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> values;
    for (const double time_stamp: time_stamp_s) {
        values.push_back(CubicFunction(time_stamp));
    }

    ClampedCubicBSplineInterpolator<double> spline;
    assert(spline.Fit(time_stamp_s, values));
    for (uint32_t i = 0; i < time_stamp_s.size(); ++i) {
        double value = 0.0;
        double first_derivative = 0.0;
        double second_derivative = 0.0;
        assert(spline.GetValue(time_stamp_s[i], value, first_derivative, second_derivative));
        assert(std::fabs(value - values[i]) < 1e-9);
    }
    for (const double query_time_stamp_s: std::vector<double> {0.5, 1.5, 2.75, 4.25}) {
        constexpr double kStep = 1e-4;
        double value = 0.0;
        double first_derivative = 0.0;
        double second_derivative = 0.0;
        double before_value = 0.0;
        double after_value = 0.0;
        assert(spline.GetValue(query_time_stamp_s, value, first_derivative, second_derivative));
        assert(spline.GetValue(query_time_stamp_s - kStep, before_value));
        assert(spline.GetValue(query_time_stamp_s + kStep, after_value));
        assert(std::fabs(first_derivative - (after_value - before_value) / (2.0 * kStep)) < 1e-7);
        assert(std::fabs(second_derivative - (after_value - 2.0 * value + before_value) / (kStep * kStep)) < 1e-5);
    }
    double value = 0.0;
    double first_derivative = 0.0;
    assert(spline.GetValue(2.0, value, first_derivative));
    assert(std::fabs(value - values[2]) < 1e-9);
    assert(!spline.GetValue(-0.01, values[0]));
    assert(!spline.GetValue(5.01, values[0]));
}

void TestVectorSpline() {
    const std::vector<double> time_stamp_s {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<Vec3> values;
    for (const double time_stamp: time_stamp_s) {
        values.emplace_back(CubicFunction(time_stamp), 2.0 * CubicFunction(time_stamp), -CubicFunction(time_stamp));
    }

    ClampedCubicBSplineInterpolator<Vec3> spline;
    assert(spline.Fit(time_stamp_s, values));
    for (uint32_t i = 0; i < time_stamp_s.size(); ++i) {
        Vec3 value = Vec3::Zero();
        assert(spline.GetValue(time_stamp_s[i], value));
        assert((value - values[i]).norm() < 1e-5);
    }
    Vec3 value = Vec3::Zero();
    Vec3 first_derivative = Vec3::Zero();
    Vec3 second_derivative = Vec3::Zero();
    const double query_time_stamp_s = 2.5;
    assert(spline.GetValue(query_time_stamp_s, value, first_derivative, second_derivative));
    assert(value.allFinite());
    assert(first_derivative.allFinite());
    assert(second_derivative.allFinite());
}

void TestInvalidInput() {
    ClampedCubicBSplineInterpolator<double> spline;
    assert(!spline.Fit({0.0, 1.0, 2.0}, {0.0, 1.0, 2.0}));
    assert(!spline.Fit({0.0, 1.0, 2.5, 3.5}, {0.0, 1.0, 2.5, 3.5}));

    const std::vector<double> valid_time_stamp_s {0.0, 1.0, 2.0, 3.0};
    const std::vector<double> valid_values {1.0, 2.0, 3.0, 4.0};
    assert(spline.Fit(valid_time_stamp_s, valid_values));
    const std::vector<double> control_points_before = spline.control_points();
    assert(!spline.Fit({0.0, 1.0, std::numeric_limits<double>::quiet_NaN(), 3.0}, valid_values));
    assert(spline.IsFitted());
    assert(spline.control_points() == control_points_before);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test clamped cubic B-spline interpolator." RESET_COLOR);
    TestScalarSpline();
    TestVectorSpline();
    TestInvalidInput();
    ReportInfo(GREEN ">> All clamped cubic B-spline interpolator tests passed." RESET_COLOR);
    return 0;
}
