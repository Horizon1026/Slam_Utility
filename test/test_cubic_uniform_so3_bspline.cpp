#include "cassert"
#include "cmath"
#include "cubic_uniform_so3_bspline.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "vector"

using namespace slam_utility;

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
    ReportInfo(GREEN ">> All cubic uniform SO(3) B-spline tests passed." RESET_COLOR);
    return 0;
}
