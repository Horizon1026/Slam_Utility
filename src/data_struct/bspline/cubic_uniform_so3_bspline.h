#ifndef _SLAM_UTILITY_CUBIC_UNIFORM_SO3_BSPLINE_H_
#define _SLAM_UTILITY_CUBIC_UNIFORM_SO3_BSPLINE_H_

#include "cubic_uniform_bspline.h"
#include "slam_basic_math.h"

#include "vector"

namespace slam_utility {

/* Class Cubic Uniform SO(3) B-Spline Declaration. */
class CubicUniformSO3BSpline {

public:
    CubicUniformSO3BSpline() = default;
    virtual ~CubicUniformSO3BSpline() = default;

    bool Fit(const std::vector<double> &all_time_stamps_s, const std::vector<TQuat<double>> &all_orientations);
    bool GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity, TVec3<double> &angular_acceleration) const;
    bool GetValue(const double time_stamp_s, TQuat<double> &orientation) const;
    bool IsFitted() const { return !control_orientations_.empty(); }

    // Reference for member variables.
    double &start_time_stamp_s() { return start_time_stamp_s_; }
    double &end_time_stamp_s() { return end_time_stamp_s_; }
    double &time_interval_s() { return time_interval_s_; }
    // Const reference for member variables.
    const double &start_time_stamp_s() const { return start_time_stamp_s_; }
    const double &end_time_stamp_s() const { return end_time_stamp_s_; }
    const double &time_interval_s() const { return time_interval_s_; }

private:
    bool Evaluate(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity) const;
    bool GetAngularVelocity(const double time_stamp_s, TVec3<double> &angular_velocity) const;
    void CalculateBasis(const double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis) const;

private:
    // Cumulative Lie-group B-spline:
    // R(t) = R_0 * Product_j Exp(beta_j(t) * Log(R_j-1^-1 * R_j)).
    std::vector<TQuat<double>> control_orientations_;
    std::vector<double> knots_;
    double start_time_stamp_s_ = 0.0;
    double end_time_stamp_s_ = 0.0;
    double time_interval_s_ = 0.0;
};

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_CUBIC_UNIFORM_SO3_BSPLINE_H_
