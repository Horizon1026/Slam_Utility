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
    bool IsFitted() const { return rotation_vector_spline_.IsFitted(); }

    const double &start_time_stamp_s() const { return rotation_vector_spline_.start_time_stamp_s(); }
    const double &end_time_stamp_s() const { return rotation_vector_spline_.end_time_stamp_s(); }
    const double &time_interval_s() const { return rotation_vector_spline_.time_interval_s(); }

private:
    bool GetAngularVelocity(const double time_stamp_s, TVec3<double> &angular_velocity) const;

private:
    // The reference orientation maps spline coordinates from the local tangent
    // space into SO(3): R(t) = R_0 * Exp(phi(t)).
    TQuat<double> reference_orientation_ = TQuat<double>::Identity();
    CubicUniformBSpline<TVec3<double>> rotation_vector_spline_;
};

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_CUBIC_UNIFORM_SO3_BSPLINE_H_
