#ifndef _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_SO3_BSPLINE_APPROXIMATOR_H_
#define _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_SO3_BSPLINE_APPROXIMATOR_H_

#include "basic_type.h"

#include "vector"

namespace slam_utility {

/* Local Uniform Cubic SO(3) B-Spline Approximator.
 *
 * A local-support, fixed-resolution uniform cubic B-spline on SO(3). Like
 * LocalUniformCubicBSplineApproximator<T> it uses the supplied orientations DIRECTLY as
 * control points (no interpolation solve), so the fitted curve is a smooth
 * APPROXIMATION of the samples and the number of control points may grow
 * without bound for arbitrarily long trajectories (O(1) evaluation after a
 * constant-time span lookup).
 *
 * Representation:
 *  - On the span [T_j, T_{j+1}] the active orientations are the four
 *    neighbouring controls {j-1, j, j+1, j+2} (clamped to [0, N-1]).
 *  - With c0..c3 those controls and consecutive increments
 *      phi_m = Log(c_{m-1}^{-1} * c_m), m = 1..3,
 *    the curve is the cumulative Lie-group form
 *      q(u) = c0 * Exp(w1(u) * phi1) * Exp(w2(u) * phi2) * Exp(w3(u) * phi3),
 *    where the cumulative weights derive from the uniform cubic scalar basis
 *    b0..b3 as w1 = b1+b2+b3, w2 = b2+b3, w3 = b3. This reduces exactly to the
 *    vector LocalUniformCubicBSplineApproximator when rotations commute about a fixed axis.
 *  - The body-frame angular velocity is obtained from the same right-trivialized
 *    recurrence used by ClampedCubicSO3BSplineInterpolator, restricted to the active span.
 *    The angular acceleration is computed by central finite difference of the
 *    analytic angular velocity.
 *
 * Consecutive control orientations are normalized and made hemisphere-
 * continuous in Fit, so quaternions and their logarithms stay continuous.
 */
class LocalUniformCubicSO3BSplineApproximator {

public:
    LocalUniformCubicSO3BSplineApproximator() = default;
    virtual ~LocalUniformCubicSO3BSplineApproximator() = default;

    // Stores the supplied orientations as control points on a uniform grid.
    // A failed call leaves the last successfully fitted spline unchanged.
    bool Fit(const double start_time_s, const double interval_s, const std::vector<TQuat<double>> &controls);

    // Getters. angular_velocity and angular_acceleration are body-frame, rad/s and rad/s^2.
    bool GetValue(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity, TVec3<double> &angular_acceleration) const;
    bool GetValue(const double time_stamp_s, TQuat<double> &orientation) const;
    bool IsFitted() const { return !controls_.empty(); }

    // Const reference for member variables.
    const double &start_time_stamp_s() const { return start_time_s_; }
    double end_time_stamp_s() const { return start_time_s_ + static_cast<double>(controls_.size() - 1) * interval_s_; }
    const double &time_interval_s() const { return interval_s_; }

private:
    static void CalculateBasisAndFirstDerivative(double u, double (&basis)[4], double (&first_basis)[4]);
    // Evaluates orientation and body-frame angular velocity (rad/s) at a valid time.
    bool Evaluate(const double time_stamp_s, TQuat<double> &orientation, TVec3<double> &angular_velocity) const;
    // Central finite difference of the analytic angular velocity to obtain the
    // body-frame angular acceleration (rad/s^2).
    bool GetAngularAcceleration(const double time_stamp_s, TVec3<double> &angular_acceleration) const;

private:
    std::vector<TQuat<double>> controls_;
    double start_time_s_ = 0.0;
    double interval_s_ = 0.0;
};

}  // namespace slam_utility

#endif  // _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_SO3_BSPLINE_APPROXIMATOR_H_
