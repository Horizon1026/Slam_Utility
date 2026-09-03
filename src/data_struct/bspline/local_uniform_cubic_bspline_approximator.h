#ifndef _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_BSPLINE_APPROXIMATOR_H_
#define _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_BSPLINE_APPROXIMATOR_H_

#include "basic_type.h"
#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "type_traits"
#include "vector"

namespace slam_utility {

/* Local Uniform Cubic B-Spline Approximator.
 *
 * This is a local-support, fixed-resolution uniform cubic B-spline whose
 * supplied samples are used DIRECTLY as control points (no interpolation
 * solve). The fitted curve is therefore a smooth APPROXIMATION of the
 * samples rather than an exact interpolation of them. Unlike the global
 * interpolator ClampedCubicBSplineInterpolator<T>, building the spline is O(N)
 * and every evaluation touches at most four neighbouring control points
 * (O(1) after a constant-time span lookup), so the number of control points
 * may grow without bound for arbitrarily long trajectories.
 *
 * Convention:
 *  - Fit(start_time_s, interval_s, controls) stores N controls located on the
 *    uniform grid T_i = start_time_s + i * interval_s, i = 0 .. N-1.
 *  - The valid query domain is exactly [start_time_s, start_time_s + (N-1)*dt].
 *  - Time t in the span [T_j, T_{j+1}] (u = (t - T_j)/dt in [0, 1]) is
 *    evaluated from the four neighbouring controls {j-1, j, j+1, j+2}
 *    (clamped to [0, N-1]), which is equivalent to duplicating one ghost
 *    control at each end. The curve is globally C^2 at every interior knot;
 *    only the outermost spans are slightly biased by the duplicated ghost,
 *    which is acceptable because endpoint exactness is not required.
 *  - Value, first and second derivative are returned in units of T, T/s and
 *    T/s^2 respectively.
 */
template <typename T>
class LocalUniformCubicBSplineApproximator {

public:
    LocalUniformCubicBSplineApproximator() = default;
    virtual ~LocalUniformCubicBSplineApproximator() = default;

    // Stores the supplied samples as control points on a uniform grid.
    // A failed call leaves the last successfully fitted spline unchanged.
    bool Fit(const double start_time_s, const double interval_s, const std::vector<T> &controls);

    // Getters. first_derivative and second_derivative are w.r.t. time in seconds.
    bool GetValue(const double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const;
    bool GetValue(const double time_stamp_s, T &value, T &first_derivative) const;
    bool GetValue(const double time_stamp_s, T &value) const;
    bool IsFitted() const { return !controls_.empty(); }

    // Const reference for member variables.
    const double &start_time_stamp_s() const { return start_time_s_; }
    double end_time_stamp_s() const { return start_time_s_ + static_cast<double>(controls_.size() - 1) * interval_s_; }
    const double &time_interval_s() const { return interval_s_; }

private:
    // Standard uniform cubic B-spline basis, its first and second derivative
    // with respect to the local parameter u, for the four active controls.
    static void CalculateWeights(const double u, double (&basis)[4], double (&first_basis)[4], double (&second_basis)[4]);

    static bool IsFiniteValue(const T &value) {
        if constexpr (std::is_arithmetic_v<T>) {
            return std::isfinite(static_cast<double>(value));
        } else {
            return value.allFinite();
        }
    }

private:
    double start_time_s_ = 0.0;
    double interval_s_ = 0.0;
    std::vector<T> controls_;
};

/* Local Uniform Cubic B-Spline Approximator definition. */
template <typename T>
bool LocalUniformCubicBSplineApproximator<T>::Fit(const double start_time_s, const double interval_s, const std::vector<T> &controls) {
    RETURN_FALSE_IF(!std::isfinite(start_time_s) || !std::isfinite(interval_s) || interval_s <= 0.0);
    RETURN_FALSE_IF(controls.size() < 4);
    for (const T &control: controls) {
        RETURN_FALSE_IF(!IsFiniteValue(control));
    }

    LocalUniformCubicBSplineApproximator<T> candidate;
    candidate.start_time_s_ = start_time_s;
    candidate.interval_s_ = interval_s;
    candidate.controls_ = controls;
    *this = std::move(candidate);
    return true;
}

template <typename T>
bool LocalUniformCubicBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const {
    RETURN_FALSE_IF(!IsFitted());
    const double end_time_s = end_time_stamp_s();
    const double tolerance = 1e-9 * std::max(1.0, std::fabs(end_time_s - start_time_s_));
    RETURN_FALSE_IF(time_stamp_s < start_time_s_ - tolerance || time_stamp_s > end_time_s + tolerance);
    const double time_s = std::clamp(time_stamp_s, start_time_s_, end_time_s);

    const uint32_t num_controls = static_cast<uint32_t>(controls_.size());
    // Locate the span [T_span, T_span+1] that contains time_s. The final sample
    // time is evaluated from the left at u = 1 so the endpoint uses the last span.
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

    // Active control indices of this span, with one duplicated ghost control at
    // each end so the whole domain is covered.
    const uint32_t index[4] = {
        span > 0 ? span - 1 : 0,
        span,
        std::min(span + 1, num_controls - 1),
        std::min(span + 2, num_controls - 1),
    };

    double basis[4];
    double first_basis[4];
    double second_basis[4];
    CalculateWeights(u, basis, first_basis, second_basis);

    value = basis[0] * controls_[index[0]];
    T first_value = first_basis[0] * controls_[index[0]];
    T second_value = second_basis[0] * controls_[index[0]];
    for (uint32_t i = 1; i < 4; ++i) {
        value += basis[i] * controls_[index[i]];
        first_value += first_basis[i] * controls_[index[i]];
        second_value += second_basis[i] * controls_[index[i]];
    }
    // Basis derivatives are computed with respect to the local parameter u;
    // convert them to derivatives with respect to time in seconds.
    first_derivative = first_value / interval_s_;
    second_derivative = second_value / (interval_s_ * interval_s_);
    return true;
}

template <typename T>
bool LocalUniformCubicBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative) const {
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
bool LocalUniformCubicBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value) const {
    T first_derivative {};
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
void LocalUniformCubicBSplineApproximator<T>::CalculateWeights(const double u, double (&basis)[4], double (&first_basis)[4], double (&second_basis)[4]) {
    const double u2 = u * u;
    const double u3 = u2 * u;
    // Standard uniform cubic B-spline basis on u in [0, 1].
    basis[0] = (1.0 - u) * (1.0 - u) * (1.0 - u) / 6.0;
    basis[1] = (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0;
    basis[2] = (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0;
    basis[3] = u3 / 6.0;
    // First derivative with respect to u.
    first_basis[0] = -(1.0 - u) * (1.0 - u) / 2.0;
    first_basis[1] = (3.0 * u2 - 4.0 * u) / 2.0;
    first_basis[2] = (-3.0 * u2 + 2.0 * u + 1.0) / 2.0;
    first_basis[3] = u2 / 2.0;
    // Second derivative with respect to u.
    second_basis[0] = 1.0 - u;
    second_basis[1] = 3.0 * u - 2.0;
    second_basis[2] = 1.0 - 3.0 * u;
    second_basis[3] = u;
}

}  // namespace slam_utility

#endif  // _SLAM_UTILITY_LOCAL_UNIFORM_CUBIC_BSPLINE_APPROXIMATOR_H_
