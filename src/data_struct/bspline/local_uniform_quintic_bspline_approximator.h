#ifndef _SLAM_UTILITY_LOCAL_UNIFORM_QUINTIC_BSPLINE_APPROXIMATOR_H_
#define _SLAM_UTILITY_LOCAL_UNIFORM_QUINTIC_BSPLINE_APPROXIMATOR_H_

#include "basic_type.h"
#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "type_traits"
#include "vector"

namespace slam_utility {

/* Local Uniform Quintic B-Spline Approximator.
 *
 * This is the fifth-degree counterpart of LocalUniformCubicBSplineApproximator.
 * Supplied samples are used directly as uniformly spaced control points, so
 * fitting is O(N) and each query only touches six neighbouring controls. A
 * quintic B-spline is globally C^4: position, velocity, acceleration and jerk
 * are continuous. In particular, its acceleration is cubic on each span rather
 * than piecewise linear, making it suitable for smooth IMU specific force.
 */
template <typename T>
class LocalUniformQuinticBSplineApproximator {

public:
    LocalUniformQuinticBSplineApproximator() = default;
    virtual ~LocalUniformQuinticBSplineApproximator() = default;

    bool Fit(double start_time_s, double interval_s, const std::vector<T> &controls);
    bool GetValue(double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const;
    bool GetValue(double time_stamp_s, T &value, T &first_derivative) const;
    bool GetValue(double time_stamp_s, T &value) const;
    bool IsFitted() const { return !controls_.empty(); }

    const double &start_time_stamp_s() const { return start_time_s_; }
    double end_time_stamp_s() const { return start_time_s_ + static_cast<double>(controls_.size() - 1) * interval_s_; }
    const double &time_interval_s() const { return interval_s_; }

private:
    void CalculateWeights(double u, double (&basis)[6], double (&first_basis)[6], double (&second_basis)[6]) const;
    double CardinalBasis(double x, uint32_t derivative_order) const;

    bool IsFiniteValue(const T &value) const {
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

template <typename T>
bool LocalUniformQuinticBSplineApproximator<T>::Fit(const double start_time_s, const double interval_s, const std::vector<T> &controls) {
    RETURN_FALSE_IF(!std::isfinite(start_time_s) || !std::isfinite(interval_s) || interval_s <= 0.0);
    RETURN_FALSE_IF(controls.size() < 6);
    for (const T &control: controls) {
        RETURN_FALSE_IF(!IsFiniteValue(control));
    }
    LocalUniformQuinticBSplineApproximator<T> candidate;
    candidate.start_time_s_ = start_time_s;
    candidate.interval_s_ = interval_s;
    candidate.controls_ = controls;
    *this = std::move(candidate);
    return true;
}

template <typename T>
bool LocalUniformQuinticBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const {
    RETURN_FALSE_IF(!IsFitted());
    const double end_time_s = end_time_stamp_s();
    const double tolerance = 1e-9 * std::max(1.0, std::fabs(end_time_s - start_time_s_));
    RETURN_FALSE_IF(time_stamp_s < start_time_s_ - tolerance || time_stamp_s > end_time_s + tolerance);
    const double time_s = std::clamp(time_stamp_s, start_time_s_, end_time_s);

    const uint32_t num_controls = static_cast<uint32_t>(controls_.size());
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

    const uint32_t index[6] = {
        span > 1 ? span - 2 : 0,
        span > 0 ? span - 1 : 0,
        span,
        std::min(span + 1, num_controls - 1),
        std::min(span + 2, num_controls - 1),
        std::min(span + 3, num_controls - 1),
    };
    double basis[6];
    double first_basis[6];
    double second_basis[6];
    CalculateWeights(u, basis, first_basis, second_basis);

    value = basis[0] * controls_[index[0]];
    T first_value = first_basis[0] * controls_[index[0]];
    T second_value = second_basis[0] * controls_[index[0]];
    for (uint32_t i = 1; i < 6; ++i) {
        value += basis[i] * controls_[index[i]];
        first_value += first_basis[i] * controls_[index[i]];
        second_value += second_basis[i] * controls_[index[i]];
    }
    first_derivative = first_value / interval_s_;
    second_derivative = second_value / (interval_s_ * interval_s_);
    return true;
}

template <typename T>
bool LocalUniformQuinticBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative) const {
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
bool LocalUniformQuinticBSplineApproximator<T>::GetValue(const double time_stamp_s, T &value) const {
    T first_derivative {};
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
void LocalUniformQuinticBSplineApproximator<T>::CalculateWeights(const double u, double (&basis)[6], double (&first_basis)[6],
                                                                 double (&second_basis)[6]) const {
    // On [T_j, T_j+1], the six controls are j-2 through j+3. Cardinal B_5
    // evaluated at u+5 down to u supplies their uniform quintic weights.
    for (uint32_t i = 0; i < 6; ++i) {
        const double x = u + 5.0 - static_cast<double>(i);
        basis[i] = CardinalBasis(x, 0);
        first_basis[i] = CardinalBasis(x, 1);
        second_basis[i] = CardinalBasis(x, 2);
    }
}

template <typename T>
double LocalUniformQuinticBSplineApproximator<T>::CardinalBasis(const double x, const uint32_t derivative_order) const {
    constexpr double kBinomial[7] = {1.0, 6.0, 15.0, 20.0, 15.0, 6.0, 1.0};
    constexpr uint32_t kDegree = 5;
    if (derivative_order > 2) {
        return 0.0;
    }
    const uint32_t power = kDegree - derivative_order;
    double factorial = 1.0;
    for (uint32_t i = 2; i <= power; ++i) {
        factorial *= static_cast<double>(i);
    }
    double value = 0.0;
    for (uint32_t i = 0; i <= kDegree + 1; ++i) {
        const double shifted_x = x - static_cast<double>(i);
        CONTINUE_IF(shifted_x <= 0.0);
        const double sign = i % 2 == 0 ? 1.0 : -1.0;
        value += sign * kBinomial[i] * std::pow(shifted_x, static_cast<int>(power));
    }
    return value / factorial;
}

}  // namespace slam_utility

#endif  // _SLAM_UTILITY_LOCAL_UNIFORM_QUINTIC_BSPLINE_APPROXIMATOR_H_
