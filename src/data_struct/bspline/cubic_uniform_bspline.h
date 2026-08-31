#ifndef _SLAM_UTILITY_CUBIC_UNIFORM_BSPLINE_H_
#define _SLAM_UTILITY_CUBIC_UNIFORM_BSPLINE_H_

#include "basic_type.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace slam_utility {

/* Class Cubic Uniform B-Spline Declaration.
 *
 * The spline uses a clamped, cubic uniform knot vector. Four repeated knots at
 * each end make the curve pass through the first and last samples. Fit()
 * computes control points that interpolate every supplied sample.
 */
template <typename T>
class CubicUniformBSpline {

public:
    CubicUniformBSpline() = default;
    virtual ~CubicUniformBSpline() = default;

    bool Fit(const std::vector<double> &time_stamp_s, const std::vector<T> &values);
    bool GetValue(double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const;
    bool GetValue(double time_stamp_s, T &value) const;

    bool IsFitted() const { return !control_points_.empty(); }
    double start_time_stamp_s() const { return start_time_stamp_s_; }
    double end_time_stamp_s() const { return end_time_stamp_s_; }
    double time_interval_s() const { return time_interval_s_; }

private:
    void CalculateBasis(double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis, std::vector<double> &second_basis) const;

private:
    std::vector<double> knots_;
    std::vector<T> control_points_;
    double start_time_stamp_s_ = 0.0;
    double end_time_stamp_s_ = 0.0;
    double time_interval_s_ = 0.0;
};

/* Class Cubic Uniform B-Spline Definition. */
template <typename T>
bool CubicUniformBSpline<T>::Fit(const std::vector<double> &time_stamp_s, const std::vector<T> &values) {
    if (time_stamp_s.size() != values.size() || values.size() < 4) {
        return false;
    }

    const double time_interval_s = time_stamp_s[1] - time_stamp_s[0];
    if (time_interval_s <= 0.0) {
        return false;
    }
    for (uint32_t i = 2; i < time_stamp_s.size(); ++i) {
        const double interval_s = time_stamp_s[i] - time_stamp_s[i - 1];
        const double tolerance = 1e-9 * std::max(1.0, std::fabs(time_interval_s));
        if (std::fabs(interval_s - time_interval_s) > tolerance) {
            return false;
        }
    }

    // A cubic B-spline requires four repeated end knots. The remaining knots are uniformly spaced, so each curve segment has the same time length.
    start_time_stamp_s_ = time_stamp_s.front();
    end_time_stamp_s_ = time_stamp_s.back();
    time_interval_s_ = time_interval_s;
    knots_.clear();
    knots_.insert(knots_.end(), 4, start_time_stamp_s_);
    const double knot_interval_s = (end_time_stamp_s_ - start_time_stamp_s_) / (values.size() - 3);
    for (uint32_t i = 1; i + 3 < values.size(); ++i) {
        knots_.push_back(start_time_stamp_s_ + i * knot_interval_s);
    }
    knots_.insert(knots_.end(), 4, end_time_stamp_s_);

    // At every sample time, S(t_i) = sum_j N_j,3(t_i) * P_j. Building these equations gives A * P = samples, where A contains cubic basis values.
    const uint32_t num_values = values.size();
    Eigen::MatrixXd interpolation_matrix(num_values, num_values);
    std::vector<double> basis;
    std::vector<double> first_basis;
    std::vector<double> second_basis;
    for (uint32_t i = 0; i < num_values; ++i) {
        CalculateBasis(time_stamp_s[i], basis, first_basis, second_basis);
        for (uint32_t j = 0; j < num_values; ++j) {
            interpolation_matrix(i, j) = basis[j];
        }
    }
    // Solve for the control points. Applying A^-1 to each value component also supports arbitrary value types, such as scalars and fixed-size vectors.
    const Eigen::MatrixXd inverse_matrix = interpolation_matrix.fullPivLu().inverse();
    if (!inverse_matrix.allFinite()) {
        control_points_.clear();
        return false;
    }

    control_points_.resize(num_values);
    for (uint32_t i = 0; i < num_values; ++i) {
        control_points_[i] = values[0] * inverse_matrix(i, 0);
        for (uint32_t j = 1; j < num_values; ++j) {
            control_points_[i] += values[j] * inverse_matrix(i, j);
        }
    }
    return true;
}

template <typename T>
bool CubicUniformBSpline<T>::GetValue(double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const {
    if (!IsFitted() || time_stamp_s < start_time_stamp_s_ || time_stamp_s > end_time_stamp_s_) {
        return false;
    }

    std::vector<double> basis;
    std::vector<double> first_basis;
    std::vector<double> second_basis;
    // A B-spline and its derivatives are linear combinations of the same control points, weighted by the corresponding basis derivatives.
    CalculateBasis(time_stamp_s, basis, first_basis, second_basis);
    value = control_points_[0] * basis[0];
    first_derivative = control_points_[0] * first_basis[0];
    second_derivative = control_points_[0] * second_basis[0];
    for (uint32_t i = 1; i < control_points_.size(); ++i) {
        value += control_points_[i] * basis[i];
        first_derivative += control_points_[i] * first_basis[i];
        second_derivative += control_points_[i] * second_basis[i];
    }
    return true;
}

template <typename T>
bool CubicUniformBSpline<T>::GetValue(double time_stamp_s, T &value) const {
    T first_derivative {};
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
void CubicUniformBSpline<T>::CalculateBasis(double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis,
                                            std::vector<double> &second_basis) const {
    // Cox-de Boor basis functions are conventionally half-open on each knot interval. Evaluate the final time infinitesimally from the left to obtain
    // the correct endpoint value and one-sided derivatives.
    const double evaluation_time_stamp_s = time_stamp_s == end_time_stamp_s_ ? std::nextafter(time_stamp_s, start_time_stamp_s_) : time_stamp_s;
    const uint32_t num_control_points = control_points_.empty() ? static_cast<uint32_t>(knots_.size() - 4) : control_points_.size();
    // Cox-de Boor recursion starts with piecewise-constant (degree-zero) basis functions and raises the degree one level at a time up to degree three.
    std::vector<std::vector<double>> basis_by_degree(4);
    basis_by_degree[0].resize(knots_.size() - 1, 0.0);
    for (uint32_t i = 0; i + 1 < knots_.size(); ++i) {
        if (evaluation_time_stamp_s >= knots_[i] && evaluation_time_stamp_s < knots_[i + 1]) {
            basis_by_degree[0][i] = 1.0;
        }
    }
    for (uint32_t degree = 1; degree <= 3; ++degree) {
        basis_by_degree[degree].resize(knots_.size() - degree - 1, 0.0);
        for (uint32_t i = 0; i < basis_by_degree[degree].size(); ++i) {
            const double left_denominator = knots_[i + degree] - knots_[i];
            const double right_denominator = knots_[i + degree + 1] - knots_[i + 1];
            if (left_denominator > 0.0) {
                basis_by_degree[degree][i] += (evaluation_time_stamp_s - knots_[i]) / left_denominator * basis_by_degree[degree - 1][i];
            }
            if (right_denominator > 0.0) {
                basis_by_degree[degree][i] += (knots_[i + degree + 1] - evaluation_time_stamp_s) / right_denominator * basis_by_degree[degree - 1][i + 1];
            }
        }
    }

    basis.assign(basis_by_degree[3].begin(), basis_by_degree[3].begin() + num_control_points);
    first_basis.assign(num_control_points, 0.0);
    second_basis.assign(num_control_points, 0.0);
    for (uint32_t i = 0; i < num_control_points; ++i) {
        const double left_denominator = knots_[i + 3] - knots_[i];
        const double right_denominator = knots_[i + 4] - knots_[i + 1];
        const double left_first = left_denominator > 0.0 ? 3.0 / left_denominator : 0.0;
        const double right_first = right_denominator > 0.0 ? 3.0 / right_denominator : 0.0;
        // dN_i,p/dt = p/(u_i+p-u_i) * N_i,p-1
        //              - p/(u_i+p+1-u_i+1) * N_i+1,p-1.
        first_basis[i] = left_first * basis_by_degree[2][i] - right_first * basis_by_degree[2][i + 1];

        const double left_left_denominator = knots_[i + 2] - knots_[i];
        const double left_right_denominator = knots_[i + 3] - knots_[i + 1];
        const double right_left_denominator = knots_[i + 3] - knots_[i + 1];
        const double right_right_denominator = knots_[i + 4] - knots_[i + 2];
        const double left_derivative = (left_left_denominator > 0.0 ? 2.0 / left_left_denominator * basis_by_degree[1][i] : 0.0) -
                                       (left_right_denominator > 0.0 ? 2.0 / left_right_denominator * basis_by_degree[1][i + 1] : 0.0);
        const double right_derivative = (right_left_denominator > 0.0 ? 2.0 / right_left_denominator * basis_by_degree[1][i + 1] : 0.0) -
                                        (right_right_denominator > 0.0 ? 2.0 / right_right_denominator * basis_by_degree[1][i + 2] : 0.0);
        // Apply the same derivative identity to the quadratic basis terms to obtain the second derivative of each cubic basis function.
        second_basis[i] = left_first * left_derivative - right_first * right_derivative;
    }
}

}  // namespace slam_utility

#endif  // end of _SLAM_UTILITY_CUBIC_UNIFORM_BSPLINE_H_
