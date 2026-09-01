#ifndef _SLAM_UTILITY_CUBIC_UNIFORM_BSPLINE_H_
#define _SLAM_UTILITY_CUBIC_UNIFORM_BSPLINE_H_

#include "algorithm"
#include "basic_type.h"
#include "cmath"
#include "slam_operations.h"
#include "type_traits"
#include "vector"

namespace slam_utility {

/* Class Cubic Uniform B-Spline Declaration */
template <typename T>
class CubicUniformBSpline {
    // The spline uses a clamped, cubic uniform knot vector.
    // Four repeated knots at each end make the curve pass through the first and last samples.

public:
    CubicUniformBSpline() = default;
    virtual ~CubicUniformBSpline() = default;

    // Computes control points that interpolate every supplied sample.
    // A failed fit leaves the last successfully fitted spline unchanged.
    bool Fit(const std::vector<double> &all_time_stamps_s, const std::vector<T> &all_values);

    // Getters.
    bool GetValue(const double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const;
    bool GetValue(const double time_stamp_s, T &value, T &first_derivative) const;
    bool GetValue(const double time_stamp_s, T &value) const;
    bool IsFitted() const { return !control_points_.empty(); }

    // Reference for member variables.
    std::vector<double> &knots() { return knots_; }
    std::vector<T> &control_points() { return control_points_; }
    double &start_time_stamp_s() { return start_time_stamp_s_; }
    double &end_time_stamp_s() { return end_time_stamp_s_; }
    double &time_interval_s() { return time_interval_s_; }
    // Const reference for member variables.
    const std::vector<double> &knots() const { return knots_; }
    const std::vector<T> &control_points() const { return control_points_; }
    const double &start_time_stamp_s() const { return start_time_stamp_s_; }
    const double &end_time_stamp_s() const { return end_time_stamp_s_; }
    const double &time_interval_s() const { return time_interval_s_; }

private:
    void CalculateBasis(const double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis, std::vector<double> &second_basis) const;

private:
    // Knot vector defining the spline's parameter intervals and basis functions.
    std::vector<double> knots_;
    // Control points combined by the basis functions to evaluate the spline.
    std::vector<T> control_points_;
    double start_time_stamp_s_ = 0.0;
    double end_time_stamp_s_ = 0.0;
    double time_interval_s_ = 0.0;
};

/* Class Cubic Uniform B-Spline Definition. */
template <typename T>
bool CubicUniformBSpline<T>::Fit(const std::vector<double> &all_time_stamps_s, const std::vector<T> &all_values) {
    RETURN_FALSE_IF(all_time_stamps_s.size() != all_values.size() || all_values.size() < 4);
    for (const double time_stamp_s: all_time_stamps_s) {
        RETURN_FALSE_IF(!std::isfinite(time_stamp_s));
    }

    const double time_interval_s = all_time_stamps_s[1] - all_time_stamps_s[0];
    RETURN_FALSE_IF(!std::isfinite(time_interval_s) || time_interval_s <= 0.0);
    for (uint32_t i = 2; i < all_time_stamps_s.size(); ++i) {
        const double interval_s = all_time_stamps_s[i] - all_time_stamps_s[i - 1];
        const double tolerance = 1e-9 * std::max(1.0, std::fabs(time_interval_s));
        RETURN_FALSE_IF(!std::isfinite(interval_s) || std::fabs(interval_s - time_interval_s) > tolerance);
    }

    // Build a candidate spline locally. No observable state is changed unless
    // node construction and the interpolation solve both succeed.
    CubicUniformBSpline<T> candidate;
    // Let N be the number of control points (not the maximum control-point index). A clamped cubic B-spline has N + 4 knots: four repeated knots at
    // each endpoint and N - 4 internal knots. These internal knots divide the parameter domain into N - 3 uniformly sized, non-zero knot spans.
    candidate.start_time_stamp_s_ = all_time_stamps_s.front();
    candidate.end_time_stamp_s_ = all_time_stamps_s.back();
    candidate.time_interval_s_ = time_interval_s;
    candidate.knots_.insert(candidate.knots_.end(), 4, candidate.start_time_stamp_s_);
    const double knot_interval_s = (candidate.end_time_stamp_s_ - candidate.start_time_stamp_s_) / (all_values.size() - 3);
    // Generate exactly N - 4 internal knots: i = 1, 2, ..., N - 4.
    for (uint32_t i = 1; i + 3 < all_values.size(); ++i) {
        candidate.knots_.push_back(candidate.start_time_stamp_s_ + i * knot_interval_s);
    }
    candidate.knots_.insert(candidate.knots_.end(), 4, candidate.end_time_stamp_s_);

    // At every sample time, S(t_i) = sum_j N_j,3(t_i) * P_j. Building these equations gives A * P = samples, where A contains cubic basis values.
    const uint32_t num_values = all_values.size();
    TMat<double> interpolation_matrix(num_values, num_values);
    std::vector<double> basis;
    std::vector<double> first_basis;
    std::vector<double> second_basis;
    for (uint32_t i = 0; i < num_values; ++i) {
        candidate.CalculateBasis(all_time_stamps_s[i], basis, first_basis, second_basis);
        for (uint32_t j = 0; j < num_values; ++j) {
            interpolation_matrix(i, j) = basis[j];
        }
    }
    uint32_t value_dimension = 1;
    // Arithmetic values are one-dimensional scalars; other supported values are Eigen vectors whose dimension is determined by the input sample.
    if constexpr (!std::is_arithmetic_v<T>) {
        value_dimension = static_cast<uint32_t>(all_values.front().size());
    }
    TMat<double> value_matrix(num_values, value_dimension);
    for (uint32_t i = 0; i < num_values; ++i) {
        if constexpr (std::is_arithmetic_v<T>) {
            value_matrix(i, 0) = static_cast<double>(all_values[i]);
        } else {
            for (uint32_t j = 0; j < value_dimension; ++j) {
                value_matrix(i, j) = static_cast<double>(all_values[i][j]);
            }
        }
    }

    // Factor once and solve A * P = values directly. Avoid explicitly forming
    // A^-1, reject rank-deficient/ill-conditioned systems, and verify the solve.
    auto decomposition = interpolation_matrix.fullPivLu();
    constexpr double kMinimumReciprocalCondition = 1e-12;
    const double reciprocal_condition = decomposition.rcond();
    RETURN_FALSE_IF(!decomposition.isInvertible() || !std::isfinite(reciprocal_condition) || reciprocal_condition < kMinimumReciprocalCondition);
    const TMat<double> solution = decomposition.solve(value_matrix);
    RETURN_FALSE_IF(!solution.allFinite());
    const double residual_norm = (interpolation_matrix * solution - value_matrix).norm();
    constexpr double kRelativeResidualTolerance = 1e-10;
    RETURN_FALSE_IF(residual_norm > kRelativeResidualTolerance * std::max(1.0, value_matrix.norm()));

    candidate.control_points_.resize(num_values);
    for (uint32_t i = 0; i < num_values; ++i) {
        if constexpr (std::is_arithmetic_v<T>) {
            candidate.control_points_[i] = static_cast<T>(solution(i, 0));
        } else {
            candidate.control_points_[i] = solution.row(i).transpose().template cast<typename T::Scalar>();
        }
    }

    knots_ = std::move(candidate.knots_);
    control_points_ = std::move(candidate.control_points_);
    start_time_stamp_s_ = candidate.start_time_stamp_s_;
    end_time_stamp_s_ = candidate.end_time_stamp_s_;
    time_interval_s_ = candidate.time_interval_s_;
    return true;
}

template <typename T>
bool CubicUniformBSpline<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative, T &second_derivative) const {
    RETURN_FALSE_IF(!IsFitted() || time_stamp_s < start_time_stamp_s_ || time_stamp_s > end_time_stamp_s_);

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
bool CubicUniformBSpline<T>::GetValue(const double time_stamp_s, T &value, T &first_derivative) const {
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
bool CubicUniformBSpline<T>::GetValue(const double time_stamp_s, T &value) const {
    T first_derivative {};
    T second_derivative {};
    return GetValue(time_stamp_s, value, first_derivative, second_derivative);
}

template <typename T>
void CubicUniformBSpline<T>::CalculateBasis(const double time_stamp_s, std::vector<double> &basis, std::vector<double> &first_basis,
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
