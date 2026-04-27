#ifndef _QUADRATIC_PLANE_H_
#define _QUADRATIC_PLANE_H_

#include "basic_type.h"

namespace slam_utility {

/* Class QuadraticPlane Declaration. */
class QuadraticPlane {
/* Quadratic plane model: ax^2 + by^2 + cz^2 + dxy + exz + fyz + gx + hy + iz + j = 0
   Note:
    - a, b, c: quadratic coefficients.
    - d, e, f: cross terms coefficients.
    - g, h, i: linear terms coefficients.
    - j: constant term.
    - x, y, z: axis coordinates of points.
*/
public:
    struct Curvatures {
        float gaussian_curvature = 0.0f;
        float mean_curvature = 0.0f;
        float max_principal_curvature = 0.0f;
        float min_principal_curvature = 0.0f;
    };

public:
    QuadraticPlane() = default;
    explicit QuadraticPlane(const Vec10 &param): param_(param) {}
    explicit QuadraticPlane(const std::vector<Vec3> &points);
    virtual ~QuadraticPlane() = default;

    // Operations.
    bool FitModel(const std::vector<Vec3> &points);
    bool FitParabolicModel(const std::vector<Vec3> &points);
    bool ComputeCurvaturesAtPoint(const Vec3 &point, Curvatures &curvatures) const;

    // Define operators.
    bool operator==(const QuadraticPlane &other) const { return param_ == other.param_; }
    bool operator!=(const QuadraticPlane &other) const { return param_ != other.param_; }

    // Reference for member variables.
    Vec10 &param() { return param_; }
    // Const reference for member variables.
    const Vec10 &param() const { return param_; }

private:
    // Parameters of plane. [a, b, c, d, e, f, g, h, i, j]
    Vec10 param_ = Vec10::Ones();
};

}  // namespace slam_utility

#endif  // end of _QUADRATIC_PLANE_H_
