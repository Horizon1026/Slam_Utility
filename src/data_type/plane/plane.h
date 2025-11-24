#ifndef _PLANE_3D_H_
#define _PLANE_3D_H_

#include "basic_type.h"
#include "normal_distribution.h"

namespace slam_utility {

/* Class Plane3D Declaration. */
class Plane3D {

public:
    Plane3D() = default;
    explicit Plane3D(const Vec4 &param): param_(param) {}
    explicit Plane3D(const std::vector<Vec3> &points);
    Plane3D(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3);
    virtual ~Plane3D() = default;

    // Operations.
    bool FitPlaneModel(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3);
    bool FitPlaneModelLse(const std::vector<Vec3> &points);
    bool FitPlaneModelPca(const std::vector<Vec3> &points);
    void GeneratePlaneModelParameters();
    bool AddNewPointToFitPlaneModel(const Vec3 &new_p_w);
    float GetDistanceToPlane(const Vec3 &p_w) const;

    // Parameters of plane.
    Vec3 normal_vector() const { return param_.head<3>(); }
    float distance_to_origin() const { return param_.tail<1>().value(); }

    // Define operators.
    bool operator==(const Plane3D &other) const { return param_ == other.param_; }
    bool operator!=(const Plane3D &other) const { return param_ != other.param_; }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    NormalDistribution<3> normal_distribution() { return normal_distribution_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }
    const NormalDistribution<3> normal_distribution() const { return normal_distribution_; }

private:
    static bool ComputeMidPoint(const std::vector<Vec3> &points, Vec3 &mid_point);

private:
    // Parameters of plane. [a, b, c, d] -> [normal_vector, distance_to_origin]
    Vec4 param_ = Vec4::Ones();
    NormalDistribution<3> normal_distribution_;
};

}  // namespace slam_utility

#endif  // end of _PLANE_3D_H_
