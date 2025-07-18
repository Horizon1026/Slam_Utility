#ifndef _PLANE_3D_H_
#define _PLANE_3D_H_

#include "basic_type.h"

namespace SLAM_UTILITY {

/* Class Plane3D Declaration. */
class Plane3D {

public:
    Plane3D() = default;
    explicit Plane3D(const Vec4 &param) : param_(param) {}
    explicit Plane3D(const std::vector<Vec3> &points);
    Plane3D(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3);
    virtual ~Plane3D() = default;

    // Operations.
    bool FitPlaneModel(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3);
    bool FitPlaneModelLse(const std::vector<Vec3> &points);
    bool FitPlaneModelPca(const std::vector<Vec3> &points);
    float GetDistanceToPlane(const Vec3 &p_w) const;

    // Parameters of plane.
    Vec3 normal_vector() const { return param_.head<3>(); }
    float distance_to_origin() const { return param_.tail<1>().value(); }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    bool ComputeMidPoint(const std::vector<Vec3> &points, Vec3 &mid_point);

private:
    Vec4 param_ = Vec4::Ones();

};

}

#endif // end of _PLANE_H_
