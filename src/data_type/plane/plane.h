#ifndef _PLANE_3D_H_
#define _PLANE_3D_H_

#include "basic_type.h"

namespace SLAM_UTILITY {

/* Class Plane3D Declaration. */
class Plane3D {

public:
    Plane3D() = default;
    explicit Plane3D(const Vec4 &param)
        : param_(param) {}
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
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    bool ComputeMidPoint(const std::vector<Vec3> &points, Vec3 &mid_point);

private:
    // Parameters of plane.
    Vec4 param_ = Vec4::Ones();
    // Mid point of points.
    Vec3 mid_point_ = Vec3::Zero();
    // Covariance matrix of points.
    Mat3 covariance_ = Mat3::Zero();
    // Number of points fitting plane.
    uint32_t num_of_points_fitting_plane_ = 0;
};

}  // namespace SLAM_UTILITY

#endif  // end of _PLANE_H_
