#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

namespace SLAM_UTILITY {

/* Class LineSegment2D Declaration. */
class LineSegment2D {

public:
    LineSegment2D() = default;
    virtual ~LineSegment2D() = default;

    Vec2 start() { return param_.head<2>(); }
    Vec2 end() { return param_.tail<2>(); }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    Vec4 param_ = Vec4::Zero();
};

/* Predeclaration for Class LineOrthonormal3D. */
class LineOrthonormal3D;

/* Class LinePlucker3D Declaration. */
class LinePlucker3D {

public:
    LinePlucker3D() = default;
    explicit LinePlucker3D(const Vec6 &param) : param_(param.normalized()) {}
    explicit LinePlucker3D(const LineOrthonormal3D &line);
    LinePlucker3D(const Vec3 &n, const Vec3 &d);
    virtual ~LinePlucker3D() = default;

    // Parameters of Plucker Line.
    Vec3 normal_vector() const { return param_.head<3>(); }
    Vec3 direction_vector() const { return param_.tail<3>(); }
    float distance() const { return normal_vector().norm() / direction_vector().norm(); }

    // Operations.
    bool SelfCheck() const { return std::fabs(normal_vector().dot(direction_vector())) < kZero; }
    void Normalize();
    LinePlucker3D TransformTo(const Quat &q_wc, const Vec3 &p_wc) const;
    Vec3 ProjectToNormalPlane() const;
    Vec3 ProjectToImagePlane(const float fx, const float fy, const float cx, const float cy) const;

    // Reference for member variables.
    Vec6 &param() { return param_; }
    // Const reference for member variables.
    const Vec6 &param() const { return param_; }

private:
    Vec6 param_ = Vec6::Ones();

};

/* Class LineOrthonormal3D Declaration. */
class LineOrthonormal3D {

public:
    LineOrthonormal3D() = default;
    explicit LineOrthonormal3D(const Vec4 &param) : param_(param) {}
    explicit LineOrthonormal3D(const LinePlucker3D &line);
    LineOrthonormal3D(const Vec3 &vector_u, const Vec1 &vector_w);

    Mat3 matrix_U() const;
    Mat2 matrix_W() const;
    Vec3 vector_u() const { return param_.head<3>(); }
    Vec1 vector_w() const { return param_.tail<1>(); }

    // Reference for member variables.
    Vec4 &param() { return param_; }
    // Const reference for member variables.
    const Vec4 &param() const { return param_; }

private:
    Vec4 param_ = Vec4::Zero();
};

}

#endif // end of _LINE_SEGMENT_H_