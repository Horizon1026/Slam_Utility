#ifndef _3D_GAUSSIAN_H_
#define _3D_GAUSSIAN_H_

#include "datatype_basic.h"
#include "datatype_rgbcolor.h"
#include "2d_gaussian.h"

namespace SLAM_UTILITY {

/* Class Gaussian3D Declaration. */
class Gaussian3D {

public:
    Gaussian3D() = default;
    virtual ~Gaussian3D() = default;

    void ProjectTo2D(const Vec3 &p_wc, const Quat &q_wc,
                     Vec2 &uv_2d, Mat2 &sigma_2d, float &mid_opacity_2d, float &depth);

    void ProjectTo2D(const Vec3 &p_wc, const Quat &q_wc, Gaussian2D &gaussian_2d);

    // Reference for member variables.
    Vec3 &p_w() { return p_w_; }
    Vec3 &sigma_s() { return sigma_s_; }
    Quat &sigma_q() { return sigma_q_; }
    float &mid_opacity() { return mid_opacity_; }
    RgbPixel &color() { return color_; }

    // Const reference for member variables.
    const Vec3 &p_w() const { return p_w_; }
    const Vec3 &sigma_s() const { return sigma_s_; }
    const Quat &sigma_q() const { return sigma_q_; }
    const float &mid_opacity() const { return mid_opacity_; }
    const RgbPixel &color() const { return color_; }

private:
    // Position and sigma matrin in world frame.
    Vec3 p_w_ = Vec3::Zero();
    Vec3 sigma_s_ = Vec3::Ones();
    Quat sigma_q_ = Quat::Identity();
    // Opacity of mid point.
    float mid_opacity_ = 0.0f;
    // Color of mid point.
    RgbPixel color_;
};

}

#endif // end of _3D_GAUSSIAN_H_
