#ifndef _SPHERICAL_HARMONIC_H_
#define _SPHERICAL_HARMONIC_H_

#include "datatype_basic.h"
#include "datatype_rgbcolor.h"

namespace SLAM_UTILITY {

/* Class SphericalHarmonicFunc Declaration. */
template <uint32_t Order>
class SphericalHarmonicFunc {

public:
    SphericalHarmonicFunc() = default;
    explicit SphericalHarmonicFunc(const Vec16 &coeff) : coeff_(coeff) {}
    virtual ~SphericalHarmonicFunc() = default;

    float GetColorInFloat(const Vec3 &p_wf, const Vec3 &p_wc);
    float GetColorInFloat(const Vec3 &direction);

    // Reference for member variables.
    Vec16 &coeff() { return coeff_; }
    // Const reference for member variables.
    const Vec16 &coeff() const { return coeff_; }

private:
    Vec16 coeff_ = Vec16::Zero();
};

/* Class SphericalHarmonicFunc Definition. */
template <uint32_t Order>
float SphericalHarmonicFunc<Order>::GetColorInFloat(const Vec3 &p_wf, const Vec3 &p_wc) {
    const Vec3 dir = p_wf - p_wc;
    return GetColorInFloat(dir.normalized());
}

}

#endif // end of _SPHERICAL_HARMONIC_H_
