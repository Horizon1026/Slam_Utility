#ifndef _BASIC_TYPE_RGBCOLOR_H_
#define _BASIC_TYPE_RGBCOLOR_H_

#include "basic_type.h"

namespace SLAM_UTILITY {

// Rgb image element definition.
struct RgbPixel {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

/* Class RgbColor Declaration. */
class RgbColor {

public:
    // Typical color definition.
    static RgbPixel kPink;
    static RgbPixel kHotPink;
    static RgbPixel kViolet;
    static RgbPixel kMagenta;
    static RgbPixel kPurple;
    static RgbPixel kLavender;
    static RgbPixel kBlue;
    static RgbPixel kRoyalBlue;
    static RgbPixel kAliceBlue;
    static RgbPixel kLightSkyBlue;
    static RgbPixel kDeepSkyBlue;
    static RgbPixel kCyan;
    static RgbPixel kWhite;
    static RgbPixel kFloralWhite;
    static RgbPixel kBlack;
    static RgbPixel kSlateGray;
    static RgbPixel kGreen;
    static RgbPixel kLightGreen;
    static RgbPixel kLaunGreen;
    static RgbPixel kYellow;
    static RgbPixel kKhaki;
    static RgbPixel kGold;
    static RgbPixel kOrange;
    static RgbPixel kDrakOrange;
    static RgbPixel kChocolate;
    static RgbPixel kOrangeRed;
    static RgbPixel kRed;
    static RgbPixel kDarkRed;
    static RgbPixel kBrown;

public:
    RgbColor() = default;
    virtual ~RgbColor() = default;

    // Return random rgb color.
    static RgbPixel Random() {
        return RgbPixel{
            .r = static_cast<uint8_t>(std::rand() % 255),
            .g = static_cast<uint8_t>(std::rand() % 255),
            .b = static_cast<uint8_t>(std::rand() % 255),
        };
    }

    // Weight [0, 1] means [cold, warm] color.
    static RgbPixel ColdWarm(const float weight) {
        const uint8_t warm = static_cast<uint8_t>(std::min(0.0f, std::max(255.0f, weight * 255.0f)));
        return RgbPixel{
            .r = warm,
            .g = 0,
            .b = static_cast<uint8_t>(255 - warm),
        };
    }

    static uint8_t ConvertFromFloat(const float value) {
        const float temp = std::max(std::min(value * 255.0f, 255.0f), 0.0f);
        return static_cast<uint8_t>(temp);
    }
    static float ConvertFromUint8(const uint8_t value) {
        return static_cast<float>(value) / 255.0f;
    }

};

}

#endif // end of _BASIC_TYPE_RGBCOLOR_H_
