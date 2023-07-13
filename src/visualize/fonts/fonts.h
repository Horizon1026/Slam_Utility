#ifndef _VISUALIZE_FONTS_H_
#define _VISUALIZE_FONTS_H_

#include "datatype_basic.h"
#include "array"

namespace SLAM_UTILITY {

// Class VisualizeFonts Declaration.
class VisualizeFonts {

public:
    VisualizeFonts() = default;
    virtual ~VisualizeFonts() = default;

    static const std::array<std::array<uint8_t, 12>, 95> &ascii_1206() { return ascii_1206_; }
    static const std::array<std::array<uint8_t, 16>, 95> &ascii_1608() { return ascii_1608_; }
    static const std::array<std::array<uint8_t, 36>, 95> &ascii_2412() { return ascii_2412_; }

private:
    static std::array<std::array<uint8_t, 12>, 95> ascii_1206_;
    static std::array<std::array<uint8_t, 16>, 95> ascii_1608_;
    static std::array<std::array<uint8_t, 36>, 95> ascii_2412_;

};

}

#endif // end of _VISUALIZE_FONTS_H_
