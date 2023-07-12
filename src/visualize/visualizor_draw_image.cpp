#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"
#include "slam_operations.h"

namespace SLAM_UTILITY {

template void Visualizor::DrawSolidRectangle<GrayImage, uint8_t>(GrayImage &image, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t &color);
template void Visualizor::DrawSolidRectangle<RgbImage, RgbPixel>(RgbImage &image, int32_t x, int32_t y, int32_t width, int32_t height, const RgbPixel &color);
template <typename ImageType, typename PixelType>
void Visualizor::DrawSolidRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color) {
    if (image.data() == nullptr) {
        return;
    }
    for (int32_t u = x; u < x + width; ++u) {
        CONTINUE_IF(u > image.cols() - 1);
        for (int32_t v = y; v < y + height; ++v) {
        BREAK_IF(v > image.rows() - 1);
            image.SetPixelValue(v, u, color);
        }
    }
}

template void Visualizor::DrawBressenhanLine<GrayImage, uint8_t>(GrayImage &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint8_t &color);
template void Visualizor::DrawBressenhanLine<RgbImage, RgbPixel>(RgbImage &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const RgbPixel &color);
template <typename ImageType, typename PixelType>
void Visualizor::DrawBressenhanLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color) {
    if (image.data() == nullptr) {
        return;
    }

    const int32_t dx = std::abs(x2 - x1);
    const int32_t dy = std::abs(y2 - y1);
    bool is_larger_than_45_deg = false;
    if (dx < dy) {
        is_larger_than_45_deg = true;
        std::swap(x1, y1);
        std::swap(x2, y2);
        std::swap(dx, dy);
    }

    // Precompute some temp variables.
    const int32_t ix = (x2 - x1) > 0 ? 1 : -1;
    const int32_t iy = (y2 - y1) > 0 ? 1 : -1;
    int32_t cx = x1;
    int32_t cy = y1;
    const int32_t dy_2 = dy << 1;
    const int32_t dy_dx_2 = (dy - dx) << 1;
    int32_t dy_2_dx = dy << 1 - dx;

    // Draw line.
    while (cx != x2) {
        if (dy_2_dx < 0) {
            dy_2_dx += dy_2;
        } else {
            cy += iy;
            dy_2_dx += dy_dx_2;
        }

        if (is_larger_than_45_deg) {
            image.SetPixelValue(cy, cx, color);
        } else {
            image.SetPixelValue(cx, cy, color);
        }
        cx += ix;
    }
}

}
