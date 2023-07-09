#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

namespace SLAM_UTILITY {

void Visualizor::ConvertUint8ToRGB(const uint8_t *gray, uint8_t *rgba, int32_t gray_size) {
    for (int32_t i = 0; i < gray_size; ++i) {
        const int32_t idx = i * 3;
        std::fill_n(rgba + idx, 3, gray[i]);
    }
}

void Visualizor::ConvertUint8ToRgbAndUpsideDown(const uint8_t *gray,
                                                uint8_t *rgba,
                                                int32_t gray_rows,
                                                int32_t gray_cols) {
    const int32_t gray_cols_3 = 3 * gray_cols;

    for (int32_t row = 0; row < gray_rows; ++row) {
        for (int32_t col = 0; col < gray_cols; ++col) {
            const int32_t offset = (gray_rows - row - 1) * gray_cols_3 + 3 * col;
            std::fill_n(rgba + offset, 3, gray[col + row * gray_cols]);
        }
    }
}

}
