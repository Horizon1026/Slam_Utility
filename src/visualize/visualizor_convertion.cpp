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

template uint8_t Visualizor::ConvertValueToUint8<float>(float value, float max_value);
template uint8_t Visualizor::ConvertValueToUint8<double>(double value, double max_value);
template <typename Scalar>
uint8_t Visualizor::ConvertValueToUint8(Scalar value, Scalar max_value) {
    value = std::fabs(value);
    if (value >= max_value) {
        return 0;
    }

    const Scalar step = max_value / 256.0;
    return 255 - static_cast<uint8_t>(value / step);
}

template bool Visualizor::ConvertMatrixToImage<float>(const TMat<float> &matrix, Image &image, float max_value, int32_t scale);
template bool Visualizor::ConvertMatrixToImage<double>(const TMat<double> &matrix, Image &image, double max_value, int32_t scale);
template <typename Scalar>
bool Visualizor::ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                      Image &image,
                                      Scalar max_value,
                                      int32_t scale) {
    if (image.data() == nullptr) {
        ReportError("[Visualizor] Image buffer is empty.");
        return false;
    }
    if (scale < 0) {
        ReportError("[Visualizor] Scale must larger than 0.");
        return false;
    }
    if (image.rows() != matrix.rows() * scale || image.cols() != matrix.cols() * scale) {
        ReportError("[Visualizor] Image buffer size does not match matrix size.");
        return false;
    }

    // Convert matrix to image.
    for (int32_t row = 0; row < matrix.rows(); ++row) {
        for (int32_t col = 0; col < matrix.cols(); ++col) {
            // Compute image value in the first line.
            const uint8_t image_value = ConvertValueToUint8(matrix(row, col), max_value);
            const int32_t image_row = row * scale;
            const int32_t image_col = col * scale;

            // Fill the block in image.
            for (int32_t i = 0; i < scale; ++i) {
                std::fill_n(image.data() + (image_row + i) * image.cols() + image_col, scale, image_value);
            }
        }
    }

    return true;
}

}
