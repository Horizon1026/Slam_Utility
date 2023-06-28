#ifndef _SLAM_UTILITY_VISUALIZOR_H_
#define _SLAM_UTILITY_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

namespace SLAM_UTILITY {

class Visualizor {

public:
	Visualizor() = default;
    virtual ~Visualizor() = default;

    template <typename Scalar>
    bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                              Image &image,
                              Scalar min_value = -1e3,
                              Scalar max_value = 1e3,
                              int32_t scale = 4);

    template <typename Scalar>
    uint8_t ConvertValueToUint8_t(Scalar value, Scalar min_value, Scalar max_value);

};

template <typename Scalar>
bool Visualizor::ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                      Image &image,
                                      Scalar min_value,
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
            const uint8_t image_value = ConvertValueToUint8_t(matrix(row, col), min_value, max_value);
            int32_t image_row = row * scale;
            int32_t image_col = col * scale;

            // Fill the block in image.
            for (int32_t i = 0; i < scale; ++i) {
                std::fill_n(image.data() + (image_row + i) * image.cols() + image_col, scale, image_value);
            }
        }
    }

    return true;
}

template <typename Scalar>
uint8_t Visualizor::ConvertValueToUint8_t(Scalar value, Scalar min_value, Scalar max_value) {
    if (value <= min_value) {
        return 0;
    } else if (value >= max_value) {
        return 255;
    }

    const Scalar step = (min_value - max_value) / 256.0;
    const Scalar ratio = (value - min_value) / step;
    return static_cast<uint8_t>(ratio);
}

}

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
