#ifndef _SLAM_UTILITY_VISUALIZOR_H_
#define _SLAM_UTILITY_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

#include "GL/glut.h"

namespace SLAM_UTILITY {

class Visualizor {

public:
	Visualizor() = delete;
    Visualizor(int argc, char *argv[]);
    virtual ~Visualizor();

    bool ShowImage(const std::string &window_title, const Image &image);

    template <typename Scalar>
    bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                              Image &image,
                              Scalar max_value = 1e3,
                              int32_t scale = 4);

private:
    template <typename Scalar>
    uint8_t ConvertValueToUint8_t(Scalar value, Scalar max_value);

    static void RefreshBuffer();

private:
    static GLint image_cols_ ;
    static GLint image_rows_ ;
    static GLint image_pixel_length_ ;
    static GLubyte *image_data_;
};

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
            const uint8_t image_value = ConvertValueToUint8_t(matrix(row, col), max_value);
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
uint8_t Visualizor::ConvertValueToUint8_t(Scalar value, Scalar max_value) {
    value = std::fabs(value);
    if (value >= max_value) {
        return 0;
    }

    const Scalar step = max_value / 256.0;
    return 255 - static_cast<uint8_t>(value / step);
}

}

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
