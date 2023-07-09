#ifndef _SLAM_UTILITY_VISUALIZOR_H_
#define _SLAM_UTILITY_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

// Dependence.
#include "glad.h"
#include "GLFW/glfw3.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "time.h"

namespace SLAM_UTILITY {

struct VisualizorWindow {
    GLFWwindow *glfw_window = nullptr;
    GLuint texture_id = 0;
};

// Class Visualizor Declaration.
class Visualizor {

public:
    virtual ~Visualizor();

    static Visualizor &GetInstance();

    static bool ShowImage(const std::string &window_title, const Image &image, bool resizable = false);
    static void WaitKey(int32_t delay_ms);

    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     Image &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);

    static void ConvertUint8ToRGB(const uint8_t *gray,
                                  uint8_t *rgba,
                                  int32_t gray_size);

    static void ConvertUint8ToRgbAndUpsideDown(const uint8_t *gray,
                                               uint8_t *rgba,
                                               int32_t gray_rows,
                                               int32_t gray_cols);

    static void WindowList();

    // Reference for member variables.
    static std::map<std::string, VisualizorWindow> &windows() { return windows_; }

private:
	Visualizor() = default;

    template <typename Scalar>
    static uint8_t ConvertValueToUint8(Scalar value, Scalar max_value);

    // Callback function.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);

    // Inner support.
    static VisualizorWindow *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    static void CreateTextureByImage(const Image &image, GLuint &texture_id);
    static void DrawTextureInCurrentWindow(GLuint texture_id);

private:
    static std::map<std::string, VisualizorWindow> windows_;
};

// Class Visualizor Definition.
// template <> uint8_t Visualizor::ConvertValueToUint8<float>(float value, float max_value);
// template <> uint8_t Visualizor::ConvertValueToUint8<double>(double value, double max_value);
template <typename Scalar>
uint8_t Visualizor::ConvertValueToUint8(Scalar value, Scalar max_value) {
    value = std::fabs(value);
    if (value >= max_value) {
        return 0;
    }

    const Scalar step = max_value / 256.0;
    return 255 - static_cast<uint8_t>(value / step);
}

// template <> bool Visualizor::ConvertMatrixToImage<float>(const TMat<float> &matrix, Image &image, float max_value, int32_t scale);
// template <> bool Visualizor::ConvertMatrixToImage<double>(const TMat<double> &matrix, Image &image, double max_value, int32_t scale);
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

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
