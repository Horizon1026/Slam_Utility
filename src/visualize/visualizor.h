#ifndef _SLAM_UTILITY_VISUALIZOR_H_
#define _SLAM_UTILITY_VISUALIZOR_H_

#include "datatype_basic.h"
#include "datatype_image.h"
#include "log_report.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "GLFW/glfw3.h"

namespace SLAM_UTILITY {

struct Texture {
    ImTextureID id = nullptr;
    uint8_t *buf = nullptr;
    int32_t rows = 0;
    int32_t cols = 0;
};

class Visualizor {

public:
    static Visualizor &GetInstance();
    virtual ~Visualizor();

    bool ShowImage(const std::string &window_title, const Image &image);

    template <typename Scalar>
    bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                              Image &image,
                              Scalar max_value = 1e3,
                              int32_t scale = 4);

    void RenderMainWindow(GLFWwindow *window);

    // Reference for member variables.
    GLFWwindow *main_window() { return main_window_; }

private:
	Visualizor();

    template <typename Scalar>
    uint8_t ConvertValueToUint8_t(Scalar value, Scalar max_value);

    static void GlfwErrorCallback(int error, const char *description);

    GLFWwindow *CreateNewWindow(int width, int height, const char *title);

    void InitializeImgui(GLFWwindow *window, const char *glsl_version = "#version 130");

    void RefreshMainWindow(GLFWwindow *window);

    void ProcessKeyboardMessage(GLFWwindow *window);

private:
    GLFWwindow *main_window_ = nullptr;

    // Store all image object with 'image name', 'texture id' and 'texture buffer'.
    std::unordered_map<std::string, Texture> image_objects_;

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
