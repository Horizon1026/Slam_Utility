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

enum class VisualizorWindowState : int8_t {
    kFixed = 0,
    kResizable = 1,
};

struct VisualizorWindow {
    GLFWwindow *glfw_window = nullptr;
    VisualizorWindowState state = VisualizorWindowState::kFixed;
    GLuint texture_id = 0;
};

class Visualizor {

public:
    virtual ~Visualizor() = default;

    static Visualizor &GetInstance();

    static bool ShowImage(const std::string &window_title, const Image &image, bool resizable = false);
    static void WaitKey(int32_t delay_s);

    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     Image &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);

    static void ConvertUint8ToRGB(const uint8_t *gray,
                                   uint8_t *rgba,
                                   int32_t gray_size);

private:
	Visualizor() = default;

    template <typename Scalar>
    static uint8_t ConvertValueToUint8(Scalar value, Scalar max_value);

    // Callback function.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);

    // Inner support.
    static VisualizorWindow *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    static GLuint CreateTextureByImage(const Image &image);
    static void DrawTextureInCurrentWindow(GLuint texture_id);

private:
    static std::map<std::string, VisualizorWindow> windows_;
};

}

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
