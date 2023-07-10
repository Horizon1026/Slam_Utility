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

    // Support for image show.
    static bool ShowImage(const std::string &window_title, const GrayImage &image, bool resizable = false);
    static void WaitKey(int32_t delay_ms);
    static void WindowList();

    // Suppotr for convertion.
    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     GrayImage &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);
    static void ConvertUint8ToRGB(const uint8_t *gray,
                                  uint8_t *rgba,
                                  int32_t gray_size);
    static void ConvertUint8ToRgbAndUpsideDown(const uint8_t *gray,
                                               uint8_t *rgba,
                                               int32_t gray_rows,
                                               int32_t gray_cols);

    // Reference for member variables.
    static std::map<std::string, VisualizorWindow> &windows() { return windows_; }
    static bool &some_key_pressed() { return some_key_pressed_; }

private:
	Visualizor() = default;

    template <typename Scalar>
    static uint8_t ConvertValueToUint8(Scalar value, Scalar max_value);

    // Callback function.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);

    // Inner support.
    static VisualizorWindow *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    static void CreateTextureByImage(const GrayImage &image, GLuint &texture_id);
    static void DrawTextureInCurrentWindow(GLuint texture_id);

private:
    static std::map<std::string, VisualizorWindow> windows_;
    static bool some_key_pressed_;
};

}

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
