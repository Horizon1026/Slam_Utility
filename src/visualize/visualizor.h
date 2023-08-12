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

    // Support for feature tracking result visualization.
    static void ShowImageWithDetectedFeatures(const std::string &window_title,
                                              const GrayImage &image,
                                              const std::vector<Vec2> &pixel_uv,
                                              RgbPixel color = RgbPixel{.r = 255, .g = 0, .b = 0});
    static void ShowImageWithTrackedFeatures(const std::string &window_title,
                                             const GrayImage &cur_image,
                                             const std::vector<Vec2> &ref_pixel_uv,
                                             const std::vector<Vec2> &cur_pixel_uv,
                                             const std::vector<uint8_t> &track_status,
                                             uint8_t min_valid_track_status_value = 2,
                                             RgbPixel tracked_color = RgbPixel{.r = 0, .g = 100, .b = 255},
                                             RgbPixel untracked_color = RgbPixel{.r = 255, .g = 0, .b = 0},
                                             RgbPixel flow_line_color = RgbPixel{.r = 0, .g = 255, .b = 0});
    static void ShowImageWithTrackedFeatures(const std::string &window_title,
                                             const GrayImage &ref_image,
                                             const GrayImage &cur_image,
                                             const std::vector<Vec2> &ref_pixel_uv,
                                             const std::vector<Vec2> &cur_pixel_uv,
                                             const std::vector<uint8_t> &track_status,
                                             uint8_t min_valid_track_status_value = 2,
                                             RgbPixel tracked_color = RgbPixel{.r = 0, .g = 255, .b = 255},
                                             RgbPixel untracked_color = RgbPixel{.r = 255, .g = 0, .b = 0});
    static void ShowImageWithTrackedFeaturesWithId(const std::string &window_title,
                                                   const GrayImage &ref_image,
                                                   const GrayImage &cur_image,
                                                   const std::vector<Vec2> &ref_pixel_uv,
                                                   const std::vector<Vec2> &cur_pixel_uv,
                                                   const std::vector<uint32_t> &ref_ids,
                                                   const std::vector<uint32_t> &cur_ids,
                                                   const std::vector<uint8_t> &tracked_status,
                                                   uint8_t min_valid_track_status_value = 2,
                                                   const std::vector<uint32_t> &ref_tracked_cnt = std::vector<uint32_t>(),
                                                   const std::vector<Vec2> &cur_optical_velocity = std::vector<Vec2>());
    static void ShowImageWithTrackedFeaturesWithId(const std::string &window_title,
                                                   const GrayImage &ref_image_left,
                                                   const GrayImage &ref_image_right,
                                                   const GrayImage &cur_image_left,
                                                   const GrayImage &cur_image_right,
                                                   const std::vector<Vec2> &ref_pixel_uv_left,
                                                   const std::vector<Vec2> &ref_pixel_uv_right,
                                                   const std::vector<Vec2> &cur_pixel_uv_left,
                                                   const std::vector<Vec2> &cur_pixel_uv_right,
                                                   const std::vector<uint32_t> &ref_ids_left,
                                                   const std::vector<uint32_t> &ref_ids_right,
                                                   const std::vector<uint32_t> &cur_ids_left,
                                                   const std::vector<uint32_t> &cur_ids_right,
                                                   const std::vector<uint32_t> &ref_tracked_cnt = std::vector<uint32_t>(),
                                                   const std::vector<Vec2> &cur_optical_velocity = std::vector<Vec2>());

    // Support for image show.
    template <typename T>
    static bool ShowImage(const std::string &window_title, const T &image, bool resizable = false);
    static void WaitKey(int32_t delay_ms);
    static void WindowList();

    // Support for image draw.
    template <typename ImageType, typename PixelType>
    static void DrawSolidRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawHollowRectangle(ImageType &image, int32_t x, int32_t y, int32_t width, int32_t height, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawBressenhanLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawNaiveLine(ImageType &image, int32_t x1, int32_t y1, int32_t x2, int32_t y2, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawSolidCircle(ImageType &image, int32_t center_x, int32_t center_y, int32_t radius, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawHollowCircle(ImageType &image, int32_t center_x, int32_t center_y, int32_t radius, const PixelType &color);
    template <typename ImageType, typename PixelType>
    static void DrawString(ImageType &image, const std::string &str, int32_t x, int32_t y, const PixelType &color, int32_t font_size = 12);

    // Support for image load and save.
    template <typename ImageType>
    static bool LoadImage(const std::string &image_file, ImageType &image);

    // Support for convertion.
    template <typename Scalar>
    static uint8_t ConvertValueToUint8(Scalar value, Scalar max_value);
    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     GrayImage &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);
    template <typename Scalar>
    static bool ConvertMatrixToImage(const TMat<Scalar> &matrix,
                                     RgbImage &image,
                                     Scalar max_value = 1e3,
                                     int32_t scale = 4);
    static void ConvertUint8ToRgb(const uint8_t *gray,
                                  uint8_t *rgb,
                                  int32_t gray_size);
    static void ConvertRgbToUint8(const uint8_t *rgb,
                                  uint8_t *gray,
                                  int32_t gray_size);
    static void ConvertUint8ToRgbAndUpsideDown(const uint8_t *gray,
                                               uint8_t *rgb,
                                               int32_t gray_rows,
                                               int32_t gray_cols);
    static void ConvertRgbToBgrAndUpsideDown(const uint8_t *rgb,
                                             uint8_t *converted_rgb,
                                             int32_t rgb_rows,
                                             int32_t rgb_cols);

    // Reference for member variables.
    static std::map<std::string, VisualizorWindow> &windows() { return windows_; }
    static bool &some_key_pressed() { return some_key_pressed_; }

private:
	Visualizor() = default;

    // Callback function for image show.
    static void ErrorCallback(int32_t error, const char *description);
    static void KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods);

    // Inner support for image show.
    static VisualizorWindow *GetWindowPointer(const std::string &title, int32_t width, int32_t height);
    template <typename T> static void PreprocessImage(const T &image, uint8_t *buff);
    template <typename T> static void CreateTextureByImage(const T &image, GLuint &texture_id);
    static void ShowTextureInCurrentWindow(GLuint texture_id);

    // Inner support for image draw.
    template <typename ImageType, typename PixelType>
    static void DrawCharacter(ImageType &image, char character, int32_t x, int32_t y, const PixelType &color, int32_t font_size = 12);

    // Inner support for feature tracking result visualization.
    static void DrawFeaturesWithIdByTrackedNumbers(const std::vector<Vec2> &pixel_uv,
                                                   const std::vector<uint32_t> &ids,
                                                   const Pixel &pixel_offset,
                                                   const std::vector<uint8_t> &status,
                                                   const uint8_t min_valid_status_value,
                                                   const std::vector<uint32_t> &tracked_cnt,
                                                   RgbImage &image);
    static void DrawFeaturesWithIdByOpticalVelocity(const std::vector<Vec2> &pixel_uv,
                                                    const std::vector<uint32_t> &ids,
                                                    const Pixel &pixel_offset,
                                                    const std::vector<uint8_t> &status,
                                                    const uint8_t min_valid_status_value,
                                                    const std::vector<Vec2> &optical_velocity,
                                                    RgbImage &image);

private:
    // Member varibles for image show.
    static std::map<std::string, VisualizorWindow> windows_;
    static bool some_key_pressed_;
};

}

#endif // end of _SLAM_UTILITY_VISUALIZOR_H_
