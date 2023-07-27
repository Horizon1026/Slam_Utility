#include "visualizor.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "log_report.h"

namespace SLAM_UTILITY {

std::map<std::string, VisualizorWindow> Visualizor::windows_;
bool Visualizor::some_key_pressed_ = false;

Visualizor &Visualizor::GetInstance() {
    static Visualizor instance;
    return instance;
}

Visualizor::~Visualizor() {
    // Clear all windows and recovery resources.
    Visualizor::windows_.clear();
    glfwTerminate();
}

void Visualizor::ShowImageWithDetectedFeatures(const std::string &window_title,
                                               const GrayImage &image,
                                               const std::vector<Vec2> &pixel_uv,
                                               RgbPixel color) {
    // Create template rgb image.
    uint8_t *show_ref_image_buf = (uint8_t *)SlamMemory::Malloc(image.rows() * image.cols() * 3);
    RgbImage show_ref_image(show_ref_image_buf, image.rows(), image.cols(), true);
    Visualizor::ConvertUint8ToRgb(image.data(), show_ref_image.data(), image.rows() * image.cols());

    for (uint32_t i = 0; i < pixel_uv.size(); ++i) {
        Visualizor::DrawSolidCircle(show_ref_image, pixel_uv[i].x(), pixel_uv[i].y(), 3, color);
    }
    Visualizor::ShowImage(window_title, show_ref_image);
}

void Visualizor::ShowImageWithTrackedFeatures(const std::string &window_title,
                                              const GrayImage &cur_image,
                                              const std::vector<Vec2> &ref_pixel_uv,
                                              const std::vector<Vec2> &cur_pixel_uv,
                                              const std::vector<uint8_t> &track_status,
                                              uint8_t min_valid_track_status_value,
                                              RgbPixel tracked_color,
                                              RgbPixel untracked_color,
                                              RgbPixel flow_line_color) {
    RETURN_IF(cur_image.data() == nullptr);
    RETURN_IF(ref_pixel_uv.size() != cur_pixel_uv.size() || cur_pixel_uv.size() != track_status.size());

    // Create template rgb image.
    uint8_t *show_cur_image_buf = (uint8_t *)SlamMemory::Malloc(cur_image.rows() * cur_image.cols() * 3);
    RgbImage show_cur_image(show_cur_image_buf, cur_image.rows(), cur_image.cols(), true);
    Visualizor::ConvertUint8ToRgb(cur_image.data(), show_cur_image.data(), cur_image.rows() * cur_image.cols());

    for (uint32_t i = 0; i < ref_pixel_uv.size(); ++i) {
        if (track_status[i] > min_valid_track_status_value) {
            Visualizor::DrawSolidCircle(show_cur_image, cur_pixel_uv[i].x(), cur_pixel_uv[i].y(), 3, untracked_color);
            continue;
        }
        Visualizor::DrawSolidCircle(show_cur_image, cur_pixel_uv[i].x(), cur_pixel_uv[i].y(), 3, tracked_color);
        Visualizor::DrawBressenhanLine(show_cur_image, ref_pixel_uv[i].x(), ref_pixel_uv[i].y(),
            cur_pixel_uv[i].x(), cur_pixel_uv[i].y(), flow_line_color);
    }
    Visualizor::ShowImage(window_title, show_cur_image);
}

void Visualizor::ShowImageWithTrackedFeatures(const std::string &window_title,
                                              const GrayImage &ref_image,
                                              const GrayImage &cur_image,
                                              const std::vector<Vec2> &ref_pixel_uv,
                                              const std::vector<Vec2> &cur_pixel_uv,
                                              const std::vector<uint8_t> &track_status,
                                              uint8_t min_valid_track_status_value,
                                              RgbPixel tracked_color,
                                              RgbPixel untracked_color) {
    RETURN_IF(ref_image.data() == nullptr || cur_image.data() == nullptr);
    RETURN_IF(ref_image.rows() != cur_image.rows() || ref_image.cols() != cur_image.cols());
    RETURN_IF(ref_pixel_uv.size() != cur_pixel_uv.size() || cur_pixel_uv.size() != track_status.size());

    // Merge reference and current image into a gray image.
    uint8_t *merged_gray_buf = (uint8_t *)SlamMemory::Malloc(cur_image.rows() * cur_image.cols() * 2 * sizeof(uint8_t));
    GrayImage merged_image(merged_gray_buf, cur_image.rows(), cur_image.cols() * 2, true);
    for (int32_t v = 0; v < merged_image.rows(); ++v) {
        for (int32_t u = 0; u < merged_image.cols(); ++u) {
            if (u < cur_image.cols()) {
                merged_image.SetPixelValueNoCheck(v, u, ref_image.GetPixelValueNoCheck(v, u));
            } else {
                merged_image.SetPixelValueNoCheck(v, u, cur_image.GetPixelValueNoCheck(v, u - cur_image.cols()));
            }
        }
    }

    // Construct image to show.
    uint8_t *merged_rgb_buf = (uint8_t *)SlamMemory::Malloc(merged_image.rows() * merged_image.cols() * 3 * sizeof(uint8_t));
    RgbImage show_image(merged_rgb_buf, merged_image.rows(), merged_image.cols(), true);
    Visualizor::ConvertUint8ToRgb(merged_image.data(), show_image.data(), merged_image.rows() * merged_image.cols());

    // Draw untracked current points.
    for (uint32_t i = 0; i < cur_pixel_uv.size(); ++i) {
        if (track_status[i] > min_valid_track_status_value) {
            Visualizor::DrawSolidCircle(show_image, cur_pixel_uv[i].x() + cur_image.cols(), cur_pixel_uv[i].y(), 3, untracked_color);
        }
    }

    // Draw tracked current points and pairs.
    for (uint32_t i = 0; i < cur_pixel_uv.size(); ++i) {
        CONTINUE_IF(track_status[i] > min_valid_track_status_value);
        Visualizor::DrawSolidCircle(show_image, cur_pixel_uv[i].x() + cur_image.cols(), cur_pixel_uv[i].y(), 3, tracked_color);
        Visualizor::DrawBressenhanLine(show_image, ref_pixel_uv[i].x(), ref_pixel_uv[i].y(),
            cur_pixel_uv[i].x() + cur_image.cols(), cur_pixel_uv[i].y(),
            RgbPixel{.r = static_cast<uint8_t>(std::rand() % 256),
                     .g = static_cast<uint8_t>(std::rand() % 256),
                     .b = static_cast<uint8_t>(std::rand() % 256)});
    }

    // Draw reference points.
    for (uint32_t i = 0; i < ref_pixel_uv.size(); ++i) {
        Visualizor::DrawSolidCircle(show_image, ref_pixel_uv[i].x(), ref_pixel_uv[i].y(), 3, tracked_color);
    }

    Visualizor::ShowImage(window_title, show_image);
}

}
