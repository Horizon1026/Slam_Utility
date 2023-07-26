#include "visualizor.h"
#include "slam_memory.h"
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
    if (ref_pixel_uv.size() != cur_pixel_uv.size() || cur_pixel_uv.size() != track_status.size()) {
        return;
    }

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

}
