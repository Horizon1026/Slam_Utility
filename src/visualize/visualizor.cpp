#include "visualizor.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "log_report.h"

namespace SLAM_UTILITY {

namespace {
    constexpr float kStringLocationColOffset = 3.0f;
    constexpr float kStringLocationRowOffset = -12.0f;
}

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

void Visualizor::ShowImageWithTrackedFeaturesWithId(const std::string &window_title,
                                                    const GrayImage &ref_image,
                                                    const GrayImage &cur_image,
                                                    const std::vector<Vec2> &ref_pixel_uv,
                                                    const std::vector<Vec2> &cur_pixel_uv,
                                                    const std::vector<uint32_t> &ref_ids,
                                                    const std::vector<uint32_t> &cur_ids,
                                                    const std::vector<uint8_t> &tracked_status,
                                                    uint8_t min_valid_track_status_value,
                                                    const std::vector<uint32_t> &ref_tracked_cnt,
                                                    const std::vector<Vec2> &cur_optical_velocity) {
    RETURN_IF(ref_image.data() == nullptr || cur_image.data() == nullptr);
    RETURN_IF(ref_image.rows() != cur_image.rows() || ref_image.cols() != cur_image.cols());
    RETURN_IF(ref_pixel_uv.size() != ref_ids.size() || cur_pixel_uv.size() != cur_ids.size());

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
    Visualizor::DrawString(show_image, "[ref image]", 0, 0, RgbPixel{.r = 200, .g = 150, .b = 255}, 16);
    Visualizor::DrawString(show_image, "[cur image]", ref_image.cols(), 0, RgbPixel{.r = 200, .g = 150, .b = 255}, 16);

    // [left] Draw points in reference image.
    std::vector<uint8_t> empty_tracked_status;
    Visualizor::DrawFeaturesWithIdByTrackedNumbers(ref_pixel_uv, ref_ids, Pixel(0, 0), empty_tracked_status,
        min_valid_track_status_value, ref_tracked_cnt, show_image);

    // [right] Draw points in current image.
    Visualizor::DrawFeaturesWithIdByOpticalVelocity(cur_pixel_uv, cur_ids, Pixel(cur_image.cols(), 0),
        tracked_status, min_valid_track_status_value, cur_optical_velocity, show_image);

    Visualizor::ShowImage(window_title, show_image);
}

void Visualizor::ShowImageWithTrackedFeaturesWithId(const std::string &window_title,
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
                                                    const std::vector<uint32_t> &ref_tracked_cnt,
                                                    const std::vector<Vec2> &cur_optical_velocity) {
    RETURN_IF(ref_image_left.rows() != cur_image_left.rows() || ref_image_left.cols() != cur_image_left.cols());
    RETURN_IF(ref_image_right.rows() != cur_image_right.rows() || ref_image_right.cols() != cur_image_right.cols());
    RETURN_IF(ref_pixel_uv_left.size() != ref_ids_left.size());
    RETURN_IF(cur_pixel_uv_left.size() != cur_ids_left.size());

    Pixel ref_left_offset = Pixel(0, 0);
    Pixel ref_right_offset = Pixel(ref_image_left.cols(), 0);
    Pixel cur_left_offset = Pixel(0, ref_image_left.rows());
    Pixel cur_right_offset = Pixel(ref_image_left.cols(), ref_image_left.rows());

    // Merge reference and current image into a gray image.
    uint8_t *merged_gray_buf = (uint8_t *)SlamMemory::Malloc(ref_image_left.rows() * ref_image_left.cols() * 4 * sizeof(uint8_t));
    GrayImage merged_image(merged_gray_buf, ref_image_left.rows() * 2, ref_image_left.cols() * 2, true);
    for (int32_t v = 0; v < merged_image.rows(); ++v) {
        for (int32_t u = 0; u < merged_image.cols(); ++u) {
            if (v < ref_image_left.rows() && u < ref_image_left.cols()) {
                merged_image.SetPixelValueNoCheck(v, u, ref_image_left.GetPixelValueNoCheck(v, u));
            } else if (v < ref_image_left.rows() * 2 && u < ref_image_left.cols()) {
                merged_image.SetPixelValueNoCheck(v, u, cur_image_left.GetPixelValueNoCheck(v - ref_image_left.rows(), u));
            } else if (v < ref_image_left.rows() && u < ref_image_left.cols() * 2) {
                merged_image.SetPixelValueNoCheck(v, u, ref_image_right.GetPixelValueNoCheck(v, u - ref_image_left.cols()));
            } else {
                merged_image.SetPixelValueNoCheck(v, u, cur_image_right.GetPixelValueNoCheck(v - ref_image_left.rows(), u - ref_image_left.cols()));
            }
        }
    }

    // Construct image to show.
    uint8_t *merged_rgb_buf = (uint8_t *)SlamMemory::Malloc(merged_image.rows() * merged_image.cols() * 3 * sizeof(uint8_t));
    RgbImage show_image(merged_rgb_buf, merged_image.rows(), merged_image.cols(), true);
    Visualizor::ConvertUint8ToRgb(merged_image.data(), show_image.data(), merged_image.rows() * merged_image.cols());
    Visualizor::DrawString(show_image, "[ref left image]", ref_left_offset.x(), ref_left_offset.y(), RgbPixel{.r = 200, .g = 150, .b = 255}, 16);
    Visualizor::DrawString(show_image, "[ref right image]", ref_right_offset.x(), ref_right_offset.y(), RgbPixel{.r = 200, .g = 150, .b = 255}, 16);
    Visualizor::DrawString(show_image, "[cur left image]", cur_left_offset.x(), cur_left_offset.y(), RgbPixel{.r = 200, .g = 150, .b = 255}, 16);
    Visualizor::DrawString(show_image, "[cur right image]", cur_right_offset.x(), cur_right_offset.y(), RgbPixel{.r = 200, .g = 150, .b = 255}, 16);

    std::vector<uint8_t> status;

    // [top left] Draw points in reference left image.
    Visualizor::DrawFeaturesWithIdByTrackedNumbers(ref_pixel_uv_left, ref_ids_left, ref_left_offset,
        status, 2, ref_tracked_cnt, show_image);

    // [top right] Draw points in reference right image.
    Visualizor::DrawFeaturesWithIdByTrackedNumbers(ref_pixel_uv_right, ref_ids_right, ref_right_offset,
        status, 2, ref_tracked_cnt, show_image);

    // [bottom left] Draw points in current left image.
    Visualizor::DrawFeaturesWithIdByOpticalVelocity(cur_pixel_uv_left, cur_ids_left, cur_left_offset,
        status, 2, cur_optical_velocity, show_image);

    // [bottom right] Draw points in current right image.
    Visualizor::DrawFeaturesWithIdByOpticalVelocity(cur_pixel_uv_right, cur_ids_right, cur_right_offset,
        status, 2, cur_optical_velocity, show_image);

    Visualizor::ShowImage(window_title, show_image);
}

void Visualizor::DrawFeaturesWithIdByTrackedNumbers(const std::vector<Vec2> &pixel_uv,
                                                    const std::vector<uint32_t> &ids,
                                                    const Pixel &pixel_offset,
                                                    const std::vector<uint8_t> &status,
                                                    const uint8_t min_valid_status_value,
                                                    const std::vector<uint32_t> &tracked_cnt,
                                                    RgbImage &image) {
    if (pixel_uv.size() == tracked_cnt.size()) {
        // If tracked cnt of reference features is valid, draw features with different color.
        for (uint32_t i = 0; i < pixel_uv.size(); ++i) {
            CONTINUE_IF(!status.empty() && status[i] > min_valid_status_value);

            const int32_t color = std::max(static_cast<int32_t>(255 - tracked_cnt[i] * 80), static_cast<int32_t>(0));
            const RgbPixel pixel_color = RgbPixel{.r = 255, .g = static_cast<uint8_t>(255 - color), .b = 0};
            Visualizor::DrawSolidCircle(image, pixel_uv[i].x() + pixel_offset.x(), pixel_uv[i].y() + pixel_offset.y(), 3, pixel_color);
            Visualizor::DrawString(image, std::to_string(ids[i]), pixel_uv[i].x() + kStringLocationColOffset + pixel_offset.x(),
                pixel_uv[i].y() + kStringLocationRowOffset + pixel_offset.y(), pixel_color);
        }
    } else {
        // Draw features with fixed color.
        const RgbPixel pixel_color = RgbPixel{.r = 255, .g = 255, .b = 0};
        for (uint32_t i = 0; i < pixel_uv.size(); ++i) {
            CONTINUE_IF(!status.empty() && status[i] > min_valid_status_value);

            Visualizor::DrawSolidCircle(image, pixel_uv[i].x() + pixel_offset.x(), pixel_uv[i].y() + pixel_offset.y(), 3, pixel_color);
            Visualizor::DrawString(image, std::to_string(ids[i]), pixel_uv[i].x() + kStringLocationColOffset + pixel_offset.x(),
                pixel_uv[i].y() + kStringLocationRowOffset + pixel_offset.y(), pixel_color);
        }
    }
}

void Visualizor::DrawFeaturesWithIdByOpticalVelocity(const std::vector<Vec2> &pixel_uv,
                                                     const std::vector<uint32_t> &ids,
                                                     const Pixel &pixel_offset,
                                                     const std::vector<uint8_t> &status,
                                                     const uint8_t min_valid_status_value,
                                                     const std::vector<Vec2> &optical_velocity,
                                                     RgbImage &image) {
    const RgbPixel pixel_color = RgbPixel{.r = 0, .g = 255, .b = 255};

    if (pixel_uv.size() == optical_velocity.size()) {
        // If optical flow velocity of current features is valid, draw the optical flow trajectory.
        const RgbPixel line_color = RgbPixel{.r = 255, .g = 10, .b = 0};
        for (uint32_t i = 0; i < pixel_uv.size(); ++i) {
            CONTINUE_IF(!status.empty() && status[i] > min_valid_status_value);

            Visualizor::DrawSolidCircle(image, pixel_uv[i].x() + pixel_offset.x(), pixel_uv[i].y() + pixel_offset.y(), 3, pixel_color);
            Visualizor::DrawString(image, std::to_string(ids[i]), pixel_uv[i].x() + kStringLocationColOffset + pixel_offset.x(),
                pixel_uv[i].y() + kStringLocationRowOffset + pixel_offset.y(), pixel_color);
            Visualizor::DrawBressenhanLine(image, pixel_uv[i].x() + pixel_offset.x(), pixel_uv[i].y() + pixel_offset.y(),
                pixel_uv[i].x() + pixel_offset.x() - optical_velocity[i].x(),
                pixel_uv[i].y() + pixel_offset.y() - optical_velocity[i].y(), line_color);
        }
    } else {
        // Only draw features.
        for (uint32_t i = 0; i < pixel_uv.size(); ++i) {
            CONTINUE_IF(!status.empty() && status[i] > min_valid_status_value);

            Visualizor::DrawSolidCircle(image, pixel_uv[i].x() + pixel_offset.x(), pixel_uv[i].y() + pixel_offset.y(), 3, pixel_color);
            Visualizor::DrawString(image, std::to_string(ids[i]), pixel_uv[i].x() + kStringLocationColOffset + pixel_offset.x(),
                pixel_uv[i].y() + kStringLocationRowOffset + pixel_offset.y(), pixel_color);
        }
    }
}

}
