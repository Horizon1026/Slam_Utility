#include "datatype_image.h"
#include "slam_operations.h"
#include "log_report.h"

#include "2d_gaussian.h"
#include "3d_gaussian.h"

#include "visualizor.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

constexpr int32_t image_rows = 1000;
constexpr int32_t image_cols = 1000;
constexpr float fx = 800.0f;
constexpr float fy = 800.0f;
constexpr float cx = 400.0f;
constexpr float cy = 400.0f;

void TestShowOne3DGaussian() {
    ReportInfo(">> Test only one 3d gaussian rendering.");
    // Camera view.
    const Quat q_wc = Quat::Identity();
    const Vec3 p_wc = Vec3(0, 0, -10.0f);

    // Allocate for show image.
    uint8_t *buf = (uint8_t *)malloc(image_rows * image_cols * 3 * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);

    // Create 3d gaussian.
    Gaussian3D gaussian_3d;
    gaussian_3d.color() = RgbColor::kGreen;
    gaussian_3d.p_w() = Vec3::Zero();
    gaussian_3d.mid_opacity() = 1.0f;
    gaussian_3d.sigma_s() = Vec3(1, 2, 1.5f);
    gaussian_3d.sigma_q() = Quat::Identity();

    // Project 3d gaussian to 2d.
    Gaussian2D gaussian_2d;
    gaussian_3d.ProjectTo2D(p_wc, q_wc, gaussian_2d);
    const Mat2 inv_sigma_2d = gaussian_2d.sigma().inverse();

    // Iterate each pixel of image to compute color.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - cx) / fx, (row - cy) / fy);
            const float opacity = gaussian_2d.GetOpacityAt(uv, inv_sigma_2d);
            const RgbPixel pixel_color = RgbPixel{
                .r = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().r) * opacity),
                .g = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().g) * opacity),
                .b = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().b) * opacity),
            };
            show_image.SetPixelValueNoCheck(row, col, pixel_color);
        }
    }

    Visualizor::ShowImage("show image", show_image);
    Visualizor::WaitKey(0);
}

void TestShowSeveral3DGaussian() {
    ReportInfo(">> Test several 3d gaussian rendering.");
    // Camera view.
    const Quat q_wc = Quat::Identity();
    const Vec3 p_wc = Vec3(0, 0, -10.0f);

    // Allocate for show image.
    uint8_t *buf = (uint8_t *)malloc(image_rows * image_cols * 3 * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);

    // Create 3d gaussian.
    std::vector<Gaussian3D> all_gaussian_3d;
    for (int32_t i = 0; i < 5; ++i) {
        Gaussian3D gaussian_3d;
        gaussian_3d.color() = RgbColor::kGreen;
        gaussian_3d.p_w() = Vec3(i, i * 0.6, 2.5f + i);
        gaussian_3d.mid_opacity() = 1.0f;
        gaussian_3d.sigma_s() = Vec3(i, i * 0.6, -2.5f + i);
        gaussian_3d.sigma_q() = Quat::Identity();
        all_gaussian_3d.emplace_back(gaussian_3d);
    }

    // Project 3d gaussian to 2d.
    std::vector<Gaussian2D> all_gaussian_2d;
    std::vector<float> all_gaussian_depth;
    for (uint32_t i = 0; i < all_gaussian_3d.size(); ++i) {
        Gaussian2D gaussian_2d;
        all_gaussian_3d[i].ProjectTo2D(p_wc, q_wc, gaussian_2d);
        all_gaussian_2d.emplace_back(gaussian_2d);
        all_gaussian_depth.emplace_back(gaussian_2d.depth());
    }

    // Sort gaussians by depth.
    std::vector<int32_t> indices;
    SlamOperation::ArgSort(all_gaussian_depth, indices);
    for (const auto &index : indices) {
        ReportInfo(all_gaussian_2d[index].depth());
    }

    // Iterate each pixel of image to compute color.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - cx) / fx, (row - cy) / fy);

            float multi_opacity = 0.0f;
            Vec3 float_color = Vec3::Zero();
            for (const auto &index : indices) {
                const auto &gaussian_2d = all_gaussian_2d[index];
                const float opacity = gaussian_2d.GetOpacityAt(uv, gaussian_2d.inv_sigma());
                if (index == indices.front()) {
                    float_color.x() = gaussian_2d.color().r * opacity;
                    float_color.y() = gaussian_2d.color().g * opacity;
                    float_color.z() = gaussian_2d.color().b * opacity;
                    multi_opacity = 1.0f - opacity;
                } else {
                    float_color.x() = gaussian_2d.color().r * opacity * multi_opacity;
                    float_color.y() = gaussian_2d.color().g * opacity * multi_opacity;
                    float_color.z() = gaussian_2d.color().b * opacity * multi_opacity;
                    multi_opacity *= 1.0f - opacity;
                }
            }

            const RgbPixel pixel_color = RgbPixel{
                .r = static_cast<uint8_t>(float_color.x()),
                .g = static_cast<uint8_t>(float_color.y()),
                .b = static_cast<uint8_t>(float_color.z()),
            };
            show_image.SetPixelValueNoCheck(row, col, pixel_color);
        }
    }

    Visualizor::ShowImage("show image", show_image);
    Visualizor::WaitKey(0);
}

int main(int argc, char *argv[]) {
    ReportInfo(">> Test 3d gaussian.");
    TestShowOne3DGaussian();
    TestShowSeveral3DGaussian();
    return 0;
}