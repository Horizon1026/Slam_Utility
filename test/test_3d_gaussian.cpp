#include "log_report.h"
#include "3d_gaussian.h"
#include "datatype_image.h"
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
    ReportInfo(">> Test 3d gaussian rendering.");
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
    Vec2 uv_2d = Vec2::Zero();
    Mat2 sigma_2d = Mat2::Identity();
    float mid_opacity_2d = 0.0f;
    gaussian_3d.ProjectTo2D(p_wc, q_wc, uv_2d, sigma_2d, mid_opacity_2d);
    const Mat2 inv_sigma_2d = sigma_2d.inverse();

    // Iterate each pixel of image.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - cx) / fx, (row - cy) / fy);
            const Vec2 diff_uv = uv - uv_2d;
            const float opacity = mid_opacity_2d * std::exp(- 0.5f * diff_uv.transpose() * inv_sigma_2d * diff_uv);
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

int main(int argc, char *argv[]) {
    ReportInfo(">> Test 3d gaussian.");
    TestShowOne3DGaussian();
    return 0;
}