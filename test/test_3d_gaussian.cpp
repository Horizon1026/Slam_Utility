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
    Gaussian2D gaussian_2d;
    gaussian_3d.ProjectTo2D(p_wc, q_wc, gaussian_2d);
    const Mat2 inv_sigma_2d = gaussian_2d.sigma().inverse();

    // Iterate each pixel of image.
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

int main(int argc, char *argv[]) {
    ReportInfo(">> Test 3d gaussian.");
    TestShowOne3DGaussian();
    return 0;
}