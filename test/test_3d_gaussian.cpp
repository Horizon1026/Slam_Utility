#include "datatype_image.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "2d_gaussian.h"
#include "3d_gaussian.h"

#include "visualizor_2d.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

constexpr int32_t image_rows = 1000;
constexpr int32_t image_cols = 1000;
constexpr float fx = 800.0f;
constexpr float fy = 800.0f;
constexpr float cx = 400.0f;
constexpr float cy = 400.0f;

void TestShowOne3DGaussian() {
    ReportColorInfo(">> Test only one 3d gaussian rendering.");
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

    // Iterate each pixel of image to compute color.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - cx) / fx, (row - cy) / fy);
            const float opacity = gaussian_2d.GetOpacityAt(uv);
            const RgbPixel pixel_color = RgbPixel {
                .r = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().r) * opacity),
                .g = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().g) * opacity),
                .b = static_cast<uint8_t>(static_cast<float>(gaussian_3d.color().b) * opacity),
            };
            show_image.SetPixelValueNoCheck(row, col, pixel_color);
        }
    }

    Visualizor2D::ShowImage("show image", show_image);
    Visualizor2D::WaitKey(0);
}

void TestShowSeveral3DGaussian() {
    ReportColorInfo(">> Test several 3d gaussian rendering.");
    // Camera view.
    const Quat q_wc = Quat::Identity();
    const Vec3 p_wc = Vec3(6, 5, -20);

    // Allocate for show image.
    uint8_t *buf = (uint8_t *)malloc(image_rows * image_cols * 3 * sizeof(uint8_t));
    RgbImage show_image(buf, image_rows, image_cols, true);

    // Color of 3d gaussians.
    std::vector<RgbPixel> all_colors = {RgbColor::kGreen, RgbColor::kRed, RgbColor::kBrown, RgbColor::kGold};

    // Create 3d gaussian.
    std::vector<Gaussian3D> all_gaussian_3d;
    for (uint32_t i = 0; i < all_colors.size(); ++i) {
        Gaussian3D gaussian_3d;
        gaussian_3d.color() = all_colors[i];
        gaussian_3d.p_w() = Vec3(i, i * 4.0f, 2.5f + i * 3.0f);
        gaussian_3d.mid_opacity() = 1.0f / static_cast<float>(i + 1);
        gaussian_3d.sigma_s() = Vec3(1, 2, 3);
        gaussian_3d.sigma_q() = Quat::Identity();
        gaussian_3d.sh_colors()[0].coeff().setRandom();
        gaussian_3d.sh_colors()[1].coeff().setRandom();
        gaussian_3d.sh_colors()[2].coeff().setRandom();
        all_gaussian_3d.emplace_back(gaussian_3d);
    }

    // Project 3d gaussian to 2d.
    std::vector<Gaussian2D> all_gaussian_2d;
    std::vector<float> all_gaussian_depth;
    for (uint32_t i = 0; i < all_gaussian_3d.size(); ++i) {
        Gaussian2D gaussian_2d;
        all_gaussian_3d[i].ProjectTo2D(p_wc, q_wc, gaussian_2d);
        gaussian_2d.mid_opacity() = 1.0f;
        all_gaussian_2d.emplace_back(gaussian_2d);
        all_gaussian_depth.emplace_back(gaussian_2d.depth());
    }

    // Sort gaussians by depth.
    std::vector<int32_t> indices;
    SlamOperation::ArgSort(all_gaussian_depth, indices);
    for (const auto &index: indices) {
        const auto &gaussian_2d = all_gaussian_2d[index];
        ReportInfo(all_gaussian_2d[index].depth() << ", rgb " << static_cast<int32_t>(gaussian_2d.color().r) << " | "
                                                  << static_cast<int32_t>(gaussian_2d.color().g) << " | " << static_cast<int32_t>(gaussian_2d.color().b));
    }

    // Iterate each pixel of image to compute color.
    for (int32_t row = 0; row < image_rows; ++row) {
        for (int32_t col = 0; col < image_cols; ++col) {
            const Vec2 uv = Vec2((col - cx) / fx, (row - cy) / fy);

            float occluded_probability = 1.0f;
            Vec3 float_color = Vec3::Zero();
            for (const auto &index: indices) {
                const auto &gaussian_2d = all_gaussian_2d[index];
                const float powered_alpha = gaussian_2d.GetOpacityAt(uv, gaussian_2d.inv_sigma());
                CONTINUE_IF(powered_alpha < static_cast<float>(1.0f / 255.0f));

                const Vec3 rgb_color = Vec3(gaussian_2d.color().r, gaussian_2d.color().g, gaussian_2d.color().b);
                float_color += rgb_color * powered_alpha * occluded_probability;
                occluded_probability *= 1.0f - powered_alpha;
                BREAK_IF(occluded_probability < 1e-3f);
            }

            const RgbPixel pixel_color = RgbPixel {
                .r = static_cast<uint8_t>(std::min(255.0f, float_color.x())),
                .g = static_cast<uint8_t>(std::min(255.0f, float_color.y())),
                .b = static_cast<uint8_t>(std::min(255.0f, float_color.z())),
            };
            show_image.SetPixelValueNoCheck(row, col, pixel_color);
        }
    }

    Visualizor2D::ShowImage("show image", show_image);
    Visualizor2D::WaitKey(0);
}

int main(int argc, char *argv[]) {
    ReportInfo(YELLOW ">> Test 3d gaussian." RESET_COLOR);
    TestShowOne3DGaussian();
    TestShowSeveral3DGaussian();
    return 0;
}
