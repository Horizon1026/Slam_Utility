#include "datatype_basic.h"
#include "log_report.h"
#include "visualizor_3d.h"

#include "kd_tree.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test kd tree." RESET_COLOR);

    // Create points cloud.
    std::vector<Vec3> raw_points;
    raw_points.reserve(1000);
    for (int32_t i = 0; i < 10; ++i) {
        for (int32_t j = 0; j < 10; ++j) {
            for (int32_t k = 0; k < 10; ++k) {
                raw_points.emplace_back(Vec3(i, j, k));
            }
        }
    }

    // Create kd-tree to find nearest points.
    // TODO:

    // Visualize result.
    Visualizor3D::Clear();
    for (const auto &point : raw_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = point,
            .color = RgbColor::kCyan,
            .radius = 2,
        });
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Visualizor 3D", 30);
    }

    return 0;
}
