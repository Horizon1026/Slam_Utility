#include "datatype_basic.h"
#include "log_report.h"
#include "slam_operations.h"

#include "visualizor_3d.h"

#include "kd_tree.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

void TestArgSort() {
    ReportInfo(YELLOW ">> Test arg sort." RESET_COLOR);

    std::vector<int32_t> values;
    for (int32_t i = 0; i < 10; ++i) {
        values.emplace_back(std::rand());
    }

    std::vector<int32_t> indices{0, 2, 5};
    SlamOperation::ArgSort(values, indices);

    for (uint32_t i = 0; i < indices.size(); ++i) {
        ReportInfo("[" << indices[i] << "] " << values[i] << " -> " << values[indices[i]]);
    }
}

void TestArgSortVector() {
    ReportInfo(YELLOW ">> Test arg sort vector." RESET_COLOR);

    std::vector<Vec3> values;
    for (int32_t i = 0; i < 10; ++i) {
        values.emplace_back(Vec3::Random());
    }

    std::vector<int32_t> indices{0, 2, 5};
    SlamOperation::ArgSortVector(values, 0, indices);

    for (uint32_t i = 0; i < indices.size(); ++i) {
        ReportInfo("[" << indices[i] << "] " << LogVec(values[i]) << " -> " << LogVec(values[indices[i]]));
    }
}

void TestKdTree() {
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

    // Create a new point.
    Vec3 target_point = Vec3(4.5, 5.1, 6.2);

    // Create kd-tree to find nearest points.
    std::vector<int32_t> sorted_point_indices(raw_points.size(), 0);
    for (uint32_t i = 0; i < sorted_point_indices.size(); ++i) {
        sorted_point_indices[i] = i;
    }
    std::unique_ptr<KdTreeNode<float, 3>> kd_tree_ptr = std::make_unique<KdTreeNode<float, 3>>();
    kd_tree_ptr->Construct(raw_points, sorted_point_indices, 0, kd_tree_ptr);
    // kd_tree_ptr->InformationRecursion();

    // Search knn.
    std::map<float, int32_t> residual_index_of_points;
    kd_tree_ptr->SearchKnn(kd_tree_ptr, raw_points, target_point, 5, residual_index_of_points);

    // Visualize result.
    Visualizor3D::Clear();
    for (const auto &point : raw_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = point,
            .color = RgbColor::kCyan,
            .radius = 2,
        });
    }
    Visualizor3D::points().emplace_back(PointType{
        .p_w = target_point,
        .color = RgbColor::kHotPink,
        .radius = 4,
    });
    for (const auto &pair : residual_index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[pair.second],
            .color = RgbColor::kGreen,
            .radius = 4,
        });

        Visualizor3D::strings().emplace_back(std::to_string(pair.first));
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Visualizor 3D", 30);
    }
}

int main(int argc, char **argv) {

    TestArgSort();
    TestArgSortVector();
    TestKdTree();

    return 0;
}
