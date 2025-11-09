#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "visualizor_3d.h"

#include "kd_tree.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

constexpr int32_t kNumOfPointsInOneDimension = 10;

void TestArgSort() {
    ReportColorInfo(">> Test arg sort.");

    std::vector<int32_t> values;
    for (int32_t i = 0; i < 10; ++i) {
        values.emplace_back(std::rand());
    }

    std::vector<int32_t> indices {0, 2, 5};
    SlamOperation::ArgSort(values, indices);

    for (uint32_t i = 0; i < indices.size(); ++i) {
        ReportInfo("[" << indices[i] << "] " << values[i] << " -> " << values[indices[i]]);
    }
}

void TestArgSortVector() {
    ReportColorInfo(">> Test arg sort vector.");

    std::vector<Vec3> values;
    for (int32_t i = 0; i < 10; ++i) {
        values.emplace_back(Vec3::Random());
    }

    std::vector<int32_t> indices {0, 2, 5};
    SlamOperation::ArgSortVector(values, 0, indices);

    for (uint32_t i = 0; i < indices.size(); ++i) {
        ReportInfo("[" << indices[i] << "] " << LogVec(values[i]) << " -> " << LogVec(values[indices[i]]));
    }
}

void TestKdTreeConstruction() {
    ReportColorInfo(">> Test kd tree construction.");

    // Create points cloud.
    std::vector<Vec3> raw_points;
    raw_points.reserve(kNumOfPointsInOneDimension * kNumOfPointsInOneDimension * kNumOfPointsInOneDimension);
    for (int32_t i = 0; i < kNumOfPointsInOneDimension; ++i) {
        for (int32_t j = 0; j < kNumOfPointsInOneDimension; ++j) {
            for (int32_t k = 0; k < kNumOfPointsInOneDimension; ++k) {
                raw_points.emplace_back(Vec3(i, j, k));
            }
        }
    }

    // Create kd-tree to find nearest points.
    std::vector<int32_t> sorted_point_indices(raw_points.size(), 0);
    for (uint32_t i = 0; i < sorted_point_indices.size(); ++i) {
        sorted_point_indices[i] = i;
    }
    std::unique_ptr<KdTreeNode<float, 3>> kd_tree_ptr = std::make_unique<KdTreeNode<float, 3>>();
    kd_tree_ptr->Construct(raw_points, sorted_point_indices, kd_tree_ptr);
    ReportInfo("Construct a kd-tree with depth " << kd_tree_ptr->GetDepth(kd_tree_ptr));

    // Extract all points in kd-tree.
    std::vector<int32_t> index_of_points;
    kd_tree_ptr->ExtractAllPoints(kd_tree_ptr, raw_points, index_of_points);

    // Visualize result.
    Visualizor3D::Clear();
    for (const auto &index: index_of_points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[index],
            .color = RgbColor::kCyan,
            .radius = 3,
        });
    }

    // Extract half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr(), raw_points, index_of_points);
    for (const auto &index: index_of_points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[index],
            .color = RgbColor::kRed,
            .radius = 3,
        });
    }

    // Extract half-half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr()->left_ptr(), raw_points, index_of_points);
    for (const auto &index: index_of_points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[index],
            .color = RgbColor::kYellow,
            .radius = 3,
        });
    }

    // Extract half-half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->left_ptr()->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr()->left_ptr()->left_ptr(), raw_points, index_of_points);
    for (const auto &index: index_of_points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[index],
            .color = RgbColor::kWhite,
            .radius = 3,
        });
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Constructed kd-tree", 30);
    }
}

void TestKdTreeSearch() {
    ReportColorInfo(">> Test kd tree search knn and radius.");

    // Create points cloud.
    std::vector<Vec3> raw_points;
    raw_points.reserve(kNumOfPointsInOneDimension * kNumOfPointsInOneDimension * kNumOfPointsInOneDimension);
    for (int32_t i = 0; i < kNumOfPointsInOneDimension; ++i) {
        for (int32_t j = 0; j < kNumOfPointsInOneDimension; ++j) {
            for (int32_t k = 0; k < kNumOfPointsInOneDimension; ++k) {
                raw_points.emplace_back(Vec3(i, j, k));
            }
        }
    }

    // Create kd-tree to find nearest points.
    std::vector<int32_t> sorted_point_indices(raw_points.size(), 0);
    for (uint32_t i = 0; i < sorted_point_indices.size(); ++i) {
        sorted_point_indices[i] = i;
    }
    std::unique_ptr<KdTreeNode<float, 3>> kd_tree_ptr = std::make_unique<KdTreeNode<float, 3>>();
    kd_tree_ptr->Construct(raw_points, sorted_point_indices, kd_tree_ptr);
    ReportInfo("Construct a kd-tree with depth " << kd_tree_ptr->GetDepth(kd_tree_ptr));

    // Extract all points in kd-tree.
    std::vector<int32_t> index_of_points;
    kd_tree_ptr->ExtractAllPoints(kd_tree_ptr, raw_points, index_of_points);

    // Visualize result full kd-tree.
    Visualizor3D::Clear();
    for (const auto &index: index_of_points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[index],
            .color = RgbColor::kCyan,
            .radius = 2,
        });
    }

    // Create target and do search.
    const Vec3 target_point = Vec3(2.2, 3.4, 5.8);
    std::multimap<float, int32_t> result_of_radius;
    kd_tree_ptr->SearchRadius(kd_tree_ptr, raw_points, target_point, 3.0f, result_of_radius);
    std::multimap<float, int32_t> result_of_knn;
    kd_tree_ptr->SearchKnn(kd_tree_ptr, raw_points, target_point, 4, result_of_knn);
    std::multimap<float, int32_t> result_of_cube;
    kd_tree_ptr->SearchCube(kd_tree_ptr, raw_points, Vec3(4.9, 5.2, 6.2), Vec3(8.1, 9.2, 8.9), result_of_cube);

    // Visualize target and result.
    Visualizor3D::points().emplace_back(PointType {
        .p_w = target_point,
        .color = RgbColor::kGold,
        .radius = 5,
    });
    for (const auto &pair: result_of_radius) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[pair.second],
            .color = RgbColor::kRed,
            .radius = 3,
        });
    }
    for (const auto &pair: result_of_cube) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[pair.second],
            .color = RgbColor::kOrange,
            .radius = 3,
        });
    }
    for (const auto &pair: result_of_knn) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = raw_points[pair.second],
            .color = RgbColor::kRoyalBlue,
            .radius = 3,
        });
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Searched result in kd-tree", 30);
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test kd tree." RESET_COLOR);
    TestArgSort();
    TestArgSortVector();
    TestKdTreeConstruction();
    TestKdTreeSearch();
    return 0;
}
