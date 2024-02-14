#include "datatype_basic.h"
#include "log_report.h"
#include "slam_operations.h"

#include "visualizor_3d.h"

#include "kd_tree.h"

using namespace SLAM_UTILITY;
using namespace SLAM_VISUALIZOR;

constexpr int32_t kNumOfPointsInOneDimension = 10;

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

void TestKdTreeConstruction() {
    ReportInfo(YELLOW ">> Test kd tree construction." RESET_COLOR);

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

    // Extract all points in kd-tree.
    std::vector<int32_t> index_of_points;
    kd_tree_ptr->ExtractAllPoints(kd_tree_ptr, raw_points, index_of_points);

    // Visualize result.
    Visualizor3D::Clear();
    for (const auto &index : index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[index],
            .color = RgbColor::kCyan,
            .radius = 3,
        });
    }

    // Extract half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr(), raw_points, index_of_points);
    for (const auto &index : index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[index],
            .color = RgbColor::kRed,
            .radius = 3,
        });
    }

    // Extract half-half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr()->left_ptr(), raw_points, index_of_points);
    for (const auto &index : index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[index],
            .color = RgbColor::kYellow,
            .radius = 3,
        });
    }

    // Extract half-half points in kd-tree.
    index_of_points.clear();
    kd_tree_ptr->left_ptr()->left_ptr()->left_ptr()->ExtractAllPoints(kd_tree_ptr->left_ptr()->left_ptr()->left_ptr(), raw_points, index_of_points);
    for (const auto &index : index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
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
    ReportInfo(YELLOW ">> Test kd tree search knn and radius." RESET_COLOR);

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

    // Extract all points in kd-tree.
    std::vector<int32_t> index_of_points;
    kd_tree_ptr->ExtractAllPoints(kd_tree_ptr, raw_points, index_of_points);

    // Visualize result full kd-tree.
    Visualizor3D::Clear();
    for (const auto &index : index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[index],
            .color = RgbColor::kCyan,
            .radius = 2,
        });
    }

    // Create target and do search.
    const Vec3 target_point = Vec3(2.2, 3.4, 5.8);
    std::multimap<float, int32_t> residual_index_of_points;
    // kd_tree_ptr->SearchKnn(kd_tree_ptr, raw_points, target_point, 7, residual_index_of_points);
    kd_tree_ptr->SearchRadius(kd_tree_ptr, raw_points, target_point, 2.0f, residual_index_of_points);

    // Visualize target and result.
    Visualizor3D::points().emplace_back(PointType{
        .p_w = target_point,
        .color = RgbColor::kGold,
        .radius = 5,
    });
    for (const auto &pair : residual_index_of_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = raw_points[pair.second],
            .color = RgbColor::kRed,
            .radius = 3,
        });
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Searched result in kd-tree", 30);
    }
}

int main(int argc, char **argv) {

    TestArgSort();
    TestArgSortVector();

    // TestKdTreeConstruction();
    TestKdTreeSearch();

    return 0;
}
