#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "visualizor_3d.h"
#include "extend_kd_tree.h"
#include "numeric"

using namespace slam_utility;
using namespace slam_visualizor;

constexpr int32_t kNumOfPointsInOneDimension = 10;

// Dummy data structure to simulate arbitrary data units (like MapKeyframe)
struct DummyData {
    Vec3 position;
    int32_t id;
};

void TestExtendKdTreeConstruction() {
    ReportColorInfo(">> Test extend kd tree construction.");

    // Create custom data units.
    std::vector<DummyData> data_units;
    data_units.reserve(kNumOfPointsInOneDimension * kNumOfPointsInOneDimension * kNumOfPointsInOneDimension);
    for (int32_t i = 0; i < kNumOfPointsInOneDimension; ++i) {
        for (int32_t j = 0; j < kNumOfPointsInOneDimension; ++j) {
            for (int32_t k = 0; k < kNumOfPointsInOneDimension; ++k) {
                data_units.push_back({Vec3(i, j, k), i * 100 + j * 10 + k});
            }
        }
    }

    // Lambda accessor to get position from data unit.
    auto accessor = [&](int32_t index) -> Vec3 {
        return data_units[index].position;
    };

    // Prepare indices.
    std::vector<int32_t> indices(data_units.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Build extend kd-tree.
    using KdTreeType = ExtendKdTreeNode<float, 3>;
    std::unique_ptr<KdTreeType> kd_tree_ptr = std::make_unique<KdTreeType>();
    kd_tree_ptr->Construct(indices, accessor);
    ReportInfo("Constructed an extend kd-tree with depth " << kd_tree_ptr->GetDepth());

    // Extract all points and visualize.
    std::vector<int32_t> extracted_indices;
    kd_tree_ptr->ExtractAllPoints(extracted_indices);
    ReportInfo("Extracted " << extracted_indices.size() << " points from extend kd-tree.");
}

void TestExtendKdTreeSearch() {
    ReportColorInfo(">> Test extend kd tree search.");

    // Create custom data units.
    std::vector<DummyData> data_units;
    for (int32_t i = 0; i < kNumOfPointsInOneDimension; ++i) {
        for (int32_t j = 0; j < kNumOfPointsInOneDimension; ++j) {
            for (int32_t k = 0; k < kNumOfPointsInOneDimension; ++k) {
                data_units.push_back({Vec3(i, j, k), i * 100 + j * 10 + k});
            }
        }
    }

    auto accessor = [&](int32_t index) -> Vec3 {
        return data_units[index].position;
    };

    std::vector<int32_t> indices(data_units.size());
    std::iota(indices.begin(), indices.end(), 0);

    using KdTreeType = ExtendKdTreeNode<float, 3>;
    std::unique_ptr<KdTreeType> kd_tree_ptr = std::make_unique<KdTreeType>();
    kd_tree_ptr->Construct(indices, accessor);

    // Target point and searches.
    const Vec3 target_point = Vec3(2.5, 3.5, 4.5);

    std::multimap<float, int32_t> result_of_radius;
    kd_tree_ptr->SearchRadius(target_point, 3.0f, accessor, result_of_radius);
    ReportInfo("SearchRadius found " << result_of_radius.size() << " points.");

    std::multimap<float, int32_t> result_of_knn;
    kd_tree_ptr->SearchKnn(target_point, 5, accessor, result_of_knn);
    ReportInfo("SearchKnn found " << result_of_knn.size() << " points.");

    std::multimap<float, int32_t> result_of_cube;
    kd_tree_ptr->SearchCube(Vec3(1, 1, 1), Vec3(3, 3, 3), accessor, result_of_cube);
    ReportInfo("SearchCube found " << result_of_cube.size() << " points.");

    // Visualization.
    Visualizor3D::Clear();
    // Raw points.
    for (const auto &data : data_units) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = data.position,
            .color = RgbColor::kCyan,
            .radius = 1,
        });
    }
    // Target.
    Visualizor3D::points().emplace_back(PointType {
        .p_w = target_point,
        .color = RgbColor::kGold,
        .radius = 5,
    });
    // Radius results.
    for (const auto &pair : result_of_radius) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = data_units[pair.second].position,
            .color = RgbColor::kGreen,
            .radius = 3,
        });
    }
    // KNN results.
    for (const auto &pair : result_of_knn) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = data_units[pair.second].position,
            .color = RgbColor::kRoyalBlue,
            .radius = 3,
        });
    }
    // Cube results.
    for (const auto &pair : result_of_cube) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = data_units[pair.second].position,
            .color = RgbColor::kRed,
            .radius = 3,
        });
    }

    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Extend kd-tree search results", 30);
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test extend kd tree." RESET_COLOR);
    TestExtendKdTreeConstruction();
    TestExtendKdTreeSearch();
    return 0;
}
