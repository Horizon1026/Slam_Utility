#include "log_report.h"
#include "datatype_basic.h"
#include "covisible_graph.h"

constexpr int32_t kCameraFrameNumber = 4;
constexpr int32_t kPointsNumber = 5;

struct Pose {
    Quat q_wc = Quat::Identity();
    Vec3 p_wc = Vec3::Zero();
};

void GenerateSimulationData(std::vector<Pose> &cameras,
                            std::vector<Vec3> &points) {
    cameras.clear();
    points.clear();

    // Cameras.
    for (int32_t i = 0; i < kCameraFrameNumber; ++i) {
        Pose camera_pose;
        camera_pose.q_wc.setIdentity();
        camera_pose.p_wc = Vec3(i, i, 0);
        cameras.emplace_back(camera_pose);
    }

    // Points.
    for (int32_t i = 0; i < 20; ++i) {
        for (int32_t j = 0; j < 20; ++j) {
            const Vec3 point(i, j, 5);
            points.emplace_back(point);
            if (points.size() == kPointsNumber) {
                return;
            }
        }
    }
}

CovisibleGraph<Vec3, Vec2> CreateGraph(std::vector<Pose> &cameras,
                                       std::vector<Vec3> &points) {
    // Generate data for each frame.
    std::vector<uint32_t> features_id;
    std::vector<Vec2> features_observe;
    features_id.reserve(kPointsNumber);
    features_observe.reserve(kPointsNumber);

    // Construct covisible graph, add cameras and points.
    CovisibleGraph<Vec3, Vec2> graph;
    for (int32_t i = 0; i < kCameraFrameNumber; ++i) {
        features_observe.clear();
        features_id.clear();

        // Use 'kPointsNumber - i' to simulate the situation of tracking loss.
        for (int32_t j = 0; j < kPointsNumber - i; ++j) {
            const Vec3 p_c = cameras[i].q_wc.inverse() * (points[j] - cameras[i].p_wc);
            const Vec2 obv = p_c.head<2>() / p_c.z();
            features_observe.emplace_back(obv);
            features_id.emplace_back(j + 1);
        }

        // Add this frame into covisible graph.
        graph.AddNewFrameWithFeatures(features_id, features_observe, 1.0f);
    }

    return graph;
}

void PrintCheckResult(CovisibleGraph<Vec3, Vec2> &graph) {
    // Show all information of this covisible graph.
    // graph.Information();

    // Check if the covisible graph is invalid.
    if (graph.SelfCheck()) {
        ReportInfo("Covisible graph self check ok.");
    } else {
        ReportError("Covisible graph self check failed.");
    }
}

void TestCovisibleGraphGeneration(std::vector<Pose> &cameras,
                                  std::vector<Vec3> &points) {
    ReportInfo(YELLOW ">> Test covisible graph generation." RESET_COLOR);
    CovisibleGraph<Vec3, Vec2> graph = CreateGraph(cameras, points);

    PrintCheckResult(graph);
}

void TestCovisibleGraphRemoveFeatures(std::vector<Pose> &cameras,
                                      std::vector<Vec3> &points,
                                      uint32_t removed_feature_id) {
    ReportInfo(YELLOW ">> Test covisible graph remove features." RESET_COLOR);
    CovisibleGraph<Vec3, Vec2> graph = CreateGraph(cameras, points);

    graph.RemoveFeature(removed_feature_id);

    PrintCheckResult(graph);
}

void TestCovisibleGraphRemoveFrames(std::vector<Pose> &cameras,
                                    std::vector<Vec3> &points,
                                    uint32_t remove_frame_id) {
    ReportInfo(YELLOW ">> Test covisible graph remove features." RESET_COLOR);
    CovisibleGraph<Vec3, Vec2> graph = CreateGraph(cameras, points);

    graph.RemoveFrame(remove_frame_id);

    PrintCheckResult(graph);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test covisible graph." RESET_COLOR);

    // Generate simulate data.
    std::vector<Pose> cameras;
    std::vector<Vec3> points;
    GenerateSimulationData(cameras, points);

    // Run test.
    TestCovisibleGraphGeneration(cameras, points);
    for (uint32_t i = 0; i < kPointsNumber + 1; ++i) {
        TestCovisibleGraphRemoveFeatures(cameras, points, i);
    }
    for (uint32_t i = 0; i < kCameraFrameNumber + 1; ++i) {
        TestCovisibleGraphRemoveFrames(cameras, points, i);
    }

    return 0;
}
