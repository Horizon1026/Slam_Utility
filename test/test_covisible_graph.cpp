#include "slam_log_reporter.h"
#include "basic_type.h"
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
    std::vector<Vec3> features_param;
    features_id.reserve(kPointsNumber);
    features_observe.reserve(kPointsNumber);
    features_param.reserve(kPointsNumber);

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
            features_param.emplace_back(points[j]);
        }

        // Add this frame into covisible graph.
        graph.AddNewFrameWithFeatures(features_id, features_observe, features_param,
        	6.0f, cameras[i].q_wc, cameras[i].p_wc);
    }

    return graph;
}

void PrintCheckResult(CovisibleGraph<Vec3, Vec2> &graph, bool show_information = false) {
    // Show all information of this covisible graph.
    if (show_information) {
        graph.Information();
    }
    ReportInfo("Covisible graph summary residual is " << graph.ComputeResidual() << ". It must be 0.");

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
    ReportInfo(YELLOW ">> Test covisible graph remove frames." RESET_COLOR);
    CovisibleGraph<Vec3, Vec2> graph = CreateGraph(cameras, points);
    graph.RemoveFrame(remove_frame_id);
    PrintCheckResult(graph);
}

void TestGettingCovisibleFeatures(std::vector<Pose> &cameras,
                                  std::vector<Vec3> &points) {
    ReportInfo(YELLOW ">> Test covisible graph get covisible features." RESET_COLOR);
    CovisibleGraph<Vec3, Vec2> graph = CreateGraph(cameras, points);

    VisualFrame<VisualFeature<Vec3, Vec2>> frame_i = graph.frames().front();
    VisualFrame<VisualFeature<Vec3, Vec2>> frame_j = graph.frames().back();
    std::vector<VisualFeature<Vec3, Vec2> *> covisible_features;
    graph.GetCovisibleFeatures(frame_i, frame_j, covisible_features);
    for (auto &feature : covisible_features) {
        ReportInfo(" - feature " << feature->id() << " is covisible, ptr is " << LogPtr(feature));
    }
    PrintCheckResult(graph, true);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test covisible graph." RESET_COLOR);

    // Generate simulate data.
    std::vector<Pose> cameras;
    std::vector<Vec3> points;
    GenerateSimulationData(cameras, points);

    // Run test.
    TestCovisibleGraphGeneration(cameras, points);
    TestGettingCovisibleFeatures(cameras, points);

    for (uint32_t i = 0; i < kPointsNumber + 1; ++i) {
        TestCovisibleGraphRemoveFeatures(cameras, points, i);
    }
    for (uint32_t i = 0; i < kCameraFrameNumber + 1; ++i) {
        TestCovisibleGraphRemoveFrames(cameras, points, i);
    }

    return 0;
}
