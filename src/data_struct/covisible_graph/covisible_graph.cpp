#include "covisible_graph.h"

namespace SLAM_UTILITY {

// Compute summary of reprojection residual.
template <>
float CovisibleGraph<Vec3, Vec2>::ComputeResidual() {
    float summary_residual = 0.0f;

    for (auto &frame : frames_) {
        for (auto &item : frame.features()) {
            auto &feature = item.second;

            // Compute reprojection residual.
            const Vec3 p_c = frame.q_wc().inverse() * (feature->param() - frame.p_wc());
            const Vec2 residual = p_c.head<2>() / p_c.z() - feature->observe(frame.id());
            summary_residual += residual.norm();
        }
    }

    return summary_residual;
}

}
