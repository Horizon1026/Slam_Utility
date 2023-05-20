#ifndef _SENSOR_UTILITY_VISUAL_FRAME_H_
#define _SENSOR_UTILITY_VISUAL_FRAME_H_

#include "datatype_basic.h"
#include "log_report.h"

#include "unordered_map"

namespace SLAM_UTILITY {

/* Class Visual Frame Declaration. */
template <typename FeatureType>
class VisualFrame {

public:
    VisualFrame() = default;
    explicit VisualFrame(uint32_t id) : id_(id) {}
    virtual ~VisualFrame() = default;

    void Information() const;

    // Reference for member variables.
    uint32_t &id() { return id_; }
    Quat &q_wc() { return q_wc_; }
    Vec3 &p_wc() { return p_wc_; }
    Vec3 &v_wc() { return v_wc_; }
    float &time_stamp_s() { return time_stamp_s_; }
    std::unordered_map<uint32_t, FeatureType *> &features() { return features_; }

private:
    // Visual frame id.
    uint32_t id_ = 0;

    // Camera position, velocity and attitude combined with this frame.
    Quat q_wc_ = Quat::Identity();
    Vec3 p_wc_ = Vec3::Zero();
    Vec3 v_wc_ = Vec3::Zero();
    float time_stamp_s_ = 0.0f;

    // Features observed by this frame.
    std::unordered_map<uint32_t, FeatureType *> features_;

};

/* Class Visual Frame Definition. */
template <typename FeatureType>
void VisualFrame<FeatureType>::Information() const {
    ReportInfo("[Visual Frame] Information of frame " << id_ << "\n"
        " - time stamp is " << time_stamp_s_ << " s\n"
        " - q_wc is " << LogQuat(q_wc_) << "\n"
        " - p_wc is " << LogVec(p_wc_) << "\n"
        " - v_wc is " << LogVec(v_wc_) << "\n"
        " - number of observed features is " << features_.size()
    );
    ReportText(" - List of features observed in this frame:\n");
    for (const auto &item : features_) {
        ReportText("   - " << item.first << ", " << LogPtr(item.second));
    }
    ReportText("\n");
}

}

#endif // end of _SENSOR_UTILITY_VISUAL_FRAME_H_
