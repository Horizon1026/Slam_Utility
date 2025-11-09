#ifndef _SENSOR_UTILITY_VISUAL_FRAME_H_
#define _SENSOR_UTILITY_VISUAL_FRAME_H_

#include "basic_type.h"
#include "slam_log_reporter.h"

#include "unordered_map"

namespace slam_utility {

/* Class Visual Frame Declaration. */
template <typename FeatureType>
class VisualFrame {

public:
    VisualFrame() = default;
    explicit VisualFrame(uint32_t id): id_(id) {}
    virtual ~VisualFrame() = default;

    void Information() const;
    void SimpleInformation() const;

    // Reference for member variables.
    uint32_t &id() { return id_; }
    Quat &q_wc() { return q_wc_; }
    Vec3 &p_wc() { return p_wc_; }
    float &time_stamp_s() { return time_stamp_s_; }
    std::unordered_map<uint32_t, FeatureType *> &features() { return features_; }
    std::vector<MatImg> &raw_images() { return raw_images_; }
    bool &need_remove() { return need_remove_; }

    // Const reference for member variables.
    const uint32_t &id() const { return id_; }
    const Quat &q_wc() const { return q_wc_; }
    const Vec3 &p_wc() const { return p_wc_; }
    const float &time_stamp_s() const { return time_stamp_s_; }
    const std::unordered_map<uint32_t, FeatureType *> &features() const { return features_; }
    const std::vector<MatImg> &raw_images() const { return raw_images_; }
    const bool &need_remove() const { return need_remove_; }

private:
    // Visual frame id.
    uint32_t id_ = 0;

    // Camera position, velocity and attitude combined with this frame.
    Quat q_wc_ = Quat::Identity();
    Vec3 p_wc_ = Vec3::Zero();
    float time_stamp_s_ = 0.0f;

    // Features observed by this frame.
    std::unordered_map<uint32_t, FeatureType *> features_;

    // Camera images of this frame.
    std::vector<MatImg> raw_images_;

    // Flag for erase in std::list.
    bool need_remove_ = false;
};

/* Class Visual Frame Definition. */
template <typename FeatureType>
void VisualFrame<FeatureType>::Information() const {
    ReportInfo("[Visual Frame] Information of frame " << id_
                                                      << "\n"
                                                         " - time stamp is "
                                                      << time_stamp_s_
                                                      << " s\n"
                                                         " - q_wc is "
                                                      << LogQuat(q_wc_)
                                                      << "\n"
                                                         " - p_wc is "
                                                      << LogVec(p_wc_)
                                                      << "\n"
                                                         " - number of observed features is "
                                                      << features_.size());
    ReportText(" - List of features observed in this frame:\n");
    for (const auto &item: features_) {
        ReportText("   - id/id " << item.first << "/" << item.second->id() << ", " << LogPtr(item.second) << "\n");
    }
    ReportText("\n");
}

template <typename FeatureType>
void VisualFrame<FeatureType>::SimpleInformation() const {
    ReportInfo("[Visual Frame] Information of frame " << id_
                                                      << "\n"
                                                         " - time stamp is "
                                                      << time_stamp_s_
                                                      << " s\n"
                                                         " - q_wc is "
                                                      << LogQuat(q_wc_)
                                                      << "\n"
                                                         " - p_wc is "
                                                      << LogVec(p_wc_)
                                                      << "\n"
                                                         " - number of observed features is "
                                                      << features_.size());
}

}  // namespace slam_utility

#endif  // end of _SENSOR_UTILITY_VISUAL_FRAME_H_
