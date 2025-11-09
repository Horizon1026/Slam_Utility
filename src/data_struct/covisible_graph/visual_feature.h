#ifndef _SENSOR_UTILITY_VISUAL_FEATURE_H_
#define _SENSOR_UTILITY_VISUAL_FEATURE_H_

#include "basic_type.h"
#include "slam_log_reporter.h"

namespace slam_utility {

enum FeatureSolvedStatus : uint8_t {
    kUnsolved = 0,
    kSolved = 1,
    kMarginalized = 2,
};

/* Class Visual Feature Declaration. */
template <typename ParamType, typename ObserveType>
class VisualFeature {

public:
    VisualFeature() = delete;
    explicit VisualFeature(uint32_t id): id_(id) {}
    virtual ~VisualFeature() = default;

    void Information() const;

    uint32_t final_frame_id() const { return first_frame_id_ + observes_.size() - 1; }

    // Reference for member variables.
    uint32_t &first_frame_id() { return first_frame_id_; }
    std::vector<ObserveType> &observes() { return observes_; }
    ObserveType &observe(uint32_t frame_id) { return observes_[frame_id - first_frame_id_]; }
    ParamType &param() { return param_; }
    FeatureSolvedStatus &status() { return status_; }

    // Const reference for member variables.
    const uint32_t &id() const { return id_; }
    const uint32_t &first_frame_id() const { return first_frame_id_; }
    const std::vector<ObserveType> &observes() const { return observes_; }
    const ObserveType &observe(uint32_t frame_id) const { return observes_[frame_id - first_frame_id_]; }
    const ParamType &param() const { return param_; }
    const FeatureSolvedStatus &status() const { return status_; }

private:
    // Visual feature id.
    const uint32_t id_ = 0;
    // Id of frame that firstly observe this feature.
    uint32_t first_frame_id_ = 0;
    // Observations in each visual frame. For example, feature point's observe in a mono rectify frame is
    // Eigen::Vector2f.
    std::vector<ObserveType> observes_;
    // Feature parameter can be solved. For example, feature point's parameter type is Eigen::Vector3f.
    ParamType param_ = ParamType::Zero();
    // Solved type of this feature.
    FeatureSolvedStatus status_ = FeatureSolvedStatus::kUnsolved;
};

/* Class Visual Feature Definition. */
template <typename ParamType, typename ObserveType>
void VisualFeature<ParamType, ObserveType>::Information() const {
    ReportInfo("[Visual Feature] Information of feature " << id_
                                                          << "\n"
                                                             " - observed in "
                                                          << observes_.size() << " frames [" << first_frame_id_ << " ~ "
                                                          << first_frame_id_ + observes_.size() - 1
                                                          << "]\n"
                                                             " - solved status is "
                                                          << static_cast<int32_t>(status_));
}

}  // namespace slam_utility

#endif  // end of _SENSOR_UTILITY_VISUAL_FEATURE_H_
