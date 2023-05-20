#ifndef _SLAM_UTILITY_COVISIBLE_GRAPH_H_
#define _SLAM_UTILITY_COVISIBLE_GRAPH_H_

#include "visual_observe.h"
#include "visual_feature.h"
#include "visual_frame.h"

#include "unordered_map"
#include "list"

namespace SLAM_UTILITY {

/* Class Covisible Graph Declaration. */
template <typename FeatureParamType, typename FeatureObserveType>
class CovisibleGraph {

using FeatureType = VisualFeature<FeatureParamType, FeatureObserveType>;
using FrameType = VisualFrame<FeatureType>;

public:
    CovisibleGraph() = default;
    virtual ~CovisibleGraph() = default;

    void Clear();

    bool GetCovisibleFeatures(const FrameType &frame_i,
                              const FrameType &frame_j,
                              std::vector<FeatureType> &covisible_features);

    // Reference for member varibles.
    std::list<FrameType> &frames() { return frames_; }
    std::unordered_map<uint32_t, FeatureType> &features() { return features_; }

private:
    // Frames and features in this covisible graph.
    std::list<FrameType> frames_;
    std::unordered_map<uint32_t, FeatureType> features_;

};

/* Class Covisible Graph Definition. */
template <typename FeatureParamType, typename FeatureObserveType>
void CovisibleGraph<FeatureParamType, FeatureObserveType>::Clear() {
    frames_.clear();
    features_.clear();
}


template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::GetCovisibleFeatures(const FrameType &frame_i,
                                                                                const FrameType &frame_j,
                                                                                std::vector<FeatureType> &covisible_features) {

    return false;
}


}

#endif // end of _SLAM_UTILITY_COVISIBLE_GRAPH_H_
