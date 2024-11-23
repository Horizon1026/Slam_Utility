#ifndef _SLAM_UTILITY_COVISIBLE_GRAPH_H_
#define _SLAM_UTILITY_COVISIBLE_GRAPH_H_

#include "visual_observe.h"
#include "visual_feature.h"
#include "visual_frame.h"

#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "unordered_map"
#include "deque"

namespace SLAM_UTILITY {

/* Class Covisible Graph Declaration. */
template <typename FeatureParamType, typename FeatureObserveType>
class CovisibleGraph {

using FeatureType = VisualFeature<FeatureParamType, FeatureObserveType>;
using FrameType = VisualFrame<FeatureType>;

public:
    CovisibleGraph() = default;
    virtual ~CovisibleGraph() = default;

    // Clear all storage in this covisible graph.
    void Clear();

    // Check this covisible graph, every observe between features and frames must be valid.
    bool SelfCheck();

    // Print all information of this covisible graph.
    void Information() const;

    // Add new frame and new features in it.
    bool AddNewFrameWithFeatures(const std::vector<uint32_t> &features_id,
                                 const std::vector<FeatureObserveType> &features_observe,
                                 const float time_stamp_s,
                                 const int32_t new_frame_id = -1,
                                 const std::vector<MatImg> &raw_images = std::vector<MatImg>());

    // Add new frame and new features in it.
    bool AddNewFrameWithFeatures(const std::vector<uint32_t> &features_id,
                                 const std::vector<FeatureObserveType> &features_observe,
                                 const std::vector<FeatureParamType> &features_param,
                                 const float time_stamp_s,
                                 const Quat &q_wc = Quat::Identity(),
                                 const Vec3 &p_wc = Vec3::Zero(),
                                 const std::vector<MatImg> &raw_images = std::vector<MatImg>());

    // Remove feature by feature_id.
    bool RemoveFeature(uint32_t feature_id);

    // Remove frame by frame_id according to rules below.
    // [1, 2, 3, 4, [5], 6] -> [1, 2, 3, 4, 5(6)], convert the old 6 to be new 5.
    // [[1], 2, 3, 4, 5, 6] -> [(1), (2), (3), (4), (5)], convert the old 2 ~ 6 to be new 1 ~ 5.
    bool RemoveFrame(uint32_t frame_id);

    // Compute summary of reprojection residual.
    float ComputeResidual();

    // Get covisible features between two frames.
    bool GetCovisibleFeatures(const uint32_t frame_i_id,
                              const uint32_t frame_j_id,
                              std::vector<FeatureType *> &covisible_features);

    // Get covisible features between two frames.
    bool GetCovisibleFeatures(const FrameType &frame_i,
                              const FrameType &frame_j,
                              std::vector<FeatureType *> &covisible_features);


    // Find frame or feature by id.
    inline FrameType *frame(uint32_t id);
    inline FeatureType *feature(uint32_t id);

    // Reference for member variables.
    std::deque<FrameType> &frames() { return frames_; }
    std::unordered_map<uint32_t, FeatureType> &features() { return features_; }

    // Const reference for member variables.
    const std::deque<FrameType> &frames() const { return frames_; }
    const std::unordered_map<uint32_t, FeatureType> &features() const { return features_; }

private:
    // Remove frame in std::list by check 'need_remove' flag.
    void RemoveFrameNeedRemove();

private:
    // Frames and features in this covisible graph.
    std::deque<FrameType> frames_;
    std::unordered_map<uint32_t, FeatureType> features_;

};

/* Class Covisible Graph Definition. */
template <typename FeatureParamType, typename FeatureObserveType>
void CovisibleGraph<FeatureParamType, FeatureObserveType>::Clear() {
    frames_.clear();
    features_.clear();
}

template <typename FeatureParamType, typename FeatureObserveType>
void CovisibleGraph<FeatureParamType, FeatureObserveType>::Information() const {
    // for (const auto &frame : frames_) {
    //     frame.Information();
    // }
    for (const auto &item : features_) {
        item.second.Information();
    }
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::SelfCheck() {
    // Traversal all features, check if their observal frame is exist.
    for (auto &item : features_) {
        auto &feature = item.second;
        if (feature.observes().empty()) {
            continue;
        }

        if (frame(feature.first_frame_id()) == nullptr) {
            ReportError("[Covisible Graph] The first frame observing this feature does not exist.");
            return false;
        }
        if (frame(feature.final_frame_id()) == nullptr) {
            ReportError("[Covisible Graph] The last frame observing this feature does not exist.");
            return false;
        }
    }

    // Traversal all frames, check if their observal feature is exist.
    for (auto &frame : frames_) {
        for (const auto &item : frame.features()) {
            if (feature(item.first) == nullptr) {
                ReportError("[Covisible Graph] Feature observed by frame does not exist.");
                return false;
            }
        }
    }

    // Traversal all frames, check if their ids increasing.
    uint32_t prev_id = 0;
    for (auto it = frames_.begin(); it != frames_.end(); ++it) {
        if (prev_id < it->id()) {
            prev_id = it->id();
        } else {
            ReportError("[Covisible Graph] Frame id does not always increase.");
            return false;
        }
    }

    // Traversal all frames, check function frame(int32_t) is correct.
    for (auto &item: frames_) {
        auto frame_ptr = frame(item.id());
        if (frame_ptr == nullptr) {
            return false;
        }
        if (frame_ptr->id() != item.id()) {
            return false;
        }
    }

    // Traversal all features, check function feature(int32_t) is correct.
    for (auto &item: features_) {
        if (item.first != item.second.id()) {
            return false;
        }

        auto feature_ptr = feature(item.second.id());
        if (feature_ptr == nullptr) {
            return false;
        }
        if (feature_ptr->id() != item.second.id()) {
            return false;
        }
    }

    return true;
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::AddNewFrameWithFeatures(const std::vector<uint32_t> &features_id,
                                                                                   const std::vector<FeatureObserveType> &features_observe,
                                                                                   const float time_stamp_s,
                                                                                   const int32_t new_frame_id,
                                                                                   const std::vector<MatImg> &raw_images) {
    if (features_id.size() != features_observe.size()) {
        return false;
    }

    // If new frame id is not appointed, use the next frame id.
    int32_t new_frame_id_ = new_frame_id;
    if (new_frame_id_ < 0) {
        new_frame_id_ = frames_.empty() ? 1 : frames_.back().id() + 1;
    }

    // Create new frame, add it at the back of frame list.
    CovisibleGraph<FeatureParamType, FeatureObserveType>::FrameType new_frame(new_frame_id_);
    frames_.emplace_back(new_frame);
    frames_.back().time_stamp_s() = time_stamp_s;

    // Add raw images.
    frames_.back().raw_images() = raw_images;

    // Add new features or new observations for exist features.
    const int32_t max_size = features_id.size();
    int32_t num_of_discarded_observe = 0;
    for (int32_t i = 0; i < max_size; ++i) {
        auto &id = features_id[i];
        auto &obv = features_observe[i];

        auto feature_ptr = feature(id);
        if (feature_ptr == nullptr) {
            // Create new feature and insert it into covisible graph.
            CovisibleGraph<FeatureParamType, FeatureObserveType>::FeatureType new_feature(id);
            auto iter = features_.insert(std::make_pair(id, new_feature));
            if (!iter.second) {
                return false;
            }
            // Add the first observe.
            iter.first->second.observes().emplace_back(obv);
            iter.first->second.first_frame_id() = new_frame_id_;
            // Get the pointer of this new feature.
            feature_ptr = &(iter.first->second);
            // Add this new feature into new frame.
            frames_.back().features().insert(std::make_pair(id, feature_ptr));
        } else if (static_cast<int32_t>(feature_ptr->observes().size() + feature_ptr->first_frame_id()) == new_frame_id_) {
            // Add new observation for exist feature.
            feature_ptr->observes().emplace_back(obv);
            // Add this new feature into new frame.
            frames_.back().features().insert(std::make_pair(id, feature_ptr));
        } else {
            ++num_of_discarded_observe;
        }
    }

    if (num_of_discarded_observe) {
        ReportWarn("[CovisibleGraph] [" << num_of_discarded_observe << "] invalid observation is discarded.");
    }

    return true;
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::AddNewFrameWithFeatures(const std::vector<uint32_t> &features_id,
                                                                                   const std::vector<FeatureObserveType> &features_observe,
                                                                                   const std::vector<FeatureParamType> &features_param,
                                                                                   const float time_stamp_s,
                                                                                   const Quat &q_wc,
                                                                                   const Vec3 &p_wc,
                                                                                   const std::vector<MatImg> &raw_images) {
    RETURN_FALSE_IF_FALSE(AddNewFrameWithFeatures(features_id, features_observe, time_stamp_s, -1, raw_images));

    // Set new features' pose param.
    frames_.back().q_wc() = q_wc;
    frames_.back().p_wc() = p_wc;

    // Set features param.
    if (features_id.size() == features_param.size()) {
        for (uint32_t i = 0; i < features_id.size(); ++i) {
            feature(features_id[i])->param() = features_param[i];
        }
    }

    return true;
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::RemoveFeature(uint32_t feature_id) {
    auto feature_ptr = feature(feature_id);
    if (feature_ptr == nullptr || features_.empty()) {
        return false;
    }

    // If the feature is empty, directly remove it.
    if (feature_ptr->observes().empty()) {
        features_.erase(feature_id);
        return true;
    }

    // Traversal all frames, remove their correspondence with the feature to be removed.
    for (auto &frame : frames_) {
        frame.features().erase(feature_id);
        if (frame.features().empty()) {
            frame.need_remove() = true;
        }
    }

    // Remove the frames that loss all features.
    RemoveFrameNeedRemove();

    // Remove this feature from covisible graph.
    features_.erase(feature_id);

    return true;
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::RemoveFrame(uint32_t frame_id) {
    auto frame_ptr = frame(frame_id);
    if (frame_ptr == nullptr || frames_.empty()) {
        return false;
    }

    if (frame_id < frames_.front().id() || frame_id > frames_.back().id()) {
        return false;
    }

    // Case 1: feature.final_frame_id() < frame_id
    // [1, 2, 3, 4, 5, [6]] - [3, 4, 5]   -->> [1, 2, 3, 4, 5] - [3, 4, 5]                         - first_frame_id

    // Case 2: feature.first_frame_id() <= frame_id
    // [1, 2, 3, 4, [5], 6] - [3, 4, [5]] -->> [1, 2, 3, 4, 5(6)] - [3, 4]                         - first_frame_id
    // [1, 2, 3, [4], 5, 6] - [3, [4], 5] -->> [1, 2, 3, 4(5), 5(6)] - [3, 4(5)]                   - first_frame_id
    // [1, 2, [3], 4, 5, 6] - [[3], 4, 5] -->> [1, 2, 3(4), 4(5), 5(6)] - [3(4), 4(5)]             - first_frame_id
    // [[1], 2, 3, 4, 5, 6] - [[1], 2, 3] -->> [1(2), 2(3), 3(4), 4(5), 5(6)] - [1(2), 2(3)]       - first_frame_id

    // Case 3: frame_id < feature.first_frame_id()
    // [1, [2], 3, 4, 5, 6] - [3, 4, 5]   -->> [1, 2(3), 3(4), 4(5), 5(6)] - [2(3), 3(4), 4(5)]    - first_frame_id - 1
    // [[1], 2, 3, 4, 5, 6] - [3, 4, 5]   -->> [1(2), 2(3), 3(4), 4(5), 5(6)] - [2(3), 3(4), 4(5)] - first_frame_id - 1

    // Traversal all features, remove their correspondence with the frame to be removed.
    std::vector<uint32_t> empty_features_id;
    for (auto &item : features_) {
        auto &feature = item.second;

        if (feature.final_frame_id() < frame_id) {
            // Case 1.
            continue;
        } else if (feature.first_frame_id() <= frame_id) {
            // Case 2.
            const uint32_t start_idx = frame_id - feature.first_frame_id();
            const uint32_t end_idx = feature.observes().size() - 1;
            for (uint32_t i = start_idx; i < end_idx; ++i) {
                feature.observes()[i] = feature.observes()[i + 1];
            }
            feature.observes().resize(end_idx);
        } else {
            // Case 3.
            --feature.first_frame_id();
        }

        // If all observation in this feature is removed, remove this feature.
        if (feature.observes().empty()) {
            empty_features_id.emplace_back(feature.id());
        }
    }

    // Exchange id of frames that behind of removed frame.
    auto frame_to_be_removed = frames_.begin();
    for (auto it = frames_.begin(); it != frames_.end(); ++it) {
        if (it->id() == frame_id) {
            frame_to_be_removed = it;
        } else if (it->id() > frame_id) {
            --it->id();
        }
    }

    // Remove frame.
    frames_.erase(frame_to_be_removed);

    // Remove empty features.
    for (auto &id : empty_features_id) {
        features_.erase(id);
    }

    return true;
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::GetCovisibleFeatures(const uint32_t frame_i_id,
                                                                                const uint32_t frame_j_id,
                                                                                std::vector<FeatureType *> &covisible_features) {
    auto frame_i_ptr = frame(frame_i_id);
    auto frame_j_ptr = frame(frame_j_id);
    if (frame_i_ptr == nullptr || frame_j_ptr == nullptr) {
        return false;
    }

    return GetCovisibleFeatures(*frame_i_ptr, *frame_j_ptr, covisible_features);
}

template <typename FeatureParamType, typename FeatureObserveType>
bool CovisibleGraph<FeatureParamType, FeatureObserveType>::GetCovisibleFeatures(const FrameType &frame_i,
                                                                                const FrameType &frame_j,
                                                                                std::vector<FeatureType *> &covisible_features) {
    const uint32_t max_size = std::min(frame_i.features().size(), frame_j.features().size());
    if (max_size == 0) {
        return false;
    }

    if (covisible_features.capacity() < max_size) {
        covisible_features.reserve(max_size);
    }
    covisible_features.clear();

    for (const auto &item : frame_i.features()) {
        if (frame_j.features().find(item.first) != frame_j.features().end()) {
            covisible_features.emplace_back(item.second);
        }
    }

    return true;
}

// Find frame or feature by id.
template <typename FeatureParamType, typename FeatureObserveType>
typename CovisibleGraph<FeatureParamType, FeatureObserveType>::FrameType *CovisibleGraph<FeatureParamType, FeatureObserveType>::frame(uint32_t id) {
    CovisibleGraph<FeatureParamType, FeatureObserveType>::FrameType temp_frame;
    temp_frame.id() = id;

    auto iter = std::lower_bound(frames_.begin(), frames_.end(), temp_frame, [&] (
        CovisibleGraph<FeatureParamType, FeatureObserveType>::FrameType frame_i,
        CovisibleGraph<FeatureParamType, FeatureObserveType>::FrameType frame_j) {
        return frame_i.id() < frame_j.id();
    });

    if (iter != frames_.end() && iter->id() == id) {
        return &(*iter);
    }

    return nullptr;
}

template <typename FeatureParamType, typename FeatureObserveType>
typename CovisibleGraph<FeatureParamType, FeatureObserveType>::FeatureType *CovisibleGraph<FeatureParamType, FeatureObserveType>::feature(uint32_t id) {
    auto iter = features_.find(id);

    if (iter != features_.end()) {
        return &(iter->second);
    }

    return nullptr;
}

// Remove frame in std::list by check 'need_remove' flag.
template <typename FeatureParamType, typename FeatureObserveType>
void CovisibleGraph<FeatureParamType, FeatureObserveType>::RemoveFrameNeedRemove() {
    for (auto it = frames_.begin(); it != frames_.end(); ) {
        if (it->need_remove()) {
            RemoveFrame(it->id());
            it = frames_.begin();
        } else {
            ++it;
        }
    }
}

}

#endif // end of _SLAM_UTILITY_COVISIBLE_GRAPH_H_
