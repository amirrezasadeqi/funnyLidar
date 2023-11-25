//
// Created by areza on 8/16/23.
//

#include "single_object_tracker.h"
#include <vector>

single_object_tracker::single_object_tracker(unsigned int trackState, unsigned int trackAge,
                                             unsigned int trackLastUpdate, unsigned int trackLifeThreshold,
                                             unsigned int trackDistThreshold, const std::vector<float> &trackPose)
        : _track_state(trackState), _track_age(trackAge), _track_last_update(trackLastUpdate),
          _track_life_threshold(trackLifeThreshold), _track_dist_threshold(trackDistThreshold), _track_pose(trackPose) {}
