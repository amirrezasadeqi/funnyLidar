//
// Created by areza on 8/16/23.
//

#ifndef SRC_SINGLE_OBJECT_TRACKER_H
#define SRC_SINGLE_OBJECT_TRACKER_H

#include <vector>

class single_object_tracker {
public:
    single_object_tracker(unsigned int trackState, unsigned int trackAge, unsigned int trackLastUpdate,
                          unsigned int trackLifeThreshold, unsigned int trackDistThreshold,
                          const std::vector<float> &trackPose);

private:
    unsigned int _track_state;
    unsigned int _track_age;
    unsigned int _track_last_update;
    unsigned int _track_life_threshold;
    unsigned int _track_dist_threshold;
    std::vector<float> _track_pose[3];
};


#endif //SRC_SINGLE_OBJECT_TRACKER_H
