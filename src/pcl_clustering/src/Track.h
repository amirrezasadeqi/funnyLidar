//
// Created by areza on 11/25/23.
//

#ifndef SRC_TRACK_H
#define SRC_TRACK_H

#include <vector>
#include <Eigen/Core>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "kalmanFilter.h"

enum TrackState {
    INITIALIZED,
    UNINITIALIZED
};

class Track {
public:
    explicit Track(KalmanFilter &filter);

    static MatrixXf getProcessModel(double time);

    void initializeTrack(const std::vector<float> &trackPose);

    std::vector<float> getTrackPose();

    void setTrackPose(const std::vector<float> &trackPose);

    void setAssociationDistThreshold(float associationDistThreshold);

    TrackState getTrackState();

    void setTrackState(TrackState trackState);

    void setFilter(const KalmanFilter &filter);

    long associateClusterToTrack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                 const std::vector<pcl::PointIndices> &clusters);

    void updateTrack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                     const std::vector<pcl::PointIndices> &clusters, long associatedClusterIndex, double deltaTime);

    bool isTrackReliable();

private:
    TrackState _trackState;
    // distance threshold for cluster track association
    float _associationDistThreshold;
    std::vector<float> _trackPose;
    KalmanFilter _filter;

};


#endif //SRC_TRACK_H
