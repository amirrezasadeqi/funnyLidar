//
// Created by areza on 11/25/23.
//

#include "Track.h"
#include "kalmanFilter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <cmath>

Track::Track(KalmanFilter &filter) : _trackState(UNINITIALIZED), _associationDistThreshold(1.0f),
                                     _trackPose({0.0f, 0.0f, 0.0f}), _filter(filter) {}

void Track::initializeTrack(const std::vector<float> &trackPose) {
    _trackState = INITIALIZED;
    _trackPose = trackPose;
    // Reinitializing the kalman filter fields.
    VectorXf state(6); // x, vx, y, vy, z, vz
    state << trackPose[0], 0.0f, trackPose[1], 0.0f, trackPose[2], 0.0f;
    _filter.setState(state);
    MatrixXf covariance(6, 6);
    covariance << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 16.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 16.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 16.0f;
    _filter.setCovariance(covariance);
}

std::vector<float> Track::getTrackPose() {
    return _trackPose;
}

TrackState Track::getTrackState() {
    return _trackState;
}

void Track::setTrackState(TrackState trackState) {
    _trackState = trackState;
}

long
Track::associateClusterToTrack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               const std::vector<pcl::PointIndices> &clusters) {
    std::vector<float> clustersDistancesToTrack;
    for (const auto &cluster: clusters) {
        // determine cluster distance to track and add it to distances vector
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, cluster, centroid);
        float distance = std::sqrt(std::pow((centroid.x() - _trackPose[0]), 2.0f) +
                                   std::pow((centroid.y() - _trackPose[1]), 2.0f) +
                                   std::pow((centroid.z() - _trackPose[2]), 2.0f));
        clustersDistancesToTrack.push_back(distance);
    }
    // determine index of cluster with minimum distance
    long minDistanceIndex = std::distance(std::begin(clustersDistancesToTrack),
                                          std::min_element(std::begin(clustersDistancesToTrack),
                                                           std::end(clustersDistancesToTrack)));
    // if the distance is less than threshold return cluster index
    if (clustersDistancesToTrack[minDistanceIndex] <= _associationDistThreshold) {
        return minDistanceIndex;
    } else {
        // else there is no associated cluster for track so return -1
        return -1;
    }
}

void
Track::updateTrack(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &clusters,
                   long associatedClusterIndex, double deltaTime) {
    if (-1 == associatedClusterIndex) {
        // Use the prediction step of Kalman Filter here to update the position (there is no association, so we can't
        // correct the prediction.)

        // Determining the process model matrix for this time step.
        _filter.setProcessModel(getProcessModel(deltaTime));
        _filter.prediction();
    } else {
        // Predict based on the process model
        // Determining the process model matrix for this time step.
        _filter.setProcessModel(getProcessModel(deltaTime));
        _filter.prediction();
        // Correct the prediction based on the associated cluster position
        Eigen::Vector4f centroid;
        const pcl::PointIndices &cluster = clusters[associatedClusterIndex];
        pcl::compute3DCentroid(*cloud, cluster, centroid);
        VectorXf measurement(3);
        measurement << centroid.x(), centroid.y(), centroid.z();
        _filter.setMeasurement(measurement);
        _filter.update();
    }
    std::vector<float> trackPose{_filter.getState()(0), _filter.getState()(2), _filter.getState()(4)};
    setTrackPose(trackPose);
}

bool Track::isTrackReliable() {
    // TODO: Determine if the track is reliable based on the covariance of the filter states.
    return true;
}

MatrixXf Track::getProcessModel(double deltaTime) {
    MatrixXf processModel(6, 6);
    auto dt = (float) deltaTime;
    processModel << 1, dt, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, dt, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, dt,
            0, 0, 0, 0, 0, 1;
    return processModel;
}

void Track::setTrackPose(const std::vector<float> &trackPose) {
    _trackPose = trackPose;
}

void Track::setFilter(const KalmanFilter &filter) {
    _filter = KalmanFilter(filter);
}

void Track::setAssociationDistThreshold(float associationDistThreshold) {
    _associationDistThreshold = associationDistThreshold;
}
