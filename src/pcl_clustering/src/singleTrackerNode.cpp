//
// Created by areza on 11/25/23.
//
#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/Marker.h>
#include "Track.h"

using Eigen::VectorXf;
using Eigen::MatrixXf;

static KalmanFilter filter(6, 3);
static Track track(filter);
static ros::Publisher coloredClusterPublisher;
static double lastTime = -1.0; // time in seconds
static ros::Publisher marker_pub;
static ros::Publisher dist_label_pub;

void colorizeClusters(pcl::PointCloud<pcl::PointXYZRGB> &coloredOutputCloud,
                      const pcl::PointCloud<pcl::PointXYZ> &inputPointCloud,
                      const std::vector<pcl::PointIndices> &clustersIndices);


void removeDummyZeroPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &unNaNedCloud);

std::vector<pcl::PointIndices> clusterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

void visualizeTrack(const std::vector<float> &trackPose, const std_msgs::Header &header);

void
visualizeColoredClusters(const pcl::PCLHeader &inCloudHeader, const pcl::PointCloud<pcl::PointXYZ>::Ptr &unNaNedCloud,
                         const std::vector<pcl::PointIndices> &clusterIndices);



void processPclCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (lastTime < 0.0) return;
    double currentTime = msg->header.stamp.toSec();
    double deltaTime = currentTime - lastTime;
    lastTime = currentTime;
    if (UNINITIALIZED == track.getTrackState()) {
        ROS_INFO("No track is initialized! Please initialize a track in Rviz.");
    } else { // A track has been initialized.
        // Read and pre-process the message point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr unNaNedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::fromROSMsg(*msg, *inCloud);
        removeDummyZeroPoints(inCloud, unNaNedCloud);
        if (unNaNedCloud->width) {
            // Cluster pre-processed point cloud
            clusterIndices = clusterPointCloud(unNaNedCloud);

            // Visualize clusters for debug////////////////////////////////////////////////
            visualizeColoredClusters(inCloud->header, unNaNedCloud, clusterIndices);
            //end visualization//////////////////////////////////////////////

            // associate the cluster to the track using cluster features(here the centroid)
            long associatedClusterIndex = track.associateClusterToTrack(unNaNedCloud, clusterIndices);
            // update the track object
            track.updateTrack(unNaNedCloud, clusterIndices, associatedClusterIndex, deltaTime);
        } else { // when there is no unNaN point in point cloud
            track.updateTrack(unNaNedCloud, clusterIndices, -1, deltaTime);
        }
        if (track.isTrackReliable()) {
            visualizeTrack(track.getTrackPose(), msg->header);
        } else {
            track.setTrackState(UNINITIALIZED);
        }
    }
}

void processSelectedPoints(const sensor_msgs::PointCloud2ConstPtr &msg) {
    lastTime = msg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ>::Ptr selectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *selectedCloud);
    if (selectedCloud->width) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*selectedCloud, centroid);
        std::vector<float> trackPose{centroid.x(), centroid.y(), centroid.z()};
        // Reinitializing the track object and setting kalman filter inside it.
        track.initializeTrack(trackPose);

        // For debug purpose//////////////
        std::vector<float> hassan = track.getTrackPose();
        std::cout << "x: " << hassan[0] << " y: " << hassan[1] << " z: " << hassan[2] << std::endl;
        // For debug purpose//////////////////////////////
    } else {
        ROS_INFO("No Points are selected!");
        return;
    }
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "singleTrackerNode");
    ros::NodeHandle nh;
    // Set up the kalman filter object fields

    MatrixXf measurementModel(3, 6);
    measurementModel << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f;
    filter.setMeasurementModel(measurementModel);

    MatrixXf processNoise(6, 6); // TODO: This must be tuned.
    processNoise.setIdentity();
    filter.setProcessNoise(processNoise);

    MatrixXf measurementNoise(3, 3); // TODO: This must be tuned.
    measurementNoise.setIdentity();
    filter.setMeasurementNoise(measurementNoise);

    // Set up the track object fields here if you need. For example,
    track.setFilter(filter);
    track.setAssociationDistThreshold(10.0f);

    ros::Subscriber avia_sub = nh.subscribe("/livox/lidar", 2, processPclCallback);
    ros::Subscriber selectedPointsSub = nh.subscribe("/rviz_selected_points", 1, processSelectedPoints);

    coloredClusterPublisher = nh.advertise<sensor_msgs::PointCloud2>("/livox/cluster_colored_data", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/livox/track/visualization_marker", 1);
    dist_label_pub = nh.advertise<visualization_msgs::Marker>("/livox/track/dist_label", 1);

    ros::spin();
    return 0;
}

void colorizeClusters(pcl::PointCloud<pcl::PointXYZRGB> &coloredOutputCloud,
                      const pcl::PointCloud<pcl::PointXYZ> &inputPointCloud,
                      const std::vector<pcl::PointIndices> &clustersIndices) {
    // loop through vector of clusters
    for (const auto &cluster: clustersIndices) {

        // create a random color for the cluster
        std::vector<unsigned char> color;
        for (char i = 0; i < 3; i++)
            color.push_back((unsigned char) (rand() % 256));

        // loop through each point of the cluster of inputPointCloud
        for (std::vector<int>::const_iterator pit = cluster.indices.begin(); pit != cluster.indices.end(); pit++) {
            // create a dummy point to be pushed back into the output pc
            pcl::PointXYZRGB dummy_point;
            // assign the RGB fields of the output to the cluster color
            dummy_point.r = color[0];
            dummy_point.g = color[1];
            dummy_point.b = color[2];
            // assign the point coordinates to the output pc
            dummy_point.x = inputPointCloud.points[*pit].x;
            dummy_point.y = inputPointCloud.points[*pit].y;
            dummy_point.z = inputPointCloud.points[*pit].z;
            // push back the point into the output pc
            coloredOutputCloud.push_back(dummy_point);
        }
    }
}

void removeDummyZeroPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &inCloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &unNaNedCloud) {
    // Finding the neighbors of the origin and removing them.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inCloud);
    pcl::PointXYZ origin(0.0, 0.0, 0.0);
    float radius = 0.1f;
    std::vector<int> idxs;
    std::vector<float> dists;
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(inCloud);
    ei.setNegative(true);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    if (kdtree.radiusSearch(origin, radius, idxs, dists) > 0) {
        indices->indices = idxs;
        ei.setIndices(indices);
        ei.filter(*unNaNedCloud);
    }
}

std::vector<pcl::PointIndices> clusterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // kdtree data structure which is useful in Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    // vector of vectors of cluster indices
    std::vector<pcl::PointIndices> cluster_indices;
    // setup euclidian clustering object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecExtractor;
    ecExtractor.setInputCloud(cloud);
    ecExtractor.setClusterTolerance(3.0);
    ecExtractor.setMinClusterSize(1);
    ecExtractor.setMaxClusterSize(500);
    ecExtractor.setSearchMethod(tree);
    ecExtractor.extract(cluster_indices);

    return cluster_indices;
}

void visualizeTrack(const std::vector<float> &trackPose, const std_msgs::Header &header) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.position.x = trackPose[0];
    marker.pose.position.y = trackPose[1];
    marker.pose.position.z = trackPose[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    visualization_msgs::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "basic_shapes";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.scale.x = 3.0;
    text_marker.scale.y = 3.0;
    text_marker.scale.z = 3.0;
    text_marker.pose.position.x = trackPose[0] + 0.8;
    text_marker.pose.position.y = trackPose[1] + 0.8;
    text_marker.pose.position.z = trackPose[2] + 0.8;
    text_marker.pose.orientation.x = 0.0;
    text_marker.pose.orientation.y = 0.0;
    text_marker.pose.orientation.z = 0.0;
    text_marker.pose.orientation.w = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 0.5;
    text_marker.lifetime = ros::Duration();
    float dist;
    dist = std::sqrt((trackPose[0] * trackPose[0]) + (trackPose[1] * trackPose[1]) + (trackPose[2] * trackPose[2]));
    char label[20];
    sprintf(label, "Distance: %f", dist);
    text_marker.text = label;
    dist_label_pub.publish(text_marker);
}

void
visualizeColoredClusters(const pcl::PCLHeader &inCloudHeader, const pcl::PointCloud<pcl::PointXYZ>::Ptr &unNaNedCloud,
                         const std::vector<pcl::PointIndices> &clusterIndices) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredClusterPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    coloredClusterPointCloud->header = inCloudHeader;
    colorizeClusters(*coloredClusterPointCloud, *unNaNedCloud, clusterIndices);
    sensor_msgs::PointCloud2 coloredClusterMsg;
    pcl::toROSMsg(*coloredClusterPointCloud, coloredClusterMsg);
    coloredClusterPublisher.publish(coloredClusterMsg);
}
