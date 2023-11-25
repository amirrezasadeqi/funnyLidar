//
// Created by areza on 8/14/23.
//

#include <ros/ros.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/Marker.h>
#include <string>

ros::Publisher unNanPublisher;
static ros::Publisher marker_pub;
static ros::Publisher dist_label_pub;

void process_pcl_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    std::cout << "Number of Points in cloud is: " << cloud->width << std::endl;

    // A better way. finding the neighbours of the origin and removing them.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ origin(0.0, 0.0, 0.0);
    float radius = 0.1f;
    std::vector<int> idxs;
    std::vector<float> dists;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setNegative(true);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    if (kdtree.radiusSearch(origin, radius, idxs, dists) > 0) {
        std::cout << "Size of idxs: " << idxs.size() << std::endl;
        indices->indices = idxs;
        ei.setIndices(indices);
        ei.filter(*cloud_unan);
    }

    if(cloud_unan->width > 0){
        std::cout << "Number of Un-NaN points in the cloud is: " << cloud_unan->width << std::endl;
        sensor_msgs::PointCloud2 outmsg;
        pcl::toROSMsg(*cloud_unan, outmsg);
        unNanPublisher.publish(outmsg);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_unan, centroid);
        std::cout << "The center of the object is at: x= " << centroid.x() <<", y=" << centroid.y() <<", z=" << centroid.z() << std::endl;

        visualization_msgs::Marker marker;
        marker.header = outmsg.header;
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.pose.position.x = centroid.x();
        marker.pose.position.y = centroid.y();
        marker.pose.position.z = centroid.z();
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
        text_marker.header = outmsg.header;
        text_marker.ns = "basic_shapes";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.scale.x = 3.0;
        text_marker.scale.y = 3.0;
        text_marker.scale.z = 3.0;
        text_marker.pose.position.x = centroid.x() + 0.8;
        text_marker.pose.position.y = centroid.y() + 0.8;
        text_marker.pose.position.z = centroid.z() + 0.8;
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
        dist = std::sqrt((centroid.x() * centroid.x()) + (centroid.y() * centroid.y()) + (centroid.z() * centroid.z()));
        char label[20];
        sprintf(label, "Distance: %f", dist);
        text_marker.text = label;
        dist_label_pub.publish(text_marker);
    }



    // removing all the invalid points with x=y=z=0;
    // setup point cloud condition
/*
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr nans_condition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    nans_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::EQ, 0.0)));
    nans_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::EQ, 0.0)));
    nans_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::EQ, 0.0)));
    // set up the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> nans_removal;
    nans_removal.setCondition(nans_condition);
    nans_removal.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unan(new pcl::PointCloud<pcl::PointXYZ>);
    nans_removal.filter(*cloud_unan);
*/

}


int main(int argc, char** argv){

    ros::init(argc, argv, "cluster_avia_adaptive");
    ros::NodeHandle nh;
    ros::Subscriber avia_sub = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, process_pcl_cb);
    unNanPublisher = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_uNaned", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/livox/adaptive_cluster/visualization_marker", 1);
    dist_label_pub = nh.advertise<visualization_msgs::Marker>("/livox/adaptive_cluster/dist_label", 1);
    ros::spin();
    return 0;
}