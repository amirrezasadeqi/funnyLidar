#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/init.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/SetCameraInfoResponse.h>

ros::Publisher clustered_data_pub;

void process_pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // convert ros message to pcl cloud
  pcl_conversions::toPCL(*msg, *cloud);
  // downsample data
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid_filter;
  pcl::PCLPointCloud2 *voxeled_cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr voxeled_cloudPtr(voxeled_cloud);
  voxel_grid_filter.setInputCloud(cloudPtr);
  voxel_grid_filter.setLeafSize(0.1, 0.1, 0.1);
  voxel_grid_filter.filter(*voxeled_cloud);
  // crop the ROI to reduce
  pcl::PassThrough<pcl::PCLPointCloud2> cropper_filter;
  pcl::PCLPointCloud2 *cropped_cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2ConstPtr cropped_cloudPtr(cropped_cloud);
  cropper_filter.setInputCloud(voxeled_cloudPtr);
  // cropper_filter.setFilterFieldName("x");
  cropper_filter.setFilterFieldName("hassan");
  cropper_filter.setFilterLimits(0.0, 30.0);
  cropper_filter.filter(*cropped_cloud);
  // convert cropped_cloud type into pcl::PointXYZ to be usable in
  // SACSegmentation.
  pcl::PointCloud<pcl::PointXYZI>::Ptr floor_segmentor_inputPtr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(*cropped_cloud, *floor_segmentor_inputPtr);

  // detect floor plane
  // create variables for segmentor output
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // create and setup the ransac plane segmentor
  pcl::SACSegmentation<pcl::PointXYZI> floor_segmentor;
  floor_segmentor.setOptimizeCoefficients(true);
  floor_segmentor.setModelType(pcl::SACMODEL_PLANE);
  floor_segmentor.setMethodType(pcl::SAC_RANSAC);
  floor_segmentor.setDistanceThreshold(0.2);
  floor_segmentor.setInputCloud(floor_segmentor_inputPtr);
  floor_segmentor.segment(*inliers, *coefficients);
  // extract out points in the inliers
  pcl::ExtractIndices<pcl::PointXYZI> floor_extractor;
  floor_extractor.setInputCloud(floor_segmentor_inputPtr);
  floor_extractor.setIndices(inliers);
  floor_extractor.setNegative(true);
  pcl::PointCloud<pcl::PointXYZI>::Ptr floor_segmented_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  floor_extractor.filter(*floor_segmented_cloud);

  // Euclidian Clustering
  // kdtree data structure which is useful in Clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(floor_segmented_cloud);
  // verctor of vector of cluster indices
  std::vector<pcl::PointIndices> cluster_indices;
  // setup euclidian clustering object
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidian_clusterer;
  euclidian_clusterer.setInputCloud(floor_segmented_cloud);
  euclidian_clusterer.setClusterTolerance(0.25);
  euclidian_clusterer.setMinClusterSize(50);
  euclidian_clusterer.setMaxClusterSize(25000);
  euclidian_clusterer.setSearchMethod(tree);
  euclidian_clusterer.extract(cluster_indices);

  std::cout << "Number of clusters in this frame is: " << cluster_indices.size()
            << std::endl;
  std::cout << "Number of Points in the floor_segmented_cloud is: "
            << floor_segmented_cloud->size() << std::endl;
  // loop through the vectors of indeces
  int j = 0;
  for (const auto &cluster : cluster_indices) {
    // dummy container to hold each cluster points
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloudPtr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*floor_segmented_cloud, cluster.indices,
                        *cluster_cloudPtr);

    // Algorithm to detect probable interesting objects
    // 1.find the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloudPtr, centroid);
    std::cout << "The centroid of cluster " << j << " is at " << std::endl
              << centroid << std::endl;
    // 2.using kdtree search the specified radius(expected radius of the object)
    // around the centroid
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cluster_cloudPtr);
    pcl::PointXYZI searchPoint;
    searchPoint.x = centroid[0];
    searchPoint.y = centroid[1];
    searchPoint.z = centroid[2];

    std::vector<int> inRadiusPointIdx;
    std::vector<float> inRadiusPointSquareDist;

    // the expected radius of interesting object
    float radius = 0.5f;

    if (kdtree.radiusSearch(searchPoint, radius, inRadiusPointIdx,
                            inRadiusPointSquareDist) > 0) {
      // 3.compare number of points in the specified radius
      // with number of points in the cluster. If most of the points are within
      // that radius, we can conclude that the cluster is the probable
      // interesting object.
      /* if (inRadiusPointIdx.size() > 0.8 * cluster_cloudPtr->size() &&
          cluster_cloudPtr->size() < 70) { */
      if (inRadiusPointIdx.size() > 0.8 * cluster_cloudPtr->size()) {
        std::cout << "Cluster " << j << " is probably interesting object."
                  << std::endl;
        // now publish the probable point cloud
        pcl::PCLPointCloud2::Ptr cluster_cloud_pc2Ptr(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cluster_cloudPtr, *cluster_cloud_pc2Ptr);
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(*cluster_cloud_pc2Ptr, output);
        clustered_data_pub.publish(output);
      }
    }
    std::cout << "Cluster " << j << " has " << cluster_cloudPtr->size()
              << " points inside." << std::endl;

    j++;
  }

  // convert back to pclopintcloud2 to continue
  /* pcl::PCLPointCloud2::Ptr floor_segmented_cloud_pc2Ptr(
      new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*floor_segmented_cloud, *floor_segmented_cloud_pc2Ptr);

  // convert to sensor_msgs::PointCloud2 to publish
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*floor_segmented_cloud_pc2Ptr, output);
  clustered_data_pub.publish(output); */
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cluster_avia_data");
  ros::NodeHandle nh;

  ros::Subscriber avia_sub;
  avia_sub = nh.subscribe("/livox/lidar", 10, process_pcl_callback);

  clustered_data_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/livox/clustered_data", 1);

  ros::spin();
}
