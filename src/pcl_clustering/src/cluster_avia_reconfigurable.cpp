#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cstdint>
#include <cstdlib>
#include <dynamic_reconfigure/server.h>
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
#include <pcl_clustering/pcl_clusteringConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/init.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/SetCameraInfoResponse.h>

ros::Publisher cluster_colored_pub;
ros::Publisher voxeled_pub;
ros::Publisher cropped_pub;
ros::Publisher floor_segmented_pub;

// global vars for getting dynamic parameter values
bool show_voxeled_output, show_cropped_output, show_floor_segmented_output,
    enable_downsampling;
float voxel_size, x_crop_min, x_crop_max, floor_segmentor_dist_threshold,
    cluster_threshold, kdtree_search_radius, intersting_object_inlier_percent;
int min_cluster_points, max_cluster_points, max_points_interesting_obj;

void dynamic_param_callback(pcl_clustering::pcl_clusteringConfig &config,
                            uint32_t level) {
  enable_downsampling = config.enable_downsampling;
  show_voxeled_output = config.show_voxeled_output;
  show_cropped_output = config.show_cropped_output;
  show_floor_segmented_output = config.show_floor_segmented_output;
  voxel_size = config.voxel_size;
  x_crop_min = config.x_crop_min;
  x_crop_max = config.x_crop_max;
  floor_segmentor_dist_threshold = config.floor_segmentor_dist_threshold;
  cluster_threshold = config.cluster_threshold;
  kdtree_search_radius = config.kdtree_search_radius;
  intersting_object_inlier_percent = config.intersting_object_inlier_percent;
  min_cluster_points = config.min_cluster_points;
  max_cluster_points = config.max_cluster_points;
  max_points_interesting_obj = config.max_points_interesting_obj;
}

void intersting_clusters_colored(
    pcl::PointCloud<pcl::PointXYZRGB> &colored_output_pc,
    const pcl::PointCloud<pcl::PointXYZI> &input_pc,
    const std::vector<pcl::PointIndices> &vec_interesting_indices) {
  std::cout << "Hello world!" << std::endl;
  // loop through vector of clusters
  for (const auto &cluster : vec_interesting_indices) {
    //   create a random color for the cluster
    std::vector<unsigned char> color;
    for (char i = 0; i < 3; i++)
      color.push_back((unsigned char)(rand() % 256));
    //   loop through each point of the cluster/indices of input_pc
    for (std::vector<int>::const_iterator pit = cluster.indices.begin();
         pit != cluster.indices.end(); pit++) {
      // create a dummy point to be pushed back into output pc
      pcl::PointXYZRGB dummy_point;
      // assign the RGB fields of the output to the cluster color
      dummy_point.r = color[0];
      dummy_point.g = color[1];
      dummy_point.b = color[2];
      // assign the point coordinates to the output pc
      dummy_point.x = input_pc.points[*pit].x;
      dummy_point.y = input_pc.points[*pit].y;
      dummy_point.z = input_pc.points[*pit].z;
      // push back the point into the output pc
      colored_output_pc.push_back(dummy_point);
    }
  }
}

void process_pcl_callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // convert ros message to pcl cloud
  pcl_conversions::toPCL(*msg, *cloud);
  // downsample data
  pcl::PCLPointCloud2 *voxeled_cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr voxeled_cloudPtr(voxeled_cloud);
  if (enable_downsampling) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloudPtr);
    voxel_grid_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid_filter.filter(*voxeled_cloud);
    // publish voxel output
    if (show_voxeled_output)
      voxeled_pub.publish(*voxeled_cloud);
  } else {
    pcl::copyPointCloud(*cloud, *voxeled_cloud);
  }

  // crop the ROI to reduce
  pcl::PassThrough<pcl::PCLPointCloud2> cropper_filter;
  pcl::PCLPointCloud2 *cropped_cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2ConstPtr cropped_cloudPtr(cropped_cloud);
  cropper_filter.setInputCloud(voxeled_cloudPtr);
  cropper_filter.setFilterFieldName("x");
  cropper_filter.setFilterLimits(x_crop_min, x_crop_max);
  cropper_filter.filter(*cropped_cloud);
  // convert cropped_cloud type into pcl::PointXYZ to be usable in
  // SACSegmentation.
  pcl::PointCloud<pcl::PointXYZI>::Ptr floor_segmentor_inputPtr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(*cropped_cloud, *floor_segmentor_inputPtr);
  // publish crop output
  if (show_cropped_output)
    cropped_pub.publish(*cropped_cloud);

  // detect floor plane
  // create variables for segmentor output
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // create and setup the ransac plane segmentor
  pcl::SACSegmentation<pcl::PointXYZI> floor_segmentor;
  floor_segmentor.setOptimizeCoefficients(true);
  floor_segmentor.setModelType(pcl::SACMODEL_PLANE);
  floor_segmentor.setMethodType(pcl::SAC_RANSAC);
  floor_segmentor.setDistanceThreshold(floor_segmentor_dist_threshold);
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
  // publish the floor segmented cloud
  if (show_floor_segmented_output) {
    sensor_msgs::PointCloud2 floor_segmented_msg;
    pcl::toROSMsg(*floor_segmented_cloud, floor_segmented_msg);
    floor_segmented_pub.publish(floor_segmented_msg);
  }

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
  euclidian_clusterer.setClusterTolerance(cluster_threshold);
  euclidian_clusterer.setMinClusterSize(min_cluster_points);
  euclidian_clusterer.setMaxClusterSize(max_cluster_points);
  euclidian_clusterer.setSearchMethod(tree);
  euclidian_clusterer.extract(cluster_indices);

  std::cout << "Number of clusters in this frame is: " << cluster_indices.size()
            << std::endl;
  std::cout << "Number of Points in the floor_segmented_cloud is: "
            << floor_segmented_cloud->size() << std::endl;
  // loop through the vectors of indeces
  int j = 0;
  // a vector with list of interesting clusters's indices as each element
  std::vector<pcl::PointIndices> interesting_cluster_indices;

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
    float radius = kdtree_search_radius;

    if (kdtree.radiusSearch(searchPoint, radius, inRadiusPointIdx,
                            inRadiusPointSquareDist) > 0) {
      // 3.compare number of points in the specified radius
      // with number of points in the cluster. If most of the points are within
      // that radius, we can conclude that the cluster is the probable
      // interesting object.
      if (inRadiusPointIdx.size() >
              intersting_object_inlier_percent * cluster_cloudPtr->size() &&
          cluster_cloudPtr->size() < max_points_interesting_obj) {
        std::cout << "Cluster " << j << " is probably interesting object."
                  << std::endl;
        // append cluster to interesting clusters
        interesting_cluster_indices.push_back(cluster);
      }
    }
    std::cout << "Cluster " << j << " has " << cluster_cloudPtr->size()
              << " points inside." << std::endl;

    j++;
  }
  // colored point cloud for visualization of interesting objects
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_output_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  intersting_clusters_colored(*colored_output_pc, *floor_segmented_cloud,
                              interesting_cluster_indices);
  pcl::PCLPointCloud2::Ptr colored_output_pc2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*colored_output_pc, *colored_output_pc2);
  sensor_msgs::PointCloud2 colorpcl2_msg;
  colored_output_pc2->header = floor_segmented_cloud->header;
  pcl_conversions::fromPCL(*colored_output_pc2, colorpcl2_msg);
  cluster_colored_pub.publish(colorpcl2_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cluster_avia_data");
  ros::NodeHandle nh;

  // dynamic reconfigurable servers and things
  dynamic_reconfigure::Server<pcl_clustering::pcl_clusteringConfig>
      param_server;
  dynamic_reconfigure::Server<
      pcl_clustering::pcl_clusteringConfig>::CallbackType f;
  f = boost::bind(&dynamic_param_callback, _1, _2);
  param_server.setCallback(f);

  ros::Subscriber avia_sub;
  avia_sub = nh.subscribe("/livox/lidar", 10, process_pcl_callback);

  cluster_colored_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/livox/cluster_colored_data", 1);
  voxeled_pub = nh.advertise<pcl::PCLPointCloud2>("/livox/voxeled_output", 1);
  cropped_pub = nh.advertise<pcl::PCLPointCloud2>("/livox/cropped_output", 1);
  floor_segmented_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox/floor_segmented_output", 1);

  ros::spin();
}
