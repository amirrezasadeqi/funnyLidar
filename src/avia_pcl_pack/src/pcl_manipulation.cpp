#include "pcl/PCLPointCloud2.h"
#include <ros/ros.h>
// PCL specific includes
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher pub;

void cloud_voxel_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // create the cloud containers
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  // convert ros pcl2 msg to pcl data
  pcl_conversions::toPCL(*input, *cloud);
  // filter the data using voxel grid
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.5, 0.5, 0.5);
  sor.filter(cloud_filtered);
  // convert back the pcl data to ros msg
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
  // publish ros msg
  pub.publish(output);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish(output);
}

pcl::PCLPointCloud2
filter_coordinate(const pcl::PCLPointCloud2ConstPtr &cloudPtr,
                  const std::string &coordinate, const double &min,
                  const double &max) {
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(cloudPtr);
  pass.setFilterFieldName(coordinate);
  pass.setFilterLimits(min, max);
  pcl::PCLPointCloud2 output;
  pass.filter(output);
  return output;
}

void cloud_passthrough_filter_cb(
    const sensor_msgs::PointCloudConstPtr &inputi) {
  sensor_msgs::PointCloud2 *input = new sensor_msgs::PointCloud2;
  sensor_msgs::convertPointCloudToPointCloud2(*inputi, *input);
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl_conversions::toPCL(*input, *cloud);

  pcl::PCLPointCloud2 output = filter_coordinate(cloudPtr, "y", -1.0, 0.0);

  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(output, msg);
  pub.publish(msg);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloud_voxel_cb);
  /* ros::Subscriber sub =
      nh.subscribe("/livox/lidar", 1, cloud_passthrough_filter_cb); */
  ros::Subscriber sub = nh.subscribe("/scan", 1, cloud_passthrough_filter_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
