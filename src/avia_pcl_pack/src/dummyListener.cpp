#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void dummyCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dummyListenerNode");
  ros::NodeHandle nh;

  ros::Subscriber dummyListener;
  dummyListener = nh.subscribe<sensor_msgs::PointCloud2>("/livox/voxelize", 10,
                                                         dummyCallback);

  ros::spin();
}
