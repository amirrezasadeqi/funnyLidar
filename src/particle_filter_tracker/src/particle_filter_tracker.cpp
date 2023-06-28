#include <boost/math/special_functions/math_fwd.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/tracking.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <visualization_msgs/Marker.h>

// create the global objects listed below:
// Particle filter tracker
pcl::tracking::ParticleFilterTracker<
    pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>::Ptr tracker_;
ros::Publisher ref_model_pub;
ros::Publisher marker_pub;

// create the callback function
//  {
//    --lock the global mutex if neccessary(if you use seperated threads
//    for visualization and tracking, you may need the mutex lock,
//    because even in tracker_ object are some points that you want to
//    visualize and in the same time they change in the tracker callback
//    which may be a source of race condition.). preprocess input cloud
//    if neccessary. give the preprocessed cloud as input to the tracker
//    object. call the compute method of the tracker object.
//  }
void pftracker_cb(const pcl::PCLPointCloud2ConstPtr &cloud) {

  // preprocess input cloud if neccessary
  pcl::PointCloud<pcl::PointXYZ>::Ptr tracker_in_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud, *tracker_in_cloud);
  // give the preprocessed cloud as input to the tracker object
  tracker_->setInputCloud(tracker_in_cloud);
  // call the compute method of the tracker object
  tracker_->compute();
  // Below code is for visualization, in future these lines may be moved into
  // another thread worker function.
  pcl::tracking::ParticleXYZRPY result = tracker_->getResult();
  Eigen::Affine3f transform = tracker_->toEigenMatrix(result);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_model_viz(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*(tracker_->getReferenceCloud()), *ref_model_viz,
                           transform);
  sensor_msgs::PointCloud2 ref_model_viz_rosmsg;
  pcl::toROSMsg(*ref_model_viz, ref_model_viz_rosmsg);
  ref_model_viz_rosmsg.header.stamp = ros::Time::now();
  ref_model_viz_rosmsg.header.frame_id = cloud->header.frame_id;
  ref_model_pub.publish(ref_model_viz_rosmsg);

  visualization_msgs::Marker marker;
  marker.header = ref_model_viz_rosmsg.header;
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.position.x = transform.translation().x();
  marker.pose.position.y = transform.translation().y();
  marker.pose.position.z = transform.translation().z();
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
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "particle_filter_tracker");
  ros::NodeHandle nh;

  // 1. Reading reference model PCD file
  pcl::PCDReader pcd_reader;
  std::string pcd_path = ros::package::getPath("particle_filter_tracker") +
                         std::string("/pcd_models/multirotor_model_2Hz.pcd");
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcd_reader.read(pcd_path, *ref_cloud);

  // 2. Setting up the covariance values used in particle filter tracker
  //    and other neccessary parameters
  // I think this is the model uncertanity
  std::vector<double> default_step_covariance =
      std::vector<double>(6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
  // I think these are the initial gusses for sensor noise covariance and
  // initial state of the system
  std::vector<double> initial_noise_covariance =
      std::vector<double>(6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

  // 3. create tracker object and setup its parameters

  // 8 is number of threads used for tracking
  pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<
      pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>::Ptr
      tracker(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<
              pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>(8));

  // I don't know what exactly this is, but from its name I think this is
  // related to feature histograms.
  pcl::tracking::ParticleXYZRPY bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;

  // setting up particle filter tracker's settings(I think these are the
  // matching parameters)
  tracker->setMaximumParticleNum(1000);
  tracker->setDelta(0.99);
  tracker->setEpsilon(0.2);
  tracker->setBinSize(bin_size);

  tracker_ = tracker;

  // set up configs of particle filter(I think these are parameters of
  // particle/bayes filter like the covariances)
  tracker_->setTrans(Eigen::Affine3f::Identity());
  tracker_->setStepNoiseCovariance(default_step_covariance);
  tracker_->setInitialNoiseCovariance(initial_noise_covariance);
  tracker_->setInitialNoiseMean(default_initial_mean);
  tracker_->setIterationNum(2);
  tracker_->setParticleNum(60);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal(
      false); // if true, coherence init fails, I don't why? check it later!

  // 4. setup the coherence object(used for matching reference and query) and
  //    add the matching conditions to the coherence object.
  pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr
      coherence(new pcl::tracking::ApproxNearestPairPointCloudCoherence<
                pcl::PointXYZ>);
  // 5. add coherence object to tracker
  pcl::tracking::DistanceCoherence<pcl::PointXYZ>::Ptr distance_coherence(
      new pcl::tracking::DistanceCoherence<pcl::PointXYZ>);
  coherence->addPointCoherence(distance_coherence);

  pcl::search::Octree<pcl::PointXYZ>::Ptr search(
      new pcl::search::Octree<pcl::PointXYZ>(
          0.01)); // 0.01 may be too small and cause overflow.
  coherence->setSearchMethod(search);
  coherence->setMaximumDistance(0.01);

  tracker_->setCloudCoherence(coherence);

  // 6. transform the reference model to the origin frame(I think canonical
  // coordinate system)
  Eigen::Vector4f center;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud_trasfered(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::compute3DCentroid(*ref_cloud, center);
  trans.translation().matrix() =
      Eigen::Vector3f(center[0], center[1], center[2]);
  pcl::transformPointCloud<pcl::PointXYZ>(*ref_cloud, *ref_cloud_trasfered,
                                          trans.inverse());
  // 7. add the prepared reference model and its transformation into origin to
  // the tracker object
  tracker_->setReferenceCloud(ref_cloud_trasfered);
  tracker_->setTrans(trans);

  ref_model_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/tracking/ref_model", 1);
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // 8. set up the callback function for doing the actual tracking
  ros::Subscriber lidar_sub = nh.subscribe("/livox/lidar", 5, pftracker_cb);
  ros::spin();
}
