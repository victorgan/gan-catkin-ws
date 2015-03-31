#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

ros::Publisher pub_pointcloud;
ros::Publisher pub_coefficients;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true); // Optional
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  int max_iterations = 5000; // 1000 is okay; 10000 is stable but slow
  seg.setMaxIterations (max_iterations);
  seg.setDistanceThreshold (0.01);

  // Do the segmentation
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, coefficients); 

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub_coefficients.publish (ros_coefficients);

  // Start to extract indicies from inliers
  
  // Create extraction object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud.makeShared());
  extract.setIndices (inliers);

  // Positive
  pcl::PointCloud<pcl::PointXYZ> cloud_p; 
  extract.setNegative (false);
  extract.filter (cloud_p);

  // pcl::PointCloud<pcl::PointXYZ> cloud_n; // negative
  // extract.setNegative (true);
  // extract.filter (cloud_n);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud_p, output);

  // Publish the data
  pub_pointcloud.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planarsegmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the ground plane pointcloud 
  pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Create a ROS publisher for the output model coefficients
  pub_coefficients = nh.advertise<pcl_msgs::ModelCoefficients> ("output2", 1);

  // Spin
  ros::spin ();
}
