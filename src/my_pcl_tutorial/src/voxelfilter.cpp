#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert to PCL data type
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Container for filtered data
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  char node_name[] = "voxelfilter";
  ros::init (argc, argv, node_name);
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  int queue_size = 1;
  char topic_inputcloud_name[] = "/camera/depth/points";
  ros::Subscriber sub = nh.subscribe (topic_inputcloud_name, queue_size, cloud_cb);

  // Create a ROS publisher for the output point cloud
  char topic_outputcloud_name[] = "voxelpoints";
  pub = nh.advertise<sensor_msgs::PointCloud2> (topic_outputcloud_name, queue_size);

  // Spin
  ros::spin ();
}
