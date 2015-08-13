#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
// Initializing the counters
int count_rgb = 1;
int count_depth = 1;
double spf;
//
  cv::Mat image1;
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);
//
// Function for converting the msg to jpg
void extraction(const sensor_msgs::ImageConstPtr& msg, int* count, ros::NodeHandle& node_handle)
{
    //ROS_WARN("Entering extraction.");
    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
    if (msg->encoding.find("bayer") != std::string::npos)
    {
      if (strcmp(msg->encoding.c_str(),"32FC1")==0)
      {
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      }else
    {
     boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";
    }
    }

    cv::Mat image;
    try
    {
    if (strcmp(msg->encoding.c_str(),"32FC1")==0)
      {
    //ROS_WARN("Sharing depth image.");
      image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
//
     depthToCV8UC1(image, image1);
    image = image1;
//
      }else
    {
    //ROS_WARN("Sharing rgb image.");
     image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    } catch(cv_bridge::Exception)
    {
     if (strcmp(msg->encoding.c_str(),"32FC1")==0)
      {
      ROS_ERROR("Unable to convert %s image to 32FC1", msg->encoding.c_str());
      }else
    {
     ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
    }

    }

    if (!image.empty()) {
    //ROS_WARN("Image converted.");
    std::string filename;
    std::stringstream sstm;
    std::ostringstream oss;
    std::string path1_string;
    node_handle.getParam("path1", path1_string);
    std::string path2_string;
    node_handle.getParam("path2", path2_string);
    /*if (node_handle.hasParam("path"))
    {
    ROS_WARN("the path.c_str() is %s",path_string.c_str());
    }
    else
    {
    ROS_WARN("No path parameter found");
    }*/

    oss << *count;
    if (strcmp(msg->encoding.c_str(),"32FC1")==0)
        {
    //ROS_WARN("Creating depth name.");
    sstm << path1_string.c_str() << "depth_" << oss.str() << ".jpg";
        }else
    {
    //ROS_WARN("Creating rgb name.");
    sstm << path2_string.c_str() << "rgb_" << oss.str() << ".jpg";
    }
    filename = sstm.str();

    cv::imwrite(filename, image);
    ROS_INFO("Saved image %s", filename.c_str());
        *count = *count +1;
        } else {
        ROS_WARN("Couldn't save image, no data!");
        }

}



void callback(ros::NodeHandle& node_handle, const sensor_msgs::ImageConstPtr& Depth_msg, const sensor_msgs::ImageConstPtr& RGB_msg)
{   
    //ros::Rate loop_rate(30);
    //ROS_WARN("Entering the callback.");
    node_handle.getParam("sec_per_frame",spf);
    //secperframe(node_handle,spf);
    int a= 1.0/spf;
    ros::Rate loop_rate(a);
    //ROS_INFO("looprate %i",a);
    //ROS_WARN("Starting extraction of depth.");
    extraction(Depth_msg,&count_depth, node_handle);
    //ROS_WARN("Starting extraction of rgb.");
    extraction(RGB_msg,&count_rgb, node_handle);
}



int main(int argc, char **argv)
{

  // Initialization
  ros::init(argc, argv, "extraction_synchronized");

  // Handler
  ros::NodeHandle nh;

  // Handler
  ros::NodeHandle nh_priv("~");

  // Suscribers to the RGB and depth kinect message
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth_image", 10);
    ROS_INFO("%s", "Suscriber of Depth set");
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "rgb_image", 10);
    ROS_INFO("%s", "Suscriber of RGB set");

  // Aproximate Time Synchronizer
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
  sync.registerCallback(boost::bind(&callback,nh_priv,_1, _2));
  ROS_INFO("%s", "Depth and RGB synchronized");

  ros::spin();


  return 0;
}

// ADDED to have good conversion
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}
