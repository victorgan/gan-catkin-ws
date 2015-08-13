#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// namespace enc = sensor_msgs::image_encodings;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    static const std::string OPENCV_WINDOW = "Image window";
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    cv::imwrite("/home/vgan/code/temp/wee.jpg", cv_ptr->image);
// 
//     int depth = cv_ptr->image.at<short int>(cv::Point(240,320));//you can change 240,320 to your interested pixel
//     ROS_INFO("Depth: %d", depth);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "writedepthimage");
  ros::NodeHandle n;
  int bufferQueue = 1000;
  ros::Subscriber sub = n.subscribe("camera/depth_registered/image_raw", bufferQueue, imageCallback);
  ros::spin();
  return 0;
}
