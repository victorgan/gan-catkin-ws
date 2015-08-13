#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import PointCloud2

def callback_plane_coefficients(cloud):
    rospy.loginfo(rospy.get_caller_id()+": Running...")
    height = cloud.height
    width = cloud.width
    rospy.loginfo(": Height: %s", height)
    rospy.loginfo(": Width: %s", width)
    rospy.loginfo(": Field: %s", cloud.fields)
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s", cloud)

def listener():
    node_name='subscribe_pointcloud'
    rospy.init_node(node_name, anonymous=True) # anonymous=True flag : chooses unique node_name if needed

    topic_name = "voxelpoints"
    rospy.Subscriber(topic_name, PointCloud2, callback_plane_coefficients)
    rospy.spin() # keeps python from exiting until this node is stopped

if __name__ == '__main__':
    listener()
