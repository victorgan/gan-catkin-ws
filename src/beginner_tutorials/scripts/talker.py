#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def talker():

    # Using std_msgs.msg/String type
    # Topic is 'chatter'
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # name of this node is 'talker'
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass # so code doesn't continue after sleep()
