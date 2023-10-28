#!/usr/bin/env python3
# ROS python API
import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Transform

# import all mavros messages 
from mavros_msgs.msg import *

import numpy as np

def odom_cb(msg):
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),msg.header.stamp,"base_link","map")

# Main function
def main():
    rospy.init_node('map_baselink_tfpub', anonymous=True)
    rate = rospy.Rate(50)
    rospy.Subscriber('mavros/local_position/odom',Odometry,callback=odom_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        br = tf.TransformBroadcaster()
        main()
    except rospy.ROSInterruptException:
        pass