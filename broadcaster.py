#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
import tf.transformations
from geometry_msgs.msg import Twist


def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "odom",
                     "map")
    print(msg.pose.pose.position, msg.pose.pose.orientation)


def listener():
    rospy.init_node('broadcaster')
    rospy.Subscriber("/mavros/odometry/in", Odometry, callback)


    rospy.spin()

if __name__ == '__main__':
    listener()
