#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    data.child_frame_id = 'base_footprint'
    data.header.frame_id = 'odom_transformed'
    pub.publish(data)

def listener():
    global pub
    pub = rospy.Publisher('/odom_transformed', Odometry, queue_size=10)
    rospy.init_node('create_new_odom', anonymous=True)
    rospy.Subscriber('/t265/odom/sample', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()