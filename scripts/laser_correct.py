#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

pub = rospy.Publisher('robot/z1_revised_scan', LaserScan, queue_size = 10)
pub_base = rospy.Publisher('robot/base_revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()
scann_base = LaserScan()

def callback(msg):
    current_time = rospy.Time.now()
    scann.header.stamp = current_time
    scann.header.frame_id = 'gripperMover'
    scann.angle_min = 0
    scann.angle_max = -2*3.1415
    scann.angle_increment = msg.angle_increment
    scann.time_increment = msg.time_increment
    scann.range_min = 0.12 
    scann.range_max = 180
    scann.ranges = msg.ranges
    scann.intensities = msg.intensities
    pub.publish(scann)

def callback_base(msg):
    current_time = rospy.Time.now()
    scann_base.header.stamp = current_time
    scann_base.header.frame_id = 'world'
    scann_base.angle_min = 0
    scann_base.angle_max = -2*3.1415
    scann_base.angle_increment = msg.angle_increment
    scann_base.time_increment = msg.time_increment
    scann_base.range_min = 0.12 
    scann_base.range_max = 180
    scann_base.ranges = msg.ranges
    scann_base.intensities = msg.intensities
    pub_base.publish(scann_base)

def listener():
    rospy.init_node('laser_correct', anonymous=True)
    sub = rospy.Subscriber('robot/z1_scan', LaserScan, callback)
    sub = rospy.Subscriber('robot/base_scan', LaserScan, callback_base)
    rospy.spin()

if __name__ == '__main__':
    listener()