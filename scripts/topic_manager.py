#!/usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class TopicManager:
    def __init__(self):
        # Publishers
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

        # Subscribers
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Data storage
        self.scan_ranges = None
        self.twist_msg = Twist()
        self.pose = Odometry().pose.pose

        # System readiness flags.
        self.scan_ready = False
        self.odom_ready = False
    
    def publish_cmd_vel(self, linear, angular):
        self.twist_msg.linear.x = linear
        self.twist_msg.angular.z = angular
        self.cmd_vel_publisher.publish(self.twist_msg)

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_ready = True
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose
        self.odom_ready = True
    
    def is_system_ready(self):
        if not self.scan_ready:
            return False
        if not self.odom_ready:
            return False
        return True