#!/usr/bin/python

import numpy as np

import rospy, rospkg
from hospitality_msgs.msg import PointFloor
from hospitality_msgs.srv import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MapServerTestNode(object):
    def __init__(self):
        self.find_route_service = rospy.ServiceProxy("/find_route_service", FindRoute)
        self.set_path_service = rospy.ServiceProxy("/set_path_service", SetPath)
        self.set_mode_service = rospy.ServiceProxy("/set_mode_service", SetMode)
        self.pose = None
        rospy.wait_for_message("/odom", Odometry)
        self.odometry_subscriber = rospy.Subscriber("/odom", Odometry, queue_size=1000, callback=self.odometry_callback)
    
    def odometry_callback(self, msg):
        self.pose = msg.pose.pose
    
    def exec_loop(self):
        wait_rate = rospy.Rate(10)
        while self.pose == None:
            wait_rate.sleep()

        while True:
            dest = PointFloor()
            dest.x = float(input("Type x coordinate >>> "))
            dest.y = float(input("Type y coordinate >>> "))
            dest.floor = int(input("Type floor >>> "))

            start = PointFloor()
            q = self.pose.orientation
            orientation = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            start.x = self.pose.position.x + 0.1 * np.cos(orientation)
            start.y = self.pose.position.y + 0.1 * np.sin(orientation)
            start.floor = 1

            response = self.find_route_service(start, dest)
            print(response.waypoints)

            self.set_path_service(response.waypoints)
            self.set_mode_service(1)



def main():
    rospy.init_node("map_server_test_node")
    node = MapServerTestNode()
    while not rospy.is_shutdown():
        node.exec_loop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass