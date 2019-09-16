#!/usr/bin/python

import rospy, rospkg
from hospitality_msgs.msg import PointFloor
from hospitality_msgs.srv import *

class MapServerTestNode(object):
    def __init__(self):
        self.find_route_service = rospy.ServiceProxy("/find_route_service", FindRoute)
        self.set_path_service = rospy.ServiceProxy("/set_path_service", SetPath)
        self.set_mode_service = rospy.ServiceProxy("/set_mode_service", SetMode)
    
    def exec_loop(self):
        while True:
            start = PointFloor()
            start.x = float(input("Type x coordinate >>> "))
            start.y = float(input("Type y coordinate >>> "))
            start.floor = int(input("Type floor >>> "))
            
            dest = PointFloor()
            dest.x = float(input("Type x coordinate >>> "))
            dest.y = float(input("Type y coordinate >>> "))
            dest.floor = int(input("Type floor >>> "))

            response = self.find_route_service(start, dest)
            print(response.waypoints)

            self.set_path_service(response.waypoints)
            self.set_mode_service(1)



def main():
    node = MapServerTestNode()
    while not rospy.is_shutdown():
        node.exec_loop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass