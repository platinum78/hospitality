#!/usr/bin/python

import os, sys, threading
import json
import BaseHTTPServer

import rospy
from std_msgs.msg import Int32
from hospitality.srv import *

def make_handler(ros_obj_dict):
    class HTTPHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        find_route_service = ros_obj_dict["find_route_service"]

        def do_POST(self, request):
            if "taskReport" in self.path:
                # Code for updating task accomplishments.
                pass
            elif "ipReport" in self.path:
                response = self.rfile.read()
                


class RobotCommServerNode:
    def __init__(self):
        rospy.init_node("robot_comm_node")

        # rospy.wait_for_service("/find_route_service")
        # self.find_route_client = rospy.ServiceProxy("/find_route_service", FindRoute)

        self.publisher = rospy.Publisher("/test_topic", Int32, queue_size=1000)
        
        # rospy.loginfo("Robot communications node started.")

        log = logging.getLogger("werkzeug")
        log.setLevel(logging.ERROR)
        self.app.add_url_rule("/", view_func=self.index)

    def index(self):
        msg = Int32(2)
        self.publisher.publish(msg)
        return self.app.make_response(("hello!", 200))
    
    def run(self):
        self.app.run("0.0.0.0", 8000)

if __name__ == "__main__":
    try:
        node = RobotCommServerNode()
        node.run()
    except rospy.ROSInterruptException:
        raise KeyboardInterrupt()