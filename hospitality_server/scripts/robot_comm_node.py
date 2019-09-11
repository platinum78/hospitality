#!/usr/bin/python

import os, sys, threading
import json
import BaseHTTPServer

import rospy
from std_msgs.msg import Int32
from hospitality.srv import *
from hospitality.msg import PointFloor

def make_handler(ros_obj_dict):
    class HTTPHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        # find_route_service = ros_obj_dict["find_route_service"]
        update_uri_service = ros_obj_dict["update_uri_service"]
        update_position_service = ros_obj_dict["update_position_service"]

        def do_POST(self, request):
            try:
                if "taskReport" in self.path:
                    # Code for updating task accomplishments.
                    pass
                elif "uriReport" in self.path:
                    response = self.rfile.read()
                    response_data = json.loads(response)
                    robot_id = response_data["robot_id"]
                    tcp_ip = response_data["tcp_ip"]
                    tcp_port = response_data["tcp_port"]
                    self.update_uri_service(robot_id, tcp_ip, tcp_port)
                elif "posReport" in self.path:
                    response = self.rfile.read()
                    response_data = json.loads(response)
                    robot_id = response_data["robot_id"]
                    pos_x = response_data["pos_x"]
                    pos_y = response_data["pos_y"]
                    pos_floor = response_data["pos_floor"]
                    pos_msg = PointFloor(pos_x, pos_y, pos_floor)
                    self.update_position_service(robot_id, pos_msg)
                
                self.send_response(200)
                self.end_headers()
                self.wfile.write("OK")
            except:
                self.send_response(500)
                self.end_headers()
                self.wfile.write("ERROR")
    
    return HTTPHandler


class RobotCommNode(object):
    def __init__(self):
        # Retrieve parameters from ROS param server.
        self.server_ip = rospy.get_param("/main_server/network/tcp_ip")
        self.server_port = rospy.get_param("/main_server/network/tcp_port")

        # Wait until all services are up.
        rospy.wait_for_service("/update_uri_service")
        rospy.wait_for_service("/update_position_service")

        # Connect to services.
        self.update_uri_service = rospy.ServiceProxy("/update_uri_service", UpdateURI)
        self.update_position_service = rospy.ServiceProxy("/update_position_service", UpdatePosition)

        # Create HTTP server.
        ros_obj_dict = {
            "update_uri_service": self.update_uri_service,
            "update_position_service": self.update_position_service
        }
        self.http_handler = make_handler(ros_obj_dict)
        self.server_address = (self.server_ip, self.server_port)
        self.httpd = BaseHTTPServer.HTTPServer(self.server_address, self.http_handler)

    def start_server(self):
        self.httpd.serve_forever()
    
    def stop_server(self):
        stop_thread = threading.Thread(target=self.httpd.shutdown, args=())
        stop_thread.start()
        stop_thread.join()
    
    def exec_node(self):
        self.start_server()
    
    def stop_node(self):
        self.stop_server()

if __name__ == "__main__":
    try:
        rospy.init_node("robot_comm_node")
        node = RobotCommNode()
        rospy.on_shutdown(node.stop_node)
        node.exec_node()
    except rospy.ROSInterruptException:
        pass