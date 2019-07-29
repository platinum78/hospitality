#!/usr/bin/python

import os, sys
import rospy
from topic_manager import TopicManager
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

def rotation_matrix(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle),  np.cos(angle)]])

def main():
    rospy.init_node("turtlebot3_drive_node", anonymous=True)
    tmgr = TopicManager()
    loop_rate = rospy.Rate(10)
    
    repulsion_const = 1
    repulsion_power = 1.5
    repulsion_forces_scalar = np.zeros(360)
    repulsion_forces_vector = np.zeros([360, 2])

    grav_const = 1
    grav_dir = np.pi / 10

    waypoint_interval = 0.1
    waypoint_reach_thres = 0.02
    kp = 10
    kt = 5

    sine = np.sin(np.linspace(0, 2 * np.pi, 360))
    cosine = np.cos(np.linspace(0, 2 * np.pi, 360))
    
    reached_local_dest = True
    reached_global_dest = False

    while not tmgr.is_system_ready():
        pass
    
    while not rospy.is_shutdown():
        if reached_local_dest:
            # Calculate repulsion forces.
            scan_ranges = tmgr.scan_ranges                                                          # Robot coordinate
            scan_ranges[scan_ranges < 0.05] = 0.05
            repulsion_forces_scalar[:] = repulsion_const / (scan_ranges ** repulsion_power)
            repulsion_forces_vector[:,0] = repulsion_forces_scalar * cosine                         # Robot coordinate
            repulsion_forces_vector[:,1] = repulsion_forces_scalar * sine                           # Robot coordinate
            repulsion_net_force = -np.sum(repulsion_forces_vector, axis=0)

            # Calculate gravitational forces.
            pose = tmgr.pose                                                                        # Inertial coordinate
            angle_robot = grav_dir - pose.orientation.z                                             # Robot coordinate
            grav_net_force = np.array([np.cos(angle_robot), np.sin(angle_robot)]) * grav_const      # Robot coordinate
            
            # Calculate net forces.
            net_force_vector = repulsion_net_force + grav_net_force
            net_force_scalar = np.sqrt(np.sum(net_force_vector ** 2))
            net_force_unit_vector = net_force_vector / net_force_scalar
            pos_diff_robot = net_force_unit_vector * waypoint_interval                              # Robot coordinate
            
            # Calculate next local destination.
            pose = tmgr.pose
            position_inertial = np.array([pose.position.x, pose.position.y])
            pos_diff_inertial = np.matmul(rotation_matrix(pose.orientation.z), pos_diff_robot)
            local_dest_inertial = position_inertial + np.reshape(pos_diff_inertial, [2])
            reached_local_dest = False
        
        # Calculate PID errors and publish velocities.
        position = np.array([pose.position.x, pose.position.y])
        pos_diff_vector = (local_dest_inertial - position) ** 2
        pos_diff_scalar = np.sqrt(np.sum(pos_diff_vector))
        angle_diff = np.arctan2(pos_diff_vector[1], pos_diff_vector[0]) - pose.orientation.z
        tmgr.publish_cmd_vel(pos_diff_scalar * kp, angle_diff * kt)
        
        if pos_diff_scalar < waypoint_reach_thres:
            rospy.loginfo("Waypoint reached!")
            reached_local_dest = True
        
        print "Repulsion net force: ", repulsion_net_force
        print "Gravitation net force: ", grav_net_force
        print "Position difference: ", pos_diff_vector
        loop_rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass