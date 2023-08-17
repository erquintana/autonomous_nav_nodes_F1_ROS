#!/usr/bin/env python3

import csv
from inspect import _void
import math
from sys import path
from traceback import print_tb

from numba.core.target_extension import current_target
import rospy

from utils import PID_smpl_ctrl
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path


# Waypoint util global variables:
catalunya_centerline = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/f1tenth_racetracks/Catalunya/Catalunya_centerline.csv" 
test_path = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/f1tenth_racetracks/Catalunya/test_path.csv" 
skirt = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/skirk.csv"
WAYPOINTS_FILE = catalunya_centerline

# PID tunning global variables:
P   =   1.0
I   =   0.0
D   =   0.0

# Vehicle params:
wheel_base   =   0.3302
mass        =   3.47
l_r         =   0.17145
I_z         =   0.04712
mu          =   0.523
h_cg        =   0.074
cs_f        =   4.718
cs_r        =   5.4562
EPSLN        =   0.05 
max_steer   =   0.41

# timing global variables:
Ts      =   0.0    # Periodo de muestreo   
tk      =   0.0
tk_1    =   0.0

actuaction = AckermannDriveStamped()
actuaction.drive.acceleration               =   0.0
actuaction.drive.jerk                       =   0.0
actuaction.drive.speed                      =   0.0
actuaction.drive.steering_angle             =   0.0
actuaction.drive.steering_angle_velocity    =   0.0

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):

        self.current_x          =   0.0
        self.current_y          =   0.0
        self.current_heading    =   0.0
        self.current_vx         =   3.0
        
        self.curvature          =   0.0
        self.lookahead_distance =   0.0

        self.d_to_goal          =   0.0
        
        self.path = self.waypoint_loader()
        
        # Publishers:
        self.drive_pub      =   rospy.Publisher('/drive', AckermannDriveStamped, queue_size=100)

        # Subscribers:
        self.odom_sub       =   rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=100)
        #self.clk_sub = rospy.Subscriber("clock", Clock, self.clock_callback, queue_size=100)

    def pose_callback(self):
        # TODO 1: find the current waypoint to track using methods mentioned in lecture
        # TODO 2: transform goal point to vehicle frame of reference
        # TODO 3: calculate curvature/steering angle
        # TODO 4: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians

        # Determine current location of vehicle:
        current_x           =       self.current_x
        current_y           =       self.current_y
        current_heading     =       self.current_heading
        current_vx          =       self.current_vx
        
        print(
            "\n --- --- --- --- --- --- --- ---\n"
            "current_x:\t",current_x ,
            "\ncurrent_y:\t",current_y ,
            "\ncurrent_heading:\t", current_heading , 
            "\ncurrent_vx:\t", current_vx)
        #Find the path point closest to vehicle:
        
        # Find the goal position:
        g_x = self.path[0][0]   # [fila , columna]
        g_y = self.path[0][1]   # [fila , columna]

        goal_x = float(g_x)
        goal_y = float(g_y)
        
        if(goal_x == 0.0): 
            goal_x = EPSLN
        if(goal_y == 0.0):
            goal_y = EPSLN 

        print("\nGOAL:\t(" , goal_x, ",", goal_y, ")\n")

        # Transform the goal point to vehicle coordinates:
        at_goal = self.goal_checker( current_x, current_y, goal_x, goal_y)   

        # Calculate the curvature and request the vehicle to set the
        # steering to that curvature:
        vx = 1.0
        alpha               =   current_heading - math.atan2( goal_y , goal_x - wheel_base/2 ) # lookahead to heading angle
        ld                  =   P*vx   # lookahead distance as P gain
        kappa               =   2*np.sin(alpha)/ld # curvature 2*sin(alpha)/ld
        #steering_actuation    =   math.atan2(kappa*wheel_base) # steering angle: arctan(kappa*wheel_base)  -> control law
        
        delta = alpha
        if delta > max_steer:
            delta = max_steer
        if delta < -max_steer:
            delta = -max_steer
        
        steering_actuation = delta

        print("\nalpha: " , alpha , "\nld: ", ld, "\nsteering_actuation: ", steering_actuation, "\nAt goal??\t", at_goal, "\nd to goal:\t", self.d_to_goal, "\nkappa: ", kappa)

        # Update the vehicles position
        # self.set_up_time()
        actuaction.drive.speed = vx
        actuaction.drive.steering_angle = steering_actuation
        self.drive_pub.publish(actuaction)

    def odom_callback(self, odom_msg):

        orientation_list = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)
        
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        self.current_heading = theta
        self.current_vx = odom_msg.twist.twist.linear.x
        self.pose_callback()
        
        return 0

    def waypoint_loader(self):

        file = open(WAYPOINTS_FILE) 
        csvreader = csv.reader(file)
        header = next(csvreader)
        rows = []
        
        for row in csvreader:
            rows.append(row)

        path = []
        path_x = []
        path_y = []

        for i in range(len(rows)):
            path.append([rows[i][0], rows[i][1]])            

        return path


    def PID_controller(self, P, I, D, desired, measured, delta_t):
        pid = PID_smpl_ctrl.PID(P, I, D, SetPoint = desired, dt = delta_t)
        output = pid.compute(feedback_value = measured)
        return output 

    def goal_checker(self, xcurrent, ycurrent, xgoal, ygoal):
        self.d_to_goal = self.distance_current_to_desired_point(xcurrent, ycurrent, xgoal, ygoal)
        if( self.d_to_goal < math.sqrt(math.pow(EPSLN,2))):
            self.path.pop(0)
            print("\tArrived to goal!! :)")
            return True
        else:
            return False
    
    def distance_current_to_desired_point(self, xcurrent, ycurrent, xgoal, ygoal):
        d = math.sqrt(math.pow( np.abs(xcurrent-xgoal), 2 ) + math.pow( np.abs(ycurrent - ygoal), 2))
        return d

    def anti_windup(self, input, output, max_out, min_out):
        if(input >= max_out):
            output = max_out
        elif(input<=min_out):
            output = min_out
        else:
            output = input            
        
        return output

    #def clock_callback(self, msg):
    #    tk = msg.clock.to_sec()

    #def set_up_time(self):
    #    if(Ts == 0.0):
    #        Ts = 0.01
#
    #    Ts = tk - tk_1
    #    tk_1 = self.tk

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
    

if __name__ == '__main__':
    main()
