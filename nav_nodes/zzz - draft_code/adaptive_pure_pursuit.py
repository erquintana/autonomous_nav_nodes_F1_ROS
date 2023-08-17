#!/usr/bin/env python3

from utils import get_actuation, nearest_point_on_trajectory_py2, first_point_on_trajectory_intersecting_circle
import csv
from inspect import _void
import math
from os import close
from sys import path
from traceback import print_tb

from numba.core.target_extension import current_target
import rospy

import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path


# Waypoint util global variables:
catalunya_centerline = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/f1tenth_racetracks/Catalunya/Catalunya_centerline.csv" 
test_path = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/f1tenth_racetracks/Catalunya/test_path.csv" 
skirt = "/home/erq/Documents/Proyecto_electrico/ackerman-f1tenth-competition/05_-_F1TENTH_ws/src/f1tenth_gym_ros/maps/skirk.csv"
# Change between waypoint files to use desired:
WAYPOINTS_FILE = catalunya_centerline

# PID tunning global variables:
P   =   3.5
I   =   0.0
D   =   0.0

# Vehicle params:
wheel_base  =   0.3302
mass        =   3.47
l_r         =   0.17145
I_z         =   0.04712
mu          =   0.523
h_cg        =   0.074
cs_f        =   4.718
cs_r        =   5.4562
PSLN        =   0.05 
lookPSLN    =   0.000000000000001
max_steer   =   0.41
min_steer   =   -0.41
v_const     =   0.75
steering_angle = 0.0 

# timing global variables:
Ts      =   0.0    # Periodo de muestreo   
tk      =   0.0
tk_1    =   0.0
t       =   0.0
t_i     =   0.0

actuaction = AckermannDriveStamped()
#actuaction.drive.acceleration               =   0.0
#actuaction.drive.jerk                       =   0.0
actuaction.drive.speed                      =   0.0
actuaction.drive.steering_angle             =   0.0
#actuaction.drive.steering_angle_velocity    =   0.0

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
        self.steering_angle     =   0.0 
        
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
        g_x              =   self.path[0][0]
        g_y              =   self.path[1][0]

        goal_x = float(g_x)
        goal_y = float(g_y)
        
        if(goal_x == 0.0): 
            goal_x = PSLN
        if(goal_y == 0.0):
            goal_y = PSLN 

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
        #self.pose_callback()
        self.adaptive_PP_nav(self.current_x, self.current_y, self.current_heading, self.lookahead_distance)
        return 0

    def adaptive_PP_nav(self, xcurrent, ycurrent, headingcurrent, lookahead ):
        look = self.find_lookahead_distance(xcurrent, ycurrent)
        closest = self.find_closest_point_it(xcurrent, ycurrent)
        if(t_i > closest): 
            curv = self.curvature_to_lookahead_point(xcurrent, ycurrent, headingcurrent, look)
        else:
            curv = 0.0000000001
        
        desired_steering                            =   math.atan(wheel_base * curv)
        actuaction.drive.speed                      =   v_const
        
        #self.steering_angle  =  self.steering_angle + self.P_controller(P, desired_steering, headingcurrent)
        
        #actuaction.drive.steering_angle = self.anti_windup(self.steering_angle, max_steer, min_steer)
        actuaction.drive.steering_angle =  self.anti_windup(desired_steering, max_steer, min_steer)
        self.drive_pub.publish(actuaction)
        
        print("\t--- ---\n\t\tADAPTIVE PP NAVIGATION")
        print("\tlook\t  :", look)
        print("\tclosest\t  :", closest)
        print("\tcurvature :", curv)
        print("\tactuations :\n", actuaction)
        
        return 0
        
    def waypoint_loader(self):
        file = open(WAYPOINTS_FILE) 
        csvreader = csv.reader(file)
        header = next(csvreader)
        rows = []
        
        for row in csvreader:
            rows.append(row)

        path = []

        for i in range(len(rows)):
            path.append( [ float(rows[i][0]) , float(rows[i][1]) ] )            
        
        return path

    def goal_checker(self, xcurrent, ycurrent, xgoal, ygoal):
        self.d_to_goal = self.distance_current_to_desired_point(xcurrent, ycurrent, xgoal, ygoal)
        if( self.d_to_goal < math.pow(PSLN,2)):
            self.path.pop(0)
            print("\tArrived to goal!! :)")
            return True
        else:
            return False
    
    def distance_current_to_desired_point(self, xcurrent, ycurrent, xgoal, ygoal):
        d = math.sqrt(math.pow( np.abs(xcurrent-xgoal), 2 ) + math.pow( np.abs(ycurrent - ygoal), 2))
        return d

    def anti_windup(self, input, max_out, min_out):
        if(input >= max_out):
            output = max_out
        elif(input<=min_out):
            output = min_out
        else:
            output = input            
        return output

    def find_closest_point_it(self, xcurrent, ycurrent ):
        d = math.sqrt((self.path[0][0] - xcurrent)** 2 + (self.path[0][1] - ycurrent)** 2)
        
        mindist_i = (0, d)

        for i, p in enumerate(self.path):
            dist = math.sqrt((p[0]-xcurrent)**2 + (p[1]-ycurrent)**2)
            if dist < mindist_i[1]:
                mindist_i = (i, dist)
        
        return mindist_i[0] 

    def find_lookahead_distance(self, xcurrent, ycurrent):
        for i, p in enumerate(reversed(self.path[:-1])):
            i_ = len(self.path)-2 - i
            d = (self.path[i_+1][0]-p[0], self.path[i_+1][1]-p[1])
            f = (p[0]-xcurrent, p[1]-ycurrent)

            a = sum(j**2 for j in d)
            b = 2*sum(j*k for j,k in zip(d,f))
            c = sum(j**2 for j in f) - self.lookahead_distance**2
            disc = b**2 - 4*a*c
            if disc >= 0:
                disc = math.sqrt(disc)
                t1 = (-b + disc)/(2*a)
                t2 = (-b - disc)/(2*a)
                # print("t1=" + str(t1) + ", t2=" + str(t2))
                if 0<=t1<=1:
                    # if (t1 >= t and i == t_i) or i > t_i:
                        t = t1
                        t_i = i_
                        # print("hit")
                        return p[0]+t*d[0], p[1]+t*d[1]
                if 0<=t2<=1:
                    # if (t2 >= t and i == t_i) or i > t_i:
                        t = t2
                        t_i = i_
                        # print("hit")
                        return p[0]+t*d[0], p[1]+t*d[1]

        return self.path[self.find_closest_point_it(xcurrent, ycurrent)][0:2]

    def curvature_to_lookahead_point(self, xcurrent, ycurrent, headingcurrent, lookahead):
        #side = np.sign(math.sin(math.pi/2 - headingcurrent)*(lookahead[0]-xcurrent) - math.cos(math.pi - headingcurrent)*(lookahead[1]-ycurrent))
        #a = -math.tan(math.pi/2 - headingcurrent)
        #c = math.tan(math.pi/2 - headingcurrent)*xcurrent - ycurrent
        ## x = abs(-math.tan(3.1415/2 - headingcurrent) * lookahead[0] + lookahead[1] + math.tan(math.pi- headingcurrent)*xcurrent - ycurrent) / math.sqrt((math.tan(3.1415/2 - headingcurrent))**2 + 1)
        #x = abs(a*lookahead[0] + lookahead[1] + c) / math.sqrt(a**2 + 1)

        #return side * (2*x/(self.lookahead_distance**2))

        alpha = math.abs(headingcurrent - math.atan((lookahead[1]-ycurrent)/(lookahead[0]-xcurrent)))
        curvature = 2*math.sin(alpha)/self.lookahead_distance
        return curvature

    def target_wheel_velocities(self):
        return 0
    
    def control_loop_for_velocities(self):
        return 0

    def P_controller(self, p, ref, measured):
        err = ref - measured
        output = p*err
        print("\n\n\t--- --- --- --- --- --- --- ---\n\t\t\tCONTROLLER")
        print("\tref = ", ref)
        print("\tmeasured = ", measured)
        print("\terr = ", err)
        print("\toutput = ", output)
        
        return output 

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
    

if __name__ == '__main__':
    main()
