#!/usr/bin/env python3

import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
 

# --------------- Driving parameters: --------------- 
# Speed, steering and distance thresholds:
MAX_DRIVE_SPEED                         =   8                              # m/s
MAX_TURN_SPEED                          =   MAX_DRIVE_SPEED/1.925                 # m/s

TURN_DISTANCE                           =   3   #3.12                                # m
TURN_STEERING_ANGLE                     =   0.220                              # rad

FREE_SPACE                              =   5.25
POST_CURVE_AND_FREE_SPACE               =   8.5
DELTA_SPEED_RAISE                       =   0.0060                           # m/s
DELTA_SPEED_POST_CURVE_AND_FREE_SPACE   =   0.0095
DELTA_SPEED_FALL                        =   0.07785                               # m/s

#DELTA_TURN                              =   0.005
DELTA_TURN_PROXIMAL                     =   0.0375
DELTA_TURN_DISTAL                       =   0.00122
DELTA_TURN_POST_CURVE_AND_FREE_SPACE    =   0.0000825


accel                                   =   0

drive_topic         =   rospy.get_param("drive_global")
odom_topic         =   rospy.get_param("odom_topic")
track               =   rospy.get_param("track")

# ---------------------------------------------------
drive = AckermannDriveStamped()             # Ackerman msgs initialize


class BraitenbergPlus(object):
    def __init__(self):
        self.first_scan_pass = True
        # Publishers:
        self.drive_pub = rospy.Publisher('drive_topic', AckermannDriveStamped, queue_size=100)

        # Subscribers:
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100)
        self.odom_sub = rospy.Subscriber('odom_topic', Odometry, self.odom_callback, queue_size=100)

    def scan_callback(self, scan_msg):
        # NOTE: scan measurements are taken in counter-clockwize direction
        scan_measures = scan_msg.ranges
        n = 12                                    # 22.5º each, zones to split scan measurements
        zones = np.array_split(scan_measures, n)  # split scan msgs into n equal parts
        
        # Splits ranges in two:
        LR = np.array_split(scan_measures, 2)     # L and R split (2 Zones)

        # Separation on L and R avg measures:
        L = round(sum(LR[1]) / len(LR[1]),6)
        R = round(sum(LR[0]) / len(LR[0]),6)
        Center = scan_measures[540]
        # Setting initial speed:
        if(self.first_scan_pass == True):
            self.first_scan_pass = False
            drive.drive.speed = MAX_DRIVE_SPEED
            
        
        # -------------- GOOOOOOO: -----------------------  
        #   +steering: left | -steering: rigth
        print ("\n|---------------------------------------|")
        if(Center < FREE_SPACE):    
            delta_turn =   DELTA_TURN_PROXIMAL     # proximal Object
            accel = DELTA_SPEED_FALL
            drive.drive.speed           =    drive.drive.speed - accel
            print("\t Braking!!!") 
        elif(Center > POST_CURVE_AND_FREE_SPACE):
            delta_turn = DELTA_TURN_POST_CURVE_AND_FREE_SPACE    # distal Object
            accel = DELTA_SPEED_POST_CURVE_AND_FREE_SPACE
            drive.drive.speed           =    drive.drive.speed + accel
            print("\t ACCELERATING A LOOOOOT!!!")   
        else:
            delta_turn = DELTA_TURN_DISTAL   # distal Object
            accel = DELTA_SPEED_RAISE
            drive.drive.speed           =    drive.drive.speed + accel
            print("\t Accelerating!!!")   

        if (R <= TURN_DISTANCE):
            # TURN L
            print (" <<<<< Left turning") 
            drive.drive.steering_angle =   (drive.drive.steering_angle + delta_turn)    # turn left
            drive.drive.speed           =    MAX_TURN_SPEED
        elif(L < TURN_DISTANCE):
            # TURN R
            print ("\t\t\t Rigth turning >>>>>")
            drive.drive.steering_angle  =    (drive.drive.steering_angle - delta_turn)     # turn rigth
            drive.drive.speed           =    MAX_TURN_SPEED
        else:
            drive.drive.steering_angle  =   0.0
            drive.drive.speed           =    drive.drive.speed    
        
        # BOUNDING ACTUATION:
        # Drive speed bounded output upper limit:
        if(drive.drive.speed >= MAX_DRIVE_SPEED):
            drive.drive.speed = MAX_DRIVE_SPEED
        # Drive speed bounded output lower limit:
        elif(drive.drive.speed <= MAX_TURN_SPEED):
            drive.drive.speed = MAX_TURN_SPEED
        
        # Bounding drive steering upper limit when turnig:
        if(drive.drive.steering_angle  >= TURN_STEERING_ANGLE):
            drive.drive.steering_angle  = TURN_STEERING_ANGLE
        # Bounding drive steering upper limit when turning:
        elif(drive.drive.steering_angle  <= -TURN_STEERING_ANGLE):
            drive.drive.steering_angle  = -TURN_STEERING_ANGLE
         
        
        
        
        print("Velocity: \t\t\t", round(drive.drive.speed, 4), 
            "\n Steering: \t\t\t", round(drive.drive.steering_angle, 4),    
            "\n Left obstacle avg distance: \t",  L,
            "\n Rigth obstacle avg distance: \t", R,
            "\n Free Space:\t\t\t",round(Center, 4))
        

        # Publishing drive msgs:
        self.drive_pub.publish(drive)

    def odom_callback(self, odom_msg):
        #print( odom_msg )
        return 0

def main():
    rospy.init_node('braitenbergPlus_node')
    braitenbergPlus = BraitenbergPlus()
    print("\n\tbraitenbergPlus node working!")
    print("\tFollowing path of ",track ," track now.")
    print("\n\t\tAckerman actuations are beeing published to /drive topic!!")

    rospy.spin()

if __name__ == '__main__':
    main()

    