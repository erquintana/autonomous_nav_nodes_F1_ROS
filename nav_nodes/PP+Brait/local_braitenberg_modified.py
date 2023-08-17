#!/usr/bin/env python3

import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from rospy.client import get_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

drive_topic         =   rospy.get_param("drive_local")
odom_topic         =   rospy.get_param("odom_topic")
track               =   rospy.get_param("track")
n            =   rospy.get_param("sectors")

# Catalunya: 
TURN_DISTANCE = rospy.get_param("turn_dist")
DELTA_TURN  = rospy.get_param("delta_turn")

drive = AckermannDriveStamped()             # Ackerman msgs initialize



MAX_DRIVE_SPEED = 4
MAX_TURN_SPEED = MAX_DRIVE_SPEED/2
TURN_STEERING_ANGLE =   rospy.get_param("max_steering_angle")  


class BraitenbergMod(object):
    def __init__(self):
        self.dir = ""
        # Publishers:
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)

        # Subscribers:
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100)

    def scan_callback(self, scan_msg):
        # NOTE: scan measurements are taken in counter-clockwize direction
        scan_measures = scan_msg.ranges                                 # 22.5º each, zones to split scan measurements

        # Splits ranges in n:
        LR = np.array_split(scan_measures, n)     # L and R split (n Zones)

        # Separation on L and R avg measures:
        R_zone = int(n/2) - 1
        L_zone = int(n/2)
        R = round( (sum(LR[R_zone]) ) / (len(LR[0])) ,6)
        L = round( (sum(LR[L_zone]) ) / (len(LR[0])) ,6)
        Center = scan_measures[540]
            
        
        # -------------- GOOOOOOO: -----------------------  
        #   +steering: left | -steering: rigth

        if(L < TURN_DISTANCE):

            # turn rigth:
            drive.drive.steering_angle = (drive.drive.steering_angle - DELTA_TURN)
            print("\t\t\t\t\tTurning R")

        elif(R <= TURN_DISTANCE):
 

            drive.drive.steering_angle = (drive.drive.steering_angle + DELTA_TURN)
            print("\tTurning L")

        else:
            drive.drive.steering_angle = 0
        
        drive.drive.speed = MAX_DRIVE_SPEED
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
        

        print("\n Velocity: \t\t\t", round(drive.drive.speed, 4), 
            "\n Steering: \t\t\t", round(drive.drive.steering_angle, 4),    
            "\n Left obstacle avg distance: \t",  L,
            "\n Rigth obstacle avg distance: \t", R,
            "\n Free Space:\t\t\t",round(Center, 4))
        

        # Publishing drive msgs:
        self.drive_pub.publish(drive)
        return 0


def main():
    rospy.init_node('braitenbergPlus_node')
    braitenberg_mod = BraitenbergMod()
    print("\n\tbraitenbergPlus node working!")
    print("\tFollowing path of ",track ," track now.")
    print("\n\t\tAckerman actuations are beeing published to /drive topic!!")

    rospy.spin()

if __name__ == '__main__':
    main()


    