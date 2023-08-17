#!/usr/bin/env python3

from sys import path
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

PLANNER_TYPE = rospy.get_param("PLANNER_TYPE")
track               =   rospy.get_param("track")
drive_global = rospy.get_param("drive_global")
drive_local = rospy.get_param("drive_local")

drive = rospy.get_param("drive")
odom_topic = rospy.get_param("odom_topic")

GSW = rospy.get_param("global_speed_weight")
LSW  = rospy.get_param("local_speed_weight")
GSTRW = rospy.get_param("global_steer_weight")
LSTRW  = rospy.get_param("local_steer_weight")

class PathPlanningNode:
    def __init__(self):
        self.drv_global_speed       =   0
        self.drv_global_steering    =   0
        self.drv_local_speed        =   0
        self.drv_local_steering     =   0
        rospy.init_node("path_planner_node")
        rospy.loginfo("Starting PathPlanningNode as path_planner_node.")

        self.global_drive_sub =     rospy.Subscriber(drive_global, AckermannDriveStamped, self.global_drive_callback)
        self.local_drive_sub =  rospy.Subscriber(drive_local, AckermannDriveStamped, self.local_drive_callback)        
        
        # Subscribers:
        self.drive_pub  =   rospy.Publisher(drive, AckermannDriveStamped, queue_size=100)
        


    def switch_planner(self):

        return 0
    
    def dynamic_weight(self):
        global_speed_weight    =   GSW
        local_speed_weight     =   LSW
        global_steer_weight    =   GSTRW
        local_steer_weight     =   LSTRW
        
                        
        return global_speed_weight, local_speed_weight, global_steer_weight, local_steer_weight
    
    def global_drive_callback(self, g_msg):
        self.drv_global_speed       = g_msg.drive.speed
        self.drv_global_steering    = g_msg.drive.steering_angle
        
        
        self.drive_handler()  
        
        return 0

    def local_drive_callback(self, l_msg):
        self.drv_local_speed       = l_msg.drive.speed
        self.drv_local_steering    = l_msg.drive.steering_angle
        return 0

    def drive_handler(self):
        drive_msg = AckermannDriveStamped()

        global_speed_weight, local_speed_weight, global_steer_weight, local_steer_weight = self.dynamic_weight()
        
        speed_overall = global_speed_weight*self.drv_global_speed + local_speed_weight*self.drv_local_speed
        steer_overall = global_steer_weight*self.drv_global_steering + local_steer_weight*self.drv_local_steering
        
        drive_msg.drive.speed = speed_overall
        drive_msg.drive.steering_angle  =  steer_overall
        
        self.drive_pub.publish(drive_msg)
        return 0

def main():
    pathPlanningNode = PathPlanningNode()
    print("\n\tpathPlanning node working!")
    print("\tFollowing path of ",track ," track now.")
    print("\n\t\tAckerman actuations are beeing published to /drive topic!!")

    rospy.spin()

if __name__ == '__main__':
    main()
