#!/usr/bin/env python3
from pp_utils import get_actuation, load_waypoints, get_current_waypoint
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import tf

wheel_base          =   rospy.get_param("wheelbase")
lookahead_distance  =   rospy.get_param("lookahead_distance")
P_GAIN              =   rospy.get_param("P_GAIN")
track               =   rospy.get_param("track")

drive_topic         =   rospy.get_param("drive_global")
odom_topic         =   rospy.get_param("odom_topic")

class PurePursuitPlanner():
    """
    Example Planner
    """
    def __init__(self):

        self.wheelbase          =   wheel_base
        self.current_x          =   0.0
        self.current_y          =   0.0
        self.current_heading    =   0.0
        self.current_vx         =   0.0 
        self.curvature          =   0.0

        self.actuaction         =  AckermannDriveStamped()

        self.waypoints = load_waypoints()

        self.in_planning_frt_pass = True

        # Publishers:
        self.drive_pub  =   rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)

        # Subscribers:
        self.odom_sub   =   rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=100)

    def odom_callback(self, odom_msg):

        if(self.in_planning_frt_pass):
            self.in_planning_frt_pass = False
            print("\n\tNavigating path now!!")
    
        orientation_list = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)
        
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        self.current_heading = theta
        self.current_vx = odom_msg.twist.twist.linear.x
        
        speed , steering = self.plan(self.current_x,self.current_y,self.current_heading, lookahead_distance, P_GAIN, self.waypoints)

        self.actuaction.drive.acceleration               =   0.0
        self.actuaction.drive.jerk                       =   0.0
        self.actuaction.drive.speed                      =   speed
        self.actuaction.drive.steering_angle             =   steering
        self.actuaction.drive.steering_angle_velocity    =   0.0
        
        self.drive_pub.publish(self.actuaction)
        
        return 0

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain, waypoints):
        position = np.array([pose_x, pose_y])
        lookahead_point = get_current_waypoint(waypoints, lookahead_distance, position, pose_theta)

        if lookahead_point is None:
            return 4.0, 0.0

        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        speed = P_GAIN*speed

        return speed, steering_angle


def main():
    rospy.init_node('pure_pursuit_node')
    pure_pursuit_planner = PurePursuitPlanner()
    print("\n\tPure pursuit node working!")
    print("\tFollowing path of ",track ," track now.")
    print("\n\t\tAckerman actuations are beeing published to /drive topic!!")

    rospy.spin()

if __name__ == '__main__':
    main()
