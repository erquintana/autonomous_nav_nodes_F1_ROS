#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import numpy as np
import rospy

track           =   rospy.get_param("track")
odom_topic      =   rospy.get_param("odom_topic")
raceline_path   =   rospy.get_param("raceline_path")

class RacelineDataPublisher(object):
    def __init__(self):

        self.wpts            = rospy.get_param("wpts_path")
        self.delimiter       = rospy.get_param("DELIMITER")
        self.skip            = rospy.get_param("SKIP_ROWS")
        self.X_IDX           = rospy.get_param("X_IDX")
        self.Y_IDX           = rospy.get_param("Y_IDX")
        self.i = 0

        self.raceline_path  = PoseStamped() 
        self.pth = Path()

        self.odom_sub       =   rospy.Subscriber(odom_topic, Odometry, self.racelice_viz_odomCallback, queue_size=10)
        self.raceline_path_pub = rospy.Publisher(raceline_path, Path, queue_size=10)        
        
        self.wpts_raceline = np.loadtxt(self.wpts, delimiter=self.delimiter, skiprows=self.skip)
        self.e = len(self.wpts_raceline)
        self.zeros = np.zeros( ( self.e , 1 ) , dtype=float)  

        self.raceline_arr = np.vstack((self.wpts_raceline[:, self.X_IDX], self.wpts_raceline[:, self.Y_IDX])).T 
        self.raceline_arr = np.append(self.raceline_arr, self.zeros, axis=1)

    def racelice_viz_odomCallback(self, data):
            
            path_stamped = PoseStamped()
            path_stamped.header = data.header
            # pose position ( x,y,z):
            path_stamped.pose.position.x = self.raceline_arr[self.i][0]    
            path_stamped.pose.position.y = self.raceline_arr[self.i][1]
            path_stamped.pose.position.z = self.raceline_arr[self.i][2]
            
            # pose orientation:
            path_stamped.pose.orientation.x = 0
            path_stamped.pose.orientation.y = 0
            path_stamped.pose.orientation.z = 0
            path_stamped.pose.orientation.w = 1

            # path data to publish:
            self.pth.header = data.header
            self.pth.poses.append(path_stamped)

            
            self.raceline_path_pub.publish(self.pth)
            #print("\n ----- ----- -----\n", self.pth)

            if(self.i < len(self.raceline_arr)-3):
                self.i = self.i + 1
            else:
                self.i = 0
            
            if(self.pth.header.seq > 1):           
                self.pth.poses.pop(0)
            
            return 0

def main():
    rospy.init_node('raceline_plotter_node')
    plotter = RacelineDataPublisher()
    print("\n\tRaceline Visualization node working!")
    print("\tVisualizing raceline for path tracking on RViz now.")
    print("\tPlotted track raceline:\t", track)
    print("\n\t\tRaceline is beeing published to /raceline_path topic!!\n")
    rospy.spin()
    return 0


if __name__ == '__main__':
    main()
