#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

path               =   rospy.get_param("path_topic")
odom_topic         =   rospy.get_param("odom_topic")

class OdomPathPlotter(object):
    def __init__(self):
        self.pth = Path()
        self.odom_sub       =   rospy.Subscriber(odom_topic, Odometry, self.odom_to_path, queue_size=1)
        self.path_pub = rospy.Publisher(path, Path, queue_size=1)

    def odom_to_path(self, data):
            self.pth.header = data.header
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            self.pth.poses.append(pose)

            self.path_pub.publish(self.pth)
            
            if(self.pth.header.seq > 1):
                self.pth.poses.pop(0)
            
            return 0

def main():
    rospy.init_node('path_plotter_node')
    plotter = OdomPathPlotter()
    print("\n\tPlotting to Path node working!")
    print("\tVisualizing Odometry data on RViz now.")
    print("\n\t\tOdometry path is beeing published to /path topic!!\n")
    rospy.spin()
    return 0


if __name__ == '__main__':
    main()
