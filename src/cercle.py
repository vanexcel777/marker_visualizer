#!/usr/bin/env python3

import numpy as np
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon



def talker():
    pub = rospy.Publisher('/cercle', PoseStamped, queue_size=10)
    rospy.init_node('random', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    raduis=  2 * (2)**(1/2)
    pose = PoseStamped()
    theta=0
    while not rospy.is_shutdown():
        theta = theta + 0.1
        
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        poses =Quaternion(0,0,0,1)
        
        x = raduis*np.sin(theta)
        y = raduis*np.cos(theta)
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = poses
         
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
