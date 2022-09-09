#!/usr/bin/env python3

import numpy as np
import random
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon


def randomwalk2D(n):
    # [0, 0, 0, ... ,0]
    x = np.zeros(n)
    y = np.zeros(n)
    directions = ["UP", "DOWN", "LEFT", "RIGHT"]
    for i in range(1, n):
        # Pick a direction at random
        step = random.choice(directions)
        
        # Move the object according to the direction
        if step == "RIGHT":
            x[i] = x[i - 1] + 0.1
            y[i] = y[i - 1]
        elif step == "LEFT":
            x[i] = x[i - 1] - 0.1
            y[i] = y[i - 1]
        elif step == "UP":
            x[i] = x[i - 1]
            y[i] = y[i - 1] + 0.1
        elif step == "DOWN":
            x[i] = x[i - 1]
            y[i] = y[i - 1] - 0.1
    
    # Return all the x and y positions of the object
    return x, y
    


def talker():
    pub = rospy.Publisher('/random_walk', PoseStamped, queue_size=1)
    rospy.init_node('random', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    pose = PoseStamped()
    i= 0 
    while not rospy.is_shutdown():
        
        
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        poses =Quaternion(0,0,0,1)
        
        
        x, y = randomwalk2D(100000)
        pose.pose.orientation = poses
        
        if( x[i] <= -2.5 or  x[i] >= 2.5 or y[i] <= -2.5 or y[i] >= 2.5 ):
        	pose.pose.position.x = x[i]
        	pose.pose.position.y = y[i]
        	i = i - 1
        
        else:
        	pose.pose.position.x = x[i]
        	pose.pose.position.y = y[i]
        	i = i+1
        
        
         
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
