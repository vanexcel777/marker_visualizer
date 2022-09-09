#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon


class RvizMarkers(object):
    def __init__(self):
        self.marker_pub = rospy.Publisher('/marker_test', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)

    def init_marker(self, index=0, z_val=0):
        self.marker_sphere = Marker()
        self.marker_sphere.header.frame_id = "map"
        self.marker_sphere.header.stamp    = rospy.get_rostime()
        self.marker_sphere.ns = "some_robot"
        self.marker_sphere.id = index
        self.marker_sphere.type = Marker().SPHERE
        self.marker_sphere.action = Marker().ADD
    
        poses = Pose(Point(0,0,0),Quaternion(0,0,0,1))
        scale = Vector3(0.5,0.5,0.5) # diameter
        
        path_color = ColorRGBA()
        path_color.a = 1
        path_color.r = 0
        path_color.g = 0
        path_color.b = 1
        

        self.marker_sphere.pose = poses
        self.marker_sphere.scale = scale             
        self.marker_sphere.color = path_color 
    
        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_sphere.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.marker_sphere)
            self.marker_pub.publish(self.marker_sphere)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marker_RvizMarkers_sphere', anonymous=True)
    marker_RvizMarkers_sphere = RvizMarkers()
    try:
        marker_RvizMarkers_sphere.start()
    except rospy.ROSInterruptException:
        pass
