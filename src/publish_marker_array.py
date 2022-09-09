#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations

class RvizMarkers_array(object):
    def __init__(self):
        self.marker_array_pub = rospy.Publisher('pace_delimiters', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)

    def init_marker(self, index=0, z_val=0):
        self.marker_path = Marker()
        self.marker_path.header.frame_id = "map"
        self.marker_path.header.stamp    = rospy.get_rostime()
        self.marker_path.ns = "Line"
        self.marker_path.id = index
        self.marker_path.type = Marker().LINE_STRIP
        self.marker_path.action = Marker().ADD
    
        path = []
        path.append( Point(5,0,0) )
        path.append( Point(0,5,0) )
        path.append( Point(-5,0,0) )
        path.append( Point(0,-5,0) )
        path.append( Point(5,0,0) )
        
        self.marker_path.points[:] = [] # clear
        self.marker_path.colors[:] = []
        
        path_color = ColorRGBA()
        path_color.a = 1
        path_color.r = 0
        path_color.g = 0
        path_color.b = 1
        
        self.marker_path.scale.x = 0.05
        for i in range(1, len(path)):
            # Start of segment is previous point
            self.marker_path.points.append(path[i-1])
            self.marker_path.colors.append(path_color)
            # End of segment is current point
            self.marker_path.points.append(path[i])
            self.marker_path.colors.append(path_color)
                
        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_path.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.marker_path) 
            self.marker_array_pub.publish(self.marker_path)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marker_RvizMarkers_path', anonymous=True)
    marker_RvizMarkers_path = RvizMarkers_array()
    try:
        marker_RvizMarkers_path.start()
    except rospy.ROSInterruptException:
        pass
