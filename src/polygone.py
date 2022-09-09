#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations

class polygone(object):
    def __init__(self):
        self.marker_polygone_pub = rospy.Publisher('polygone', Marker, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker(index=0,z_val=0)

    def init_marker(self, index=0, z_val=0):
        self.marker_polygone = Marker()
        self.marker_polygone.header.frame_id = "map"
        self.marker_polygone.header.stamp    = rospy.get_rostime()
        self.marker_polygone.ns = "Line"
        self.marker_polygone.id = index
        self.marker_polygone.type = Marker().LINE_STRIP
        self.marker_polygone.action = Marker().ADD
    
        path = []
        path.append( Point(2.5,0,0) )
        path.append( Point(0,2.5,0) )
        path.append( Point(-2.5,0,0) )
        path.append( Point(0,-2.5,0) )
        path.append( Point(2.5,-2.5,0) )
        path.append( Point(2.5,0,0) )
        
        self.marker_polygone.points[:] = [] # clear
        self.marker_polygone.colors[:] = []
        
        path_color = ColorRGBA()
        path_color.a = 1
        path_color.r = 0
        path_color.g = 0
        path_color.b = 1
        
        self.marker_polygone.scale.x = 0.05
        
        for i in range(1, len(path)):
            # Start of segment is previous point
            self.marker_polygone.points.append(path[i-1])
            self.marker_polygone.colors.append(path_color)
            # End of segment is current point
            self.marker_polygone.points.append(path[i])
            self.marker_polygone.colors.append(path_color)
                
        # If we want it forever, 0, otherwise seconds before desapearing
        self.marker_polygone.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.marker_polygone) 
            self.marker_polygone_pub.publish(self.marker_polygone)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('polygone_path', anonymous=True)
    marker_polygone_path = polygone()
    try:
        marker_polygone_path.start()
    except rospy.ROSInterruptException:
        pass
