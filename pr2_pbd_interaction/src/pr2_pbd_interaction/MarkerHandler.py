'''This file handles the marker messages to be drawn
'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose, Quaternion

class MarkerHandler:

    def __init__(self):
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.prev_markers = 0

    def update(self, world):
        objs = world.get_landmarks()
        markers = MarkerArray()

        
        objId = 0
        for obj in objs:
            if obj.bounds != None:
                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = rospy.Time.now()
                m.ns = "pbd"
                m.id = objId
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose = obj.pose
                m.scale.x = obj.bounds.x
                m.scale.y = obj.bounds.y
                m.scale.z = obj.bounds.z
                m.color.a = 1.0
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                # rospy.loginfo(m)
                markers.markers.append(m)
                objId += 1
        for i in range(objId, self.prev_markers):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = "pbd"
            m.id = i
            m.action = Marker.DELETE
            markers.markers.append(m)
        self.marker_pub.publish(markers)
        self.prev_markers = len(objs)
