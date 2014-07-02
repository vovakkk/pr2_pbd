'''Representation of landmarks in the world

This module handles scanning for objects in the world and 
storing them.
'''

# These ROS imports must come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')

import rospy
from tabletop_object_detector.srv import TabletopSegmentation

from Landmark import Landmark
from TableDescriptor import TableDescriptor
from ObjectDescriptor import ObjectDescriptor
from geometry_msgs.msg import Point, Pose, Quaternion

class  World(object):
    '''This represents landmarks in the world.

    An instance of this class carries the information about the 
    objects.
    '''

    segmentation_service = rospy.get_param("/pr2_pbd_interaction/tabletop_segmentation_service")

    def __init__(self):
        pass

    def scan_landmarks(self, force_types=None):
        '''Scans for landmarks and stores/updates them in the world

        Args:
            force_types ([LandmarkDescriptor], optional): the scanned objects
                are forced to match of one of the descriptors provided
        '''
        rospy.wait_for_service(self.segmentation_service)
        try:
            get_segmentation = rospy.ServiceProxy(self.segmentation_service, TabletopSegmentation)
            resp = get_segmentation()
            self.landmarks = []
            self.landmarks.append(Landmark(TableDescriptor(resp.table), 
                resp.table.pose.pose))
            def make_landmark(cluster):
                points = cluster.points
                if (len(points) == 0):
                    return Point(0, 0, 0)
                [minX, maxX, minY, maxY, minZ, maxZ] = [
                    points[0].x, points[0].x, points[0].y, points[0].y,
                    points[0].z, points[0].z]
                for pt in points:
                    minX = min(minX, pt.x)
                    minY = min(minY, pt.y)
                    minZ = min(minZ, pt.z)
                    maxX = max(maxX, pt.x)
                    maxY = max(maxY, pt.y)
                    maxZ = max(maxZ, pt.z)
                return Landmark(ObjectDescriptor(cluster), 
                    Pose(Point((minX + maxX) / 2, (minY + maxY) / 2,
                        (minZ + maxZ) / 2), Quaternion(0, 0, 0, 1)), 
                    Point(maxX - minX, maxY - minY, maxZ - minZ))
            self.landmarks += [make_landmark(cluster)
                for cluster in resp.clusters]
        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s"%e

    def get_landmarks(self):
        '''Returns a list of Landmark that were found

        Returns: 
            type: [Landmark]
            list of landmarks
        '''
        return self.landmarks
