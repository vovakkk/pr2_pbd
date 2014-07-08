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
from RobotDescriptor import RobotDescriptor
from geometry_msgs.msg import Point, Pose, Quaternion

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal

class  World(object):
    '''This represents landmarks in the world.

    An instance of this class carries the information about the 
    objects.
    '''

    segmentation_service = rospy.get_param("/pr2_pbd_interaction/tabletop_segmentation_service")

    def __init__(self):
        self.landmarks = [Landmark(RobotDescriptor(None), 
                Pose())]

    def scan_landmarks(self, force_types=None):
        '''Scans for landmarks and stores/updates them in the world

        Args:
            force_types ([LandmarkDescriptor], optional): the scanned objects
                are forced to match of one of the descriptors provided
        '''
        #look down
        client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        client.wait_for_server()
        goal = PointHeadGoal()
        goal.target.point.x = 1
        goal.target.point.y = 0
        goal.target.point.z = 0.4
        goal.target.header.frame_id = "base_link"
        goal.pointing_frame = "high_def_frame"
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 1.0
        client.send_goal(goal)
        client.wait_for_result()

        rospy.wait_for_service(self.segmentation_service)
        try:
            get_segmentation = rospy.ServiceProxy(self.segmentation_service, TabletopSegmentation)
            resp = get_segmentation()
            self.landmarks = []
            # add the robot
            self.landmarks.append(Landmark(RobotDescriptor(None), 
                Pose()))

            # add the table
            self.landmarks.append(Landmark(TableDescriptor(resp.table), 
                resp.table.pose.pose))
            def make_landmark(clusterInd):
                cluster = clusterInd[0]
                ind = clusterInd[1]
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
                return Landmark(ObjectDescriptor(cluster, ind % 4),
                    Pose(Point((minX + maxX) / 2, (minY + maxY) / 2,
                        (minZ + maxZ) / 2), Quaternion(0, 0, 0, 1)), 
                    Point(maxX - minX, maxY - minY, maxZ - minZ))
            self.landmarks += [make_landmark(cluster)
                for cluster in zip(resp.clusters, range(0, len(resp.clusters)))]
        except rospy.ServiceException, e:
            print "Call to segmentation service failed: %s"%e

    def get_landmarks(self):
        '''Returns a list of Landmark that were found

        Returns: 
            type: [Landmark]
            list of landmarks
        '''
        return self.landmarks

    def find_landmark(self, descriptor):
        '''Finds the landmark that best fits the descriptor

        Args:
            descriptor: LandmarkDescriptor

        Returns:
            type: Landmark
            The best matched landmark
        '''
        maxVal = descriptor.compare(self.landmarks[0].descriptor)
        maxInd = 0
        for i in range(1, len(self.landmarks)):
            newVal = descriptor.compare(self.landmarks[i].descriptor)
            if (newVal > maxVal):
                maxVal = newVal
                maxInd = i

        return self.landmarks[maxInd]
