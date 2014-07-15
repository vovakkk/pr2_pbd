'''This module is a descroptor of an object
'''

import operator
import math
import numpy
from numpy import linalg
import rospy
from functools import partial

class ObjectDescriptor:
    '''This class handles the descriptor of an object
    '''

    RED = 0
    GREEN = 1
    BLUE = 2
    PINK = 3

    '''This variable is the friendly name for the descriptor'''
    friendly_name = "object"

    def getVals(self, data):
        '''This helper method returns an array containing the max, min,
        and average radii
        '''
        # convert to numpy array
        npPts = map(lambda pt: numpy.array([pt.x, pt.y, pt.z]), data.points)

        # get covariance matrix
        covMat = numpy.cov(numpy.array(npPts).T)

        # find eigen values/vectors
        eigVals, eigVects = linalg.eig(covMat)

        # find biggest eigen value (major vector)
        mainDir = eigVects[0]
        mainVal = eigVals[0]
        for i in range(1, len(eigVals)):
            if (eigVals[i] > mainVal):
                mainVal = eigVals[i]
                mainDir = eigVects[i]

        # normalize main direction
        mainDir = mainDir / linalg.norm(mainDir)


        # get 'length'
        minDot = numpy.dot(mainDir, npPts[0])
        maxDot = numpy.dot(mainDir, npPts[0])
        for i in range(1, len(npPts)):
            minDot = min(minDot, numpy.dot(mainDir, npPts[i]))
            maxDot = max(maxDot, numpy.dot(mainDir, npPts[i]))
        objLength = maxDot - minDot

        #group several segments of points together
        NUM_INTS = 5

        ptGroups = map(lambda (x): [], range(0, NUM_INTS))

        for pt in npPts:
            majDist = numpy.dot(mainDir, pt)
            groupInd = min(int((majDist - minDot) / objLength * NUM_INTS),
                    NUM_INTS - 1)
            ptGroups[groupInd].append(pt)# - majDist * mainDir)

        #get nullspace of mainDir
        diffVects = [numpy.array([1,0,0]), 
            numpy.array([0,1,0])]
        dotVals = map(partial(numpy.dot, mainDir), diffVects)
        npVect = diffVects[1 if dotVals[0] > dotVals[1] else 0]
        axis1 = numpy.cross(npVect, mainDir)
        axis2 = numpy.cross(axis1, mainDir)
        [axis1, axis2] = map(lambda (x): x / linalg.norm(x), [axis1, axis2])

        NUM_COMP_AXESES = 20
        axeses = map(lambda (i): math.sin(i * 2 * math.pi / NUM_COMP_AXESES) * axis1 +
                math.cos(i * 2 * math.pi / NUM_COMP_AXESES) * axis2, range(0, NUM_COMP_AXESES))

        def getProjs(pt):
            return map(partial(numpy.dot, pt), axeses)

        #find the diameter at each segment
        maxDs = [0] * NUM_INTS
        for ind, ptGroup in enumerate(ptGroups):
            minProjs = maxProjs = getProjs(ptGroup[0])
            for i in range(1, len(ptGroup)):
                ithProj = getProjs(ptGroup[i])
                minProjs = map(min, zip(minProjs, ithProj))
                maxProjs = map(max, zip(minProjs, ithProj))
            maxDs[ind] = max(map(lambda (a, b): b - a, zip(minProjs, maxProjs)))

        rospy.loginfo([objLength] + maxDs)

        return [objLength] + maxDs


        # centerPt = map(lambda x: x / len(data.points), 
        #     reduce(lambda acc, pt: [acc[0] + pt.x, acc[1] + pt.y,
        #     acc[2] + pt.z], data.points, [0,0,0]))

        # def rad(pt):
        #     return math.sqrt((pt.x - centerPt[0]) * (pt.x - centerPt[0]) +
        #         (pt.y - centerPt[1]) * (pt.y - centerPt[1]) +
        #         (pt.z - centerPt[2]) * (pt.z - centerPt[2]))

        # maxRad = rad(data.points[0])
        # minRad = rad(data.points[0])
        # avgRad = 0

        # for pt in data.points:
        #     maxRad = max(maxRad, rad(pt))
        #     minRad = min(minRad, rad(pt))
        #     avgRad += rad(pt) / len(data.points)
        # return [maxRad, minRad, avgRad]

    def __init__(self, data, color):
        '''Initializes a descriptor given the specified data.
        It should extract needed features from the data and store
        them.

        Args:
            data (sensor_msgs.PointCloud): the data
        '''
        self.descVals = self.getVals(data)
        self.color = color
        self.friendly_name = ["red", "green", "blue", "pink"][self.color] + " object"

    def compare(self, descriptor):
        '''This method compares self to the descriptor and provides a number score
        that reflects how good of a descriptor it is. Higher number means
        better descriptor. Negative number if unacceptable.

        Args:
            descriptor (LandmarkDescriptor): the data to compare to
        '''
        comp = lambda (ar): reduce(operator.add, map(abs, map(lambda (a, b): a - b, 
            zip(ar, self.descVals[1:]))), 0)
        return ((100 - abs(descriptor.descVals[0] - self.descVals[0]) * 30 +
            min(comp(descriptor.descVals[1:]),comp(descriptor.descVals[-2::-1])))
            if isinstance(descriptor, ObjectDescriptor) else -1)