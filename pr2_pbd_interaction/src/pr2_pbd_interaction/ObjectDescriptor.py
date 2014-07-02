'''This module is a descroptor of an object
'''

import operator
import math

class ObjectDescriptor:
    '''This class handles the descriptor of an object
    '''

    def getVals(self, data):
        '''This helper method returns an array containing the max, min,
        and average radii
        '''
        centerPt = map(lambda x: x / len(data.points), 
            reduce(lambda acc, pt: [acc[0] + pt.x, acc[1] + pt.y,
            acc[2] + pt.z], data.points, [0,0,0]))

        def rad(pt):
            return math.sqrt((pt.x - centerPt[0]) * (pt.x - centerPt[0]) +
                (pt.y - centerPt[1]) * (pt.y - centerPt[1]) +
                (pt.z - centerPt[2]) * (pt.z - centerPt[2]))

        maxRad = rad(data.points[0])
        minRad = rad(data.points[0])
        avgRad = 0

        for pt in data.points:
            maxRad = max(maxRad, rad(pt))
            minRad = min(minRad, rad(pt))
            avgRad += rad(pt) / len(data.points)
        return [maxRad, minRad, avgRad]

    def __init__(self, data):
        '''Initializes a descriptor given the specified data.
        It should extract needed features from the data and store
        them.

        Args:
            data (sensor_msgs.PointCloud): the data
        '''
        self.descVals = self.getVals(data)

    def compare(self, data):
        '''This method compares the data and provides a number score
        that reflects how good of a descriptor it is. Higher number means
        better descriptor.

        Args:
            data (sensor_msgs.PointCloud): the data to compare to
        '''
        return -reduce(operator.add, map(abs, map(operator.sub, 
            zip(self.getVals(data), self.descVals))), 0)