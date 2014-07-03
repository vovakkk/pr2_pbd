'''This module is a descroptor of an object
'''

import operator
import math

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
        return (1 / (0.00001 + reduce(operator.add, map(abs, map(lambda (a, b): a - b, 
            zip(descriptor.descVals, self.descVals))), 0))
            if isinstance(descriptor, ObjectDescriptor) else -1)