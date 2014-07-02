'''Representation of a landmark in the world

This module stores a landmark.
'''

class Landmark:
    '''Python representation of a landmark
    '''
    def __init__(self, descriptor, pose, bounds=None):
        self.descriptor = descriptor
        self.pose = pose
        self.bounds = bounds