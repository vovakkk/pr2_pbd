'''This module is a landmark descriptor for a tabletop
'''

class TableDescriptor:
    '''This class represents a descriptor for a tabletop
    landmark.
    '''

    def __init__(self, data):
        '''Initializes a descriptor given the specified data.
        It should extract needed features from the data and store
        them.

        Args:
            data (no type required): the data 
        '''

        '''This variable is the friendly name for the descriptor'''
        self.friendly_name = "table"

    def compare(self, descriptor):
        '''This method compares self to the descriptor and provides a number score
        that reflects how good of a descriptor it is. Higher number means
        better descriptor. Negative number if unacceptable.

        Args:
            descriptor (LandmarkDescriptor): the data to compare to
        '''
        return 1 if isinstance(descriptor, TableDescriptor) else -1