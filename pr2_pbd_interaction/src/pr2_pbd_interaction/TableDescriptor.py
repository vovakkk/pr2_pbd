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
        pass

    def compare(self, data):
        '''This method compares the data and provides a number score
        that reflects how good of a descriptor it is. Higher number means
        better descriptor.

        Args:
            data (unknown type): the data to compare to
        '''
        return 0