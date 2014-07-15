'''This file is an "interface" used just for documentation.
'''

class LandmarkDescriptor:
    '''All descriptors should include the methods specified here
    '''

    '''This variable is the friendly name for the descriptor'''
    friendly_name = ""

    def __init__(self, data):
        '''Initializes a descriptor given the specified data.
        It should extract needed features from the data and store
        them.

        Args:
            data (unknown type): the data 
        '''
        pass

    def compare(self, descriptor):
        '''This method compares self to the descriptor and provides a number score
        that reflects how good of a descriptor it is. Higher number means
        better descriptor. Negative number if unacceptable.

        Args:
            descriptor (LandmarkDescriptor): the data to compare to
        '''
        pass