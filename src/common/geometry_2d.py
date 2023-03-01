import numpy as np

class AbFrame:
    '''
    AbFrame object implements functionality of a 2D frame of coordinates in which A=(0,0), B=(0,1)
    '''
    def __init__(self, a_position, b_position):
        self.a_position = a_position
        self.b_position = b_position

        self.scale = np.norm(a_position-b_position)
        self.rotation_matrix =
        self.translation = a_position

    def transform_point_ab_to_world(self, x, y):
        pass

    def transform_point_world_to_ab(self, x, y):
        pass

    def transform_pose_ab_to_world(self, orientation, x, y):
        pass

    def transform_pose_world_to_ab(self, orientation, x, y):
        pass