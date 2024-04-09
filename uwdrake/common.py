import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class RigidBodyState:
    SIZE = 13
    POS_X = 0
    POS_Y = 1
    POS_Z = 2
    ORI_X = 3
    ORI_Y = 4
    ORI_Z = 5
    ORI_W = 6
    VLIN_X = 7
    VLIN_Y = 8
    VLIN_Z = 9
    VANG_X = 10
    VANG_Y = 11
    VANG_Z = 12

def skew(vector):
    """
    this function returns a numpy array with the skew symmetric cross product matrix for vector.
    the skew symmetric cross product matrix is defined such that
    np.cross(a, b) = np.dot(skew(a), b)

    :param vector: An array like vector to create the skew symmetric cross product matrix for
    :return: A numpy array of the skew symmetric cross product vector
    """

    return np.array([[0, -vector[2], vector[1]], 
                    [vector[2], 0, -vector[0]], 
                    [-vector[1], vector[0], 0]])

def quat2rot(quat):
    """ Calculate Rotation matrix for quaternion
    """
    x, y, z, w = quat

    rot_ib = np.array([
        [2*(w**2+x**2)-1, 2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),     2*(w**2+y**2)-1, 2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z + w*x),   2*(w**2+z**2)-1]
    ])

    return rot_ib


