##
# @file
# @brief Shared state layouts and quaternion / frame-transform helpers.

import numpy as np
import math
import transforms3d
from scipy.spatial.transform import Rotation as R

class RigidBodyState:
    ## @brief Index layout of the 13-element rigid-body state
    #  [pos(3), quat xyzw(4), vlin(3), vang(3)].
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

    @staticmethod
    def position(state):
        '''@brief World position [x, y, z] from a full state vector.'''
        return np.asarray(state)[RigidBodyState.POS_X:RigidBodyState.POS_Z + 1]

    @staticmethod
    def quaternion(state):
        '''@brief Orientation quaternion [x, y, z, w] from a full state vector.'''
        return np.asarray(state)[RigidBodyState.ORI_X:RigidBodyState.ORI_W + 1]

    @staticmethod
    def pose(state):
        '''@brief Pose [x, y, z, qx, qy, qz, qw] from a full state vector.'''
        return np.asarray(state)[RigidBodyState.POS_X:RigidBodyState.ORI_W + 1]

class TwistState:
    ## @brief Index layout of a 6-element body twist [vlin(3), vang(3)].
    SIZE = 6
    VLIN_X = 0
    VLIN_Y = 1
    VLIN_Z = 2
    VANG_X = 3
    VANG_Y = 4
    VANG_Z = 5

class UVDegreesOfFreedom:
    ## @brief Index layout of the 6 vehicle DOFs.
    SIZE = 6
    SURGE = 0
    SWAY = 1
    HEAVE = 2
    ROLL = 3
    PITCH = 4
    YAW = 5


class Wrench(UVDegreesOfFreedom):
    ## @brief A 6-DOF wrench [force(3), torque(3)]; shares the DOF layout.
    pass

def ned_transform_rbstate(state):
    '''@brief Transform a full RigidBodyState from ENU to NED frame.'''
    pos = np.array([state[RigidBodyState.POS_X],
                    state[RigidBodyState.POS_Y],
                    state[RigidBodyState.POS_Z]])
    ori = np.array([state[RigidBodyState.ORI_X],
                    state[RigidBodyState.ORI_Y],
                    state[RigidBodyState.ORI_Z],
                    state[RigidBodyState.ORI_W]])
    vlin = np.array([state[RigidBodyState.VLIN_X],
                    state[RigidBodyState.VLIN_Y],
                    state[RigidBodyState.VLIN_Z]])
    vang = np.array([state[RigidBodyState.VANG_X],
                    state[RigidBodyState.VANG_Y],
                    state[RigidBodyState.VANG_Z]])

    return np.concatenate((ned_transform_vec3(pos),
                           vehicle_transform_quaternion(ori),
                           vehicle_transform_vec3(vlin),
                           vehicle_transform_vec3(vang)))

def ned_transform_vec3(vec):
    '''@brief Transform a 3-vector from ENU to NED frame.'''
    ned_transform = R.from_euler("xyz", [math.pi, 0, math.pi/2.0])

    return ned_transform.apply(vec)

def ned_transform_quaternion(xyzw):
    '''@brief Transform a quaternion (xyzw) from ENU to NED frame.'''
    ned_transform = R.from_euler("xyz", [math.pi, 0, math.pi/2.0])

    q = R.from_quat(xyzw)
    q = ned_transform * q

    return q.as_quat()

def vehicle_transform_vec3(data_in):
    '''@brief Transform a 3-vector from AUV z-up to z-down (negate y and z).'''
    return np.array([data_in[0], -data_in[1], -data_in[2]])

def vehicle_transform_quaternion(xyzw):
    '''@brief Transform a quaternion (xyzw) from AUV z-up to z-down.'''
    vehicle_transform = R.from_euler("xyz", [math.pi, 0, 0])

    q = R.from_quat(xyzw)
    q = q * vehicle_transform

    return q.as_quat()


def skew(vector):
    '''@brief Skew-symmetric cross-product matrix S such that cross(a, b) = S(a) @ b.'''
    return np.array([[0, -vector[2], vector[1]],
                    [vector[2], 0, -vector[0]],
                    [-vector[1], vector[0], 0]])

def quat2rot(quat):
    '''@brief Rotation matrix (body->world) for a quaternion given as [x, y, z, w].'''
    x, y, z, w = quat

    rot_ib = np.array([
        [2*(w**2+x**2)-1, 2*(x*y-w*z),     2*(x*z+w*y)],
        [2*(x*y+w*z),     2*(w**2+y**2)-1, 2*(y*z-w*x)],
        [2*(x*z-w*y),     2*(y*z + w*x),   2*(w**2+z**2)-1]
    ])

    return rot_ib


def rpy2quat(rpy):
    '''@brief Roll-pitch-yaw (rad) to a normalized quaternion [x, y, z, w].'''
    rpy = unify_rpy(rpy)
    wxyz = transforms3d.euler.euler2quat(rpy[0], rpy[1], rpy[2], 'rxyz')
    xyzw = np.hstack((wxyz[1:4], wxyz[0]))
    return unify_quaternion(xyzw)


def quat2rpy(quat, in_degrees=False):
    '''@brief Quaternion [x, y, z, w] to roll-pitch-yaw (xyz order).'''
    rot = R.from_quat(quat)
    return np.array(rot.as_euler('xyz', degrees=in_degrees))


def unify_quaternion(quat):
    '''@brief Return the unit-norm version of a quaternion.'''
    return quat / np.linalg.norm(quat)


def unify_rpy(rpy):
    '''@brief Wrap each roll/pitch/yaw angle into [0, 2*pi).'''
    return [alpha % (2 * np.pi) for alpha in rpy]

def quat2rpy_array(seq_out, array_in, degrees):
    '''@brief Convert an (N, 4) array of quaternions to (N, 3) Euler angles.

    @param seq_out Euler sequence, e.g. "xyz".
    @param array_in (N, 4) quaternions [x, y, z, w].
    @param degrees Return degrees if True, radians otherwise.
    '''
    array_out = np.empty((array_in.shape[0], 3))

    for i in range(array_in.shape[0]):
        rot = R.from_quat(array_in[i, :])
        array_out[i, :] = rot.as_euler(seq_out, degrees=degrees)

    return array_out

def euler_to_quat_array(seq_in, x_in, y_in, z_in, degrees):
    '''@brief Convert per-axis Euler-angle arrays to quaternion component arrays.

    @return Tuple of arrays (x, y, z, w).
    '''
    x = np.empty_like(x_in)
    y = np.empty_like(x_in)
    z = np.empty_like(x_in)
    w = np.empty_like(x_in)

    for i in range(len(x)):
        rot = R.from_euler(seq_in, [x_in[i], y_in[i], z_in[i]], degrees=degrees)
        x[i], y[i], z[i], w[i] = rot.as_quat()

    return x, y, z, w

def quaternion_product(p_x, p_y, p_z, p_w, q_x, q_y, q_z, q_w):
    '''@brief Hamilton product t = p * q of two quaternions (component-wise args).'''
    t_x = p_w * q_x + p_x * q_w + p_y * q_z - p_z * q_y
    t_y = p_w * q_y - p_x * q_z + p_y * q_w + p_z * q_x
    t_z = p_w * q_z + p_x * q_y - p_y * q_x + p_z * q_w
    t_w = p_w * q_w - p_x * q_x - p_y * q_y - p_z * q_z

    return np.array([t_x, t_y, t_z, t_w])

def quaternion_conjugate(q):
    '''@brief Conjugate of a quaternion [x, y, z, w].'''
    return np.array([-q[0], -q[1], -q[2], q[3]])

def calculate_lookat_orientation(target_position, observer_position, observer_rotmat):
    '''@brief Quaternion orienting the observer to look from its position at a target.

    @param observer_rotmat Rotation from the observer frame to the base link.
    @return Look-at orientation as a quaternion [x, y, z, w].
    '''
    world_z = np.array(([0.0, 0.0, 1.0]))
    relative_distance = target_position - observer_position
    normal_vector = relative_distance / np.linalg.norm(relative_distance)

    tangent_vector = np.cross(normal_vector, world_z)
    # Degenerate when the look direction is colinear with world z; pick a fallback.
    if np.linalg.norm(tangent_vector) < 0.01:
        tangent_vector = np.array([-1.0, 0.0, 0.0])

    tangent_vector = tangent_vector / np.linalg.norm(tangent_vector)
    binormal_vector = np.cross(normal_vector, tangent_vector)

    rot = np.column_stack((tangent_vector, binormal_vector, normal_vector))
    rot = rot @ observer_rotmat.T

    return R.from_matrix(rot).as_quat()

def normalize_angle_radians(angle):
    '''@brief Wrap an angle into (-pi, pi].'''
    return (angle + math.pi) % (2 * math.pi) - math.pi
