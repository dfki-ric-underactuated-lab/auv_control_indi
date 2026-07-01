##
# @file
# @brief Polar (radius/azimuth/altitude) orbit state and cartesian conversions.

import copy
import math

import numpy as np

from ..common import RigidBodyState, quat2rot


class OrbitControlStatus:
    ## @brief Orbit controller phase: aligning onto vs. running along the orbit.
    WAITING = 0,
    RUNNING = 1,
    ALIGNMENT = 2


class PolarControlState:
    ## @brief Holds the vehicle pose/velocity expressed in the orbit's polar frame.
    # Indices into the 3-element polar coordinate / velocity / error vectors.
    RADIUS = 0
    AZIMUTH = 1
    ALTITUDE = 2

    def __init__(self, plane_transform):
        '''@brief Initialize an empty state on the given orbit plane.'''
        self.status = OrbitControlStatus.WAITING

        self.plane_transform = plane_transform

        self.polar_coords = np.zeros((3))

        self.error_vector = np.zeros((3))

        self.azimuth_error = 0.0
        self.alignment_azimuth = 0.0 # Used as a reference during alignment phase.

        self.quat = np.zeros((4))
        self.quat_ref = np.zeros((4))

        self.body_velocity = np.zeros((6))
        self.polar_velocity = np.zeros((3))

        self.polar_axis = np.array([0.0, 0.0, 1.0])

        self.fix_altitude_relative_to_ground_plane = False

    def copy(self):
        '''@brief Shallow copy of the state.'''
        return copy.copy(self)

    def update(self, world_position, world_quaternion, body_velocity, world_center_point):
        '''@brief Recompute polar coordinates, frame rotations and polar velocity.

        @param world_position Vehicle position in world frame.
        @param world_quaternion Vehicle orientation [x, y, z, w].
        @param body_velocity Body twist [vlin(3), vang(3)].
        @param world_center_point Orbit centre in world frame.
        '''
        self.body_velocity = body_velocity
        self.quat = world_quaternion

        # Polar coordinates relative to the orbit centre.
        relative_position = world_position - world_center_point

        radius = math.sqrt(relative_position[0] ** 2 + relative_position[1] ** 2)
        azimuth = math.atan2(relative_position[1], relative_position[0])

        if self.fix_altitude_relative_to_ground_plane:
            # Ground plane at world z = 0: altitude is the absolute world z.
            altitude = world_position[2]
        else:
            altitude = relative_position[2]

        self.polar_coords = np.array([radius, azimuth, altitude])

        # Frame rotations (polar / body / reference -> world).
        self.polar_quat = np.concatenate((
            self.polar_axis * np.sin(self.azimuth() / 2.0),
            [np.cos(self.azimuth() / 2.0)]
        ))

        self.rot_polar_to_world = quat2rot(self.polar_quat)
        self.rot_body_to_world = quat2rot(self.quat)
        self.rot_ref_to_world = quat2rot(self.quat_ref)
        self.rot_polar_to_body = self.rot_body_to_world.T\
                                @ self.rot_polar_to_world

        # Body velocity expressed in the polar frame.
        self.polar_velocity = self.rot_polar_to_body.T.dot(self.body_velocity[0:3])


    def radius(self):
        '''@brief Current orbit radius.'''
        return self.polar_coords[PolarControlState.RADIUS]

    def azimuth(self):
        '''@brief Current azimuth angle [rad].'''
        return self.polar_coords[PolarControlState.AZIMUTH]

    def altitude(self):
        '''@brief Current altitude.'''
        return self.polar_coords[PolarControlState.ALTITUDE]

    def orientation_error(self):
        '''@brief Angle [rad] between the current and reference orientation.'''
        abs_inner_product = np.abs(np.dot(self.quat_ref, self.quat))

        if abs_inner_product >= 1.0:
            # Guard against arccos domain errors.
            return 0.0

        return 2 * np.arccos(abs_inner_product)

    def radius_error(self):
        '''@brief Radius component of the tracking error.'''
        return self.error_vector[PolarControlState.RADIUS]

    def altitude_error(self):
        '''@brief Altitude component of the tracking error.'''
        return self.error_vector[PolarControlState.ALTITUDE]

    def tangential_velocity(self):
        '''@brief Tangential (azimuthal) velocity component.'''
        return self.polar_velocity[PolarControlState.AZIMUTH]

    def radial_velocity(self):
        '''@brief Radial velocity component.'''
        return self.polar_velocity[PolarControlState.RADIUS]

    def altitude_velocity(self):
        '''@brief Altitude velocity component.'''
        return self.polar_velocity[PolarControlState.ALTITUDE]


def polar_to_cartesian_state(polar_state):
    '''@brief Convert a [radius, azimuth, altitude, quat(4), vlin(3), vang(3)] state
    into a Cartesian RigidBodyState-layout vector.
    '''
    radius = polar_state[PolarControlState.RADIUS]
    azimuth = polar_state[PolarControlState.AZIMUTH]

    cartesian_state = np.empty(RigidBodyState.SIZE)
    cartesian_state[RigidBodyState.POS_X] = radius * math.cos(azimuth)
    cartesian_state[RigidBodyState.POS_Y] = radius * math.sin(azimuth)
    cartesian_state[RigidBodyState.POS_Z] = polar_state[PolarControlState.ALTITUDE]
    # Orientation and velocities carry over unchanged.
    cartesian_state[RigidBodyState.ORI_X:] = polar_state[3:]

    return cartesian_state


def cartesian_to_polar_state(cartesian_state):
    '''@brief Inverse of polar_to_cartesian_state.'''
    x = cartesian_state[RigidBodyState.POS_X]
    y = cartesian_state[RigidBodyState.POS_Y]

    polar_state = np.empty(RigidBodyState.SIZE)
    polar_state[PolarControlState.RADIUS] = math.sqrt(x ** 2 + y ** 2)
    polar_state[PolarControlState.AZIMUTH] = math.atan2(y, x)
    polar_state[PolarControlState.ALTITUDE] = cartesian_state[RigidBodyState.POS_Z]
    # Orientation and velocities carry over unchanged.
    polar_state[3:] = cartesian_state[RigidBodyState.ORI_X:]

    return polar_state
