##
# @file
# @brief Orbit-inspection guidance: tracks a cylindrical orbit around a target.

from pydrake.systems.framework import BasicVector, LeafSystem, PortDataType

import numpy as np
import math

from ..common import RigidBodyState, calculate_lookat_orientation
from .polar_control_state import PolarControlState, OrbitControlStatus


class OrbitControlCommand:
    ## @brief Orbit setpoint (radius/azimuth/altitude, motion mode) and acceptance
    #  tolerances used to switch between alignment and orbiting.
    def __init__(self):
        self.radius              = 0.0
        self.altitude            = 0.0
        self.azimuth             = 0.0
        self.continuous_motion   = True
        self.tangential_velocity = 0.3

        self.target_point        = np.zeros(3)

        # Tolerances for switching from alignment to orbiting.
        self.allowed_orientation_error = 30.0            # degrees
        self.allowed_radius_error      = 0.3             # meters
        self.allowed_altitude_error    = 0.3             # meters
        self.acceptance_azimuth        = 3.0 * math.pi / 180.0  # rad

class PolarController(LeafSystem):
    ## @brief Outputs a body-twist command that flies the vehicle onto and along a
    #  cylindrical orbit while looking at the target.
    #
    #  Ports: in "state"(13) -> out "control_twist"(6) plus reference/telemetry
    #  ports ("ori_ref", "polar_ref", "polar", "polar_velocity").
    def __init__(self, Kp, vlin_max, vang_max, observer_rotmat):
        '''@brief Declare ports and store gains, limits and the observer rotation.

        @param Kp Diagonal polar gain matrix (radius, azimuth, altitude, + attitude).
        @param observer_rotmat Rotation from the observer/camera frame to the base link.
        '''
        super().__init__()

        self.twist_output_port = self.DeclareVectorOutputPort(
            "control_twist",
            BasicVector(6),
            self.Update)

        self.state_input_port = self.DeclareInputPort("state",
            PortDataType.kVectorValued,
            size=RigidBodyState.SIZE)

        self.ori_ref_output_port = self.DeclareVectorOutputPort(
            "ori_ref",
            BasicVector(4),
            self.GetReferenceOrientation)

        self.polar_ref_output_port = self.DeclareVectorOutputPort(
            "polar_ref",
            BasicVector(3),
            self.GetReferencePolarCoordinates)

        self.polar_output_port = self.DeclareVectorOutputPort(
            "polar",
            BasicVector(3),
            self.GetPolarCoordinates)

        self.polar_velocity_output_port = self.DeclareVectorOutputPort(
            "polar_velocity",
            BasicVector(3),
            self.GetPolarVelocity)

        self.observer_rotmat = observer_rotmat
        self.gains = Kp
        self.max_angular_velocity = vang_max
        self.max_linear_velocity  = vlin_max

        self.command = OrbitControlCommand()
        self.command.radius = 2.0
        self.command.azimuth = 0.0
        self.command.altitude = 3.0
        self.command.continuous_motion = True
        self.command.tangential_velocity = 0.1

        self.world_target_position = np.array([0.0, 0.0, -5.0])

        self.polar_state = PolarControlState(np.array([0,0,0,1]))
        self.polar_state.status = OrbitControlStatus.RUNNING

        self.error_list = []

    def GetReferenceOrientation(self, context, output):
        '''@brief Output the current reference (look-at) orientation.'''
        output.SetFromVector(self.polar_state.quat_ref)

    def GetReferencePolarCoordinates(self, context, output):
        '''@brief Output the reference polar coordinates (setpoint).'''
        output.SetFromVector(self.polar_state.error_vector + self.polar_state.polar_coords)

    def GetPolarCoordinates(self, context, output):
        '''@brief Output the current polar coordinates.'''
        output.SetFromVector(self.polar_state.polar_coords)

    def GetPolarVelocity(self, context, output):
        '''@brief Output the current polar-frame velocity.'''
        output.SetFromVector(self.polar_state.polar_velocity)

    def Update(self, context, output):
        '''@brief Update the orbit state machine and emit the body-twist command.'''
        x = self.state_input_port.Eval(context)

        polar_state = self.polar_state
        command = self.command

        world_observer_position = x[0:3]

        polar_state.update(world_observer_position, x[3:7],
                           x[7:13], self.world_target_position)

        radius_error = command.radius - polar_state.radius()
        altitude_error = command.altitude - polar_state.altitude()

        # Reference orientation: look at the centre point.
        polar_state.quat_ref = calculate_lookat_orientation(
            target_position   = self.world_target_position,
            observer_position = world_observer_position,
            observer_rotmat   = self.observer_rotmat
        )

        if abs(polar_state.orientation_error()) > command.allowed_orientation_error or\
            abs(radius_error) > command.allowed_radius_error or\
            abs(altitude_error) > command.allowed_altitude_error:
            # Alignment: fly onto the orbit, holding the azimuth at which alignment started.
            if polar_state.status != OrbitControlStatus.ALIGNMENT:
                polar_state.alignment_azimuth = polar_state.azimuth()
                polar_state.status = OrbitControlStatus.ALIGNMENT
            azimuth_error = (polar_state.alignment_azimuth - polar_state.azimuth()
                             + math.pi) % (2 * math.pi) - math.pi
            tangential_velocity = self.gains[1, 1] * azimuth_error
        elif command.continuous_motion:
            # Orbit continuously; tangential-velocity sign sets the direction.
            polar_state.status = OrbitControlStatus.RUNNING
            azimuth_error = 0.0
            tangential_velocity = command.tangential_velocity
        else:
            # Drive to and hold the fixed azimuth setpoint.
            azimuth_error = (command.azimuth - polar_state.azimuth()
                             + math.pi) % (2 * math.pi) - math.pi
            tangential_velocity = self.gains[1, 1] * azimuth_error
            polar_state.status = (OrbitControlStatus.WAITING
                                  if abs(azimuth_error) < command.acceptance_azimuth
                                  else OrbitControlStatus.RUNNING)

        polar_state.azimuth_error = azimuth_error
        polar_state.error_vector = np.array([radius_error, azimuth_error, altitude_error])

        twist_cmd = self.control(polar_state, tangential_velocity,
                                 abs(command.tangential_velocity), self.gains,
                                 self.max_linear_velocity, self.max_angular_velocity)

        output.SetFromVector(twist_cmd)

    def control(self, polar_state:PolarControlState, tangential_velocity, max_tangential_velocity, polar_pose_gains, vlin_max, vang_max):
        '''@brief Polar pose control law -> velocity-limited body twist.'''
        # Linear law on radius/altitude; azimuth velocity is commanded directly.
        polar_vel_cmd = polar_pose_gains[0:3,0:3].dot(polar_state.error_vector)
        polar_vel_cmd[1] = np.clip(tangential_velocity,
                                   -max_tangential_velocity,
                                   +max_tangential_velocity)

        vlin_cmd = polar_state.rot_polar_to_body.dot(polar_vel_cmd)

        vang_control     = self.calculate_angular_control(polar_state, polar_pose_gains)
        vang_feedforward = self.calculate_angular_feedforward(polar_state, polar_vel_cmd)
        vang_cmd         = vang_control + vang_feedforward

        vlin_cmd = np.clip(vlin_cmd, -vlin_max, vlin_max)
        vang_cmd = np.clip(vang_cmd, -vang_max, vang_max)

        return np.concatenate((vlin_cmd, vang_cmd))

    def calculate_angular_control(self, polar_state:PolarControlState, polar_pose_gains):
        '''@brief Attitude control velocity (Chaturvedi et al. 2011, rotation-matrix law).'''
        a1 = a2 = a3 = 1
        e1 = np.array([1, 0, 0])
        e2 = np.array([0, 1, 0])
        e3 = np.array([0, 0, 1])

        rot_err = np.cross(a1*e1, polar_state.rot_ref_to_world.T.dot(
                    polar_state.rot_body_to_world.dot(e1)))\
                + np.cross(a2*e2, polar_state.rot_ref_to_world.T.dot(
                    polar_state.rot_body_to_world.dot(e2)))\
                + np.cross(a3*e3, polar_state.rot_ref_to_world.T.dot(
                    polar_state.rot_body_to_world.dot(e3)))

        return - polar_pose_gains[3:6,3:6].dot(rot_err)

    def calculate_angular_feedforward(self, polar_state:PolarControlState, polar_vel_cmd):
        '''@brief Angular-velocity feed-forward from the tangential velocity setpoint.

        Uses the rigid-particle relation omega = (r x v) / |r|^2 for a cylindrical
        orbit (see https://en.wikipedia.org/wiki/Angular_velocity).
        '''
        # Normalized radial direction (radius cancels out below).
        r_vector = np.array([1.0, 0.0, 0.0])

        # Only the tangential velocity matters for a cylindrical (not spherical) orbit.
        tangential_velocity = np.array([0, polar_state.polar_velocity[1], 0])

        angular_velocity = np.cross(r_vector, tangential_velocity)\
                         / (polar_state.radius() + 0.0000001)

        # World frame -> body frame.
        return polar_state.rot_body_to_world.T.dot(angular_velocity)
