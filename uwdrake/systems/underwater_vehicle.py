##
# @file
# @brief Drake plant integrating the 6-DOF vehicle dynamics from a wrench input.

from pydrake.systems.framework import Context, PortDataType

from pydrake.all import (
    TemplateSystem,
    LeafSystem_
)

from ..common import RigidBodyState
from ..physics.motion_model import MotionModel

import numpy as np

## @brief Scalar-templated LeafSystem simulating an underwater vehicle.
#  Ports: in "wrench" (6) -> out "state" (13, continuous state) and
#  "imu_accelerations" (6).
@TemplateSystem.define("UnderwaterVehicle_")
def UnderwaterVehicle_(T):

    class Impl(LeafSystem_[T]):
        def _construct(self, motion_model:MotionModel, converter = None):
            '''@brief Declare the state and ports for the given motion model.'''
            LeafSystem_[T].__init__(self, converter)

            self.motion_model = motion_model

            state_index = self.DeclareContinuousState(RigidBodyState.SIZE)
            self.state_output_port = self.DeclareStateOutputPort("state", state_index)
            self.wrench_input_port = self.DeclareInputPort("wrench",
                                                           PortDataType.kVectorValued, size=6)

            self.imu_acc_out_port = self.DeclareVectorOutputPort(
                "imu_accelerations",
                6,
                self.UpdateImuAccelerations
            )

        def _construct_copy(self, other, converter=None):
            '''@brief Scalar-conversion copy constructor (Drake TemplateSystem hook).'''
            Impl._construct(self, other.motion_model.copy(), converter=converter)

        def UpdateImuAccelerations(self, context, output):
            '''@brief Output the body linear/angular accelerations (IMU model).'''
            state_dot = self.CalcXDot(context)
            output.SetFromVector(state_dot[7:13])

        def CalcXDot(self, context:Context):
            '''@brief Full state derivative from the dynamic + kinematic equations.'''
            wrench_input = self.wrench_input_port.Eval(context)

            ori_x  = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.ORI_X)
            ori_y  = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.ORI_Y)
            ori_z  = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.ORI_Z)
            ori_w  = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.ORI_W)
            vlin_x = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VLIN_X)
            vlin_y = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VLIN_Y)
            vlin_z = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VLIN_Z)
            vang_x = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VANG_X)
            vang_y = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VANG_Y)
            vang_z = context.get_continuous_state_vector().GetAtIndex(RigidBodyState.VANG_Z)

            twist = np.array([vlin_x, vlin_y, vlin_z, vang_x, vang_y, vang_z])
            orientation = np.array([ori_x, ori_y, ori_z, ori_w])

            twist_dot = self.motion_model.calculate_acceleration(twist, orientation, wrench_input)
            pose_dot = self.motion_model.calculate_kinematics(twist, orientation)

            return np.concatenate((pose_dot, twist_dot), axis=None)

        def DoCalcTimeDerivatives(self, context:Context, derivatives):
            '''@brief Drake time-derivative callback.'''
            state_dot = self.CalcXDot(context)
            derivatives.get_mutable_vector().SetFromVector(state_dot)

    return Impl

## @brief Concrete (double) UnderwaterVehicle system.
UnderwaterVehicle = UnderwaterVehicle_[None]
