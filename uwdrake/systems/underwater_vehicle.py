
from pydrake.systems.framework import InputPort, Context, PortDataType, OutputPort

from pydrake.all import (
    TemplateSystem,
    LeafSystem_
)

from ..common import RigidBodyState
from ..physics.motion_model import MotionModel

import numpy as np

@TemplateSystem.define("UnderwaterVehicle_")
def UnderwaterVehicle_(T):
    '''
    This system is used to simulate the behavior of an underwater vehicle
    following a given motion model. 
    '''

    class Impl(LeafSystem_[T]):
        def _construct(self, motion_model:MotionModel, converter = None):
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
            Impl._construct(self, other.motion_model.copy(), converter=converter)


        def UpdateImuAccelerations(self, context, output):
            state_dot = self.CalcXDot(context)
            
            output.SetFromVector(state_dot[7:13])

        def CalcXDot(self, context:Context):
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

            # dynamic equation
            twist_dot = self.motion_model.CalculateAcceleration(twist, orientation, wrench_input)

            # kinematic equation
            pose_dot = self.motion_model.CalculateKinematics(twist, orientation)
            
            # set dertivatives
            state_dot = np.concatenate((pose_dot, twist_dot), axis=None)
            return state_dot


        def DoCalcTimeDerivatives(self, context:Context, derivatives):
            state_dot = self.CalcXDot(context)

            derivatives.get_mutable_vector().SetFromVector(state_dot)

        def GetWrenchInputPort(self) -> InputPort:
            return super().get_input_port(self.in_port_idx_wrench_)

        def GetStateOutputPort(self) -> OutputPort:
            return super().get_output_port(self.in_port_idx_wrench_)

    return Impl

UnderwaterVehicle = UnderwaterVehicle_[None]