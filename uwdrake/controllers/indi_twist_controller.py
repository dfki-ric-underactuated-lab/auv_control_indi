##
# @file
# @brief Incremental Nonlinear Dynamic Inversion (INDI) body-twist controller.

from pydrake.systems.framework import BasicVector, LeafSystem

from pydrake.all import (
    PortDataType
)

import numpy as np
from ..physics.motion_model import MotionModel
from ..controllers.qp_control_allocation import QpControlAllocation
from ..common import RigidBodyState

class IndiTwistController(LeafSystem):
    ## @brief Tracks a body-twist reference using measured acceleration (IMU) and
    #  RPM feedback, so it needs only the mass matrix, not the full model.
    #
    #  Ports: in "twist_ref"(6), "state"(13), "imu"(6), "rpm_values"(n)
    #         -> out "rpm_setpoint"(n).
    def __init__(self, time_step, motion_model:MotionModel, Kp, Kd):
        '''@brief Declare ports and store the PD gains and control allocator.

        @param Kp Proportional twist-error gain matrix.
        @param Kd Derivative (acceleration) gain matrix.
        '''
        super().__init__()

        self.time_step = time_step

        n_inputs = motion_model.propulsion_model.n_inputs

        self.ref_input_port = self.DeclareInputPort("twist_ref",
                                        PortDataType.kVectorValued,
                                        size=6)

        self.state_input_port = self.DeclareInputPort("state",
            PortDataType.kVectorValued,
            size=RigidBodyState.SIZE)

        self.imu_input_port = self.DeclareInputPort("imu",
            PortDataType.kVectorValued,
            size=6)

        self.rpm_input_port = self.DeclareInputPort("rpm_values",
            PortDataType.kVectorValued,
            size=n_inputs)

        self.control_output_port = self.DeclareVectorOutputPort(
            "rpm_setpoint",
            BasicVector(n_inputs),
            self.Update)

        self.Kp = Kp
        self.Kd = Kd

        self.motion_model = motion_model

        self.control_allocation = QpControlAllocation(self.motion_model)
        # When True, allocate incrementally via QP (respects thrust saturation).
        self.enable_qp = False

    def Update(self, context, output):
        '''@brief Output callback: PD control law + (QP or pseudo-inverse) allocation.'''
        state = self.state_input_port.Eval(context)
        x_d = self.ref_input_port.Eval(context)
        x0_dot = self.imu_input_port.Eval(context)

        vlin = np.array([state[RigidBodyState.VLIN_X],
                         state[RigidBodyState.VLIN_Y],
                         state[RigidBodyState.VLIN_Z]])
        vang = np.array([state[RigidBodyState.VANG_X],
                         state[RigidBodyState.VANG_Y],
                         state[RigidBodyState.VANG_Z]])

        x0 = np.concatenate((vlin, vang))

        # Current thrust operating point (measured RPM -> wrench).
        n0 = self.rpm_input_port.Eval(context)
        u0 = self.motion_model.propulsion_model.calculate_wrench(n0)

        # PD control law: desired acceleration.
        ctrl_variable = self.Kp.dot(x_d - x0) - self.Kd.dot(x0_dot)

        if self.enable_qp:
            # Incremental allocation about the current acceleration.
            n = self.control_allocation.allocate_incremental(ctrl_variable - x0_dot, n0)
        else:
            # Classical increment: convert the acceleration delta to a wrench delta.
            u_delta = self.motion_model.mass_matrix.dot(ctrl_variable - (x0_dot))
            u = u0 + u_delta
            n = self.motion_model.propulsion_model.allocate(u)

        output.SetFromVector(n)
