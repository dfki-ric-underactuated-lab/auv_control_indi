from pydrake.systems.framework import BasicVector, LeafSystem

from pydrake.all import (
    PortDataType
)

import numpy as np
from ..physics.motion_model import MotionModel
from ..common import skew, quat2rot, RigidBodyState

class IndiTwistController(LeafSystem):
    def __init__(self, time_step, motion_model:MotionModel, Kp, Kd):
        super().__init__()

        self.time_step = time_step

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
            size=8)

        self.control_output_port = self.DeclareVectorOutputPort(
            "rpm_setpoint",
            BasicVector(8),
            self.Update)
        
        self.Kp = Kp
        self.Kd = Kd

        self.motion_model = motion_model

    def Update(self, context, output):
        # Derivative of Reference Signal. TODO: Use Derivative of ref_input_port!
        x_d_dot = np.zeros((6))

        # State vectors and accelerations
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
        
        # Convert RPM readings to wrench.
        n0 = self.rpm_input_port.Eval(context)
        u0 = self.motion_model.propulsion_model.calculate_wrench(n0)

        # PD Control Law
        ctrl_variable = self.Kp.dot(x_d - x0) + self.Kd.dot(x_d_dot - x0_dot)

        # INDI Control Increment
        u_delta = self.motion_model.mass_matrix.dot(ctrl_variable - (x0_dot))

        u = u0 + u_delta
        
        # Control Allocation
        n = self.motion_model.propulsion_model.allocate(u)
        output.SetFromVector(n)
        
    
