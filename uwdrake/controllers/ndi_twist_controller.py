from pydrake.all import (
    PortDataType,
    BasicVector,
    LeafSystem,
    Context
)

import numpy as np
from ..physics.motion_model import MotionModel
from ..common import skew, quat2rot, RigidBodyState

class NdiTwistController(LeafSystem):
    """
    A Model-Based PD-Controller.
    """

    def __init__(self, motion_model, Kp, Kd, K_theta):
        super().__init__()


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

        self.state = self.DeclareContinuousState(4) # 4 adaptive parameters


        self.state_output_port = self.DeclareStateOutputPort(
                                        "state", self.state)
        
        self.Kp = Kp
        self.Kd = Kd
        self.K_theta = K_theta
        
        self.motion_model = motion_model
        

    def GetRegressorMatrix(self, orientation):
        rot_bi = quat2rot(orientation).T
        
        z_b = rot_bi.dot([0, 0, -1])

        regressor_matrix = np.block([[np.transpose([z_b]), np.zeros((3,3))],
                                     [np.zeros((3,1)), skew(z_b)]])
        
        return regressor_matrix


    # Adaptive Restoring Force Parameters
    def DoCalcTimeDerivatives(self, context:Context, derivatives):
        x = self.state_input_port.Eval(context)
        x_d = self.ref_input_port.Eval(context)

        nu = x[7:13]
        ori = x[3:7]

        theta_dot = self.K_theta @ self.GetRegressorMatrix(ori).T.dot(x_d - nu)
        
        derivatives.get_mutable_vector().SetFromVector(theta_dot)
        
    def Update(self, context, output):
        # Derivative of Reference Signal. TODO: Use Derivative of ref_input_port!
        x_d_dot = np.zeros((6))

        # Actual State
        state = self.state_input_port.Eval(context)
        
        # Velocity Setpoint
        x_d = self.ref_input_port.Eval(context)

        # Acceleration
        x0_dot = self.imu_input_port.Eval(context)

        vlin = np.array([state[RigidBodyState.VLIN_X],
                         state[RigidBodyState.VLIN_Y],
                         state[RigidBodyState.VLIN_Z]])
        vang = np.array([state[RigidBodyState.VANG_X],
                         state[RigidBodyState.VANG_Y],
                         state[RigidBodyState.VANG_Z]])
        quat = np.array([state[RigidBodyState.ORI_X],
                         state[RigidBodyState.ORI_Y],
                         state[RigidBodyState.ORI_Z],
                         state[RigidBodyState.ORI_W]])

        x_lin = np.concatenate((vlin, vang))
        
        # PD Control Law
        ctrl_variable = self.Kp.dot(x_d - x_lin) + self.Kd.dot(x_d_dot - x0_dot)

        # Compensate Restoring Forces
        theta = context.get_continuous_state_vector().CopyToVector()
        tau_restoring = self.GetRegressorMatrix(quat).dot(theta)

        # Model-Based Control Law
        tau_inertia = self.motion_model.mass_matrix.dot(ctrl_variable)
        tau_coriolis = self.motion_model.CalculateCoriolisWrench(x_lin)
        tau_drag = self.motion_model.CalculateDampingWrench(x_lin)
        u = tau_inertia + tau_drag + tau_coriolis + tau_restoring

        # Control Allocation
        n = self.motion_model.propulsion_model.allocate(u)

        output.SetFromVector(n)
        