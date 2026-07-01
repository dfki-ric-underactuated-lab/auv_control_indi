##
# @file
# @brief Nonlinear Dynamic Inversion (NDI) body-twist controller with adaptive restoring.

from pydrake.all import (
    PortDataType,
    BasicVector,
    LeafSystem,
    Context
)

import numpy as np
from ..controllers.qp_control_allocation import QpControlAllocation
from ..common import skew, quat2rot, RigidBodyState

class NdiTwistController(LeafSystem):
    ## @brief Model-based PD twist controller that inverts the full dynamics and
    #  adapts the restoring-force parameters online.
    #
    #  Ports: in "twist_ref"(6), "state"(13), "imu"(6), "rpm_values"(n)
    #         -> out "rpm_setpoint"(n), "state"(4 adaptive params).
    def __init__(self, motion_model, Kp, Kd, K_theta):
        '''@brief Declare ports/state and store the PD and adaptation gains.

        @param K_theta Restoring-parameter adaptation gain (zero disables adaptation).
        '''
        super().__init__()

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

        # 4 adaptive restoring-force parameters carried as continuous state.
        self.state = self.DeclareContinuousState(4)
        self.state_output_port = self.DeclareStateOutputPort("state", self.state)

        self.Kp = Kp
        self.Kd = Kd
        self.K_theta = K_theta

        self.motion_model = motion_model

        self.control_allocation = QpControlAllocation(self.motion_model)
        # When True, allocate via QP (respects thrust saturation).
        self.enable_qp = False

    def get_regressor_matrix(self, orientation):
        '''@brief Restoring-force regressor Phi(q) so that tau_restoring = Phi(q) * theta.'''
        rot_bi = quat2rot(orientation).T
        z_b = rot_bi.dot([0, 0, -1])

        return np.block([[np.transpose([z_b]), np.zeros((3,3))],
                         [np.zeros((3,1)), skew(z_b)]])

    def DoCalcTimeDerivatives(self, context:Context, derivatives):
        '''@brief Adaptation law for the restoring-force parameters theta.'''
        x = self.state_input_port.Eval(context)
        x_d = self.ref_input_port.Eval(context)

        nu = x[7:13]
        ori = x[3:7]

        theta_dot = self.K_theta @ self.get_regressor_matrix(ori).T.dot(x_d - nu)

        derivatives.get_mutable_vector().SetFromVector(theta_dot)

    def Update(self, context, output):
        '''@brief Output callback: model-based PD law + control allocation.'''
        state = self.state_input_port.Eval(context)
        x_d = self.ref_input_port.Eval(context)
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

        # PD control law: desired acceleration.
        ctrl_variable = self.Kp.dot(x_d - x_lin) - self.Kd.dot(x0_dot)

        # Restoring-force compensation using the adapted parameters.
        theta = context.get_continuous_state_vector().CopyToVector()
        tau_restoring = self.get_regressor_matrix(quat).dot(theta)

        # Model terms.
        tau_inertia = self.motion_model.mass_matrix.dot(ctrl_variable)
        tau_coriolis = self.motion_model.calculate_coriolis_wrench(x_lin)
        tau_drag = self.motion_model.calculate_damping_wrench(x_lin)

        if self.enable_qp:
            # QP allocation with the disturbance wrench as feed-forward.
            tau_disturbances = tau_drag + tau_coriolis + tau_restoring
            n = self.control_allocation.allocate(ctrl_variable, tau_disturbances,
                            np.ones(self.motion_model.propulsion_model.n_inputs))
        else:
            # Classical: full inverse-dynamics wrench, then pseudo-inverse allocation.
            u = tau_inertia + tau_drag + tau_coriolis + tau_restoring
            n = self.motion_model.propulsion_model.allocate(u)

        output.SetFromVector(n)
