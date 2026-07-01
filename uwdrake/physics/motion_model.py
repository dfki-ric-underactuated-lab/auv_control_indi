##
# @file
# @brief 6-DOF rigid-body dynamics of the underwater vehicle (Fossen model).

import numpy as np
import yaml

from ..common import skew, quat2rot
from ..physics.drag_model import LinearDragModel, QuadraticDragModel
from ..physics.propulsion_model import PropulsionModel

class MotionModel:
    ## @brief Mass, damping, restoring and Coriolis terms of the vehicle dynamics.
    def __init__(self):
        self.params = None
        self.propulsion_model = None
        self.mass_matrix = None
        self.mass_matrix_inv = None
        self.restoring_params = None
        self.drag_model = None

    def copy(self):
        '''@brief Deep copy carrying the same parameters (propulsion model excluded).'''
        m = MotionModel()
        m.update_params(self.params)

        return m

    def update_params(self, params:dict):
        '''@brief (Re)build the mass matrix, restoring terms and drag model from params.

        @param params Dict with keys 'M', 'restoring', 'drag_model' (+ 'Dl'/'Dq').
        '''
        self.params = params.copy()
        self.mass_matrix = np.asarray(self.params['M'])
        if not np.all(np.linalg.eigvals(self.mass_matrix) > 0):
            raise Exception('Mass matrix is not positive definite!')

        self.mass_matrix_inv = np.linalg.inv(self.mass_matrix)

        self.restoring_params = np.asarray(self.params['restoring'])

        if self.params['drag_model'] == 'linear':
            self.drag_model = LinearDragModel(self.params)
        elif self.params['drag_model'] == 'quadratic':
            self.drag_model = QuadraticDragModel(self.params)
        elif params['drag_model'] == 'none':
            self.drag_model = None
        else:
            raise Exception('Unrecognized drag model ' + self.params['drag_model'])

    def inject_noise(self, fraction):
        '''@brief Perturb the model parameters by Gaussian noise (model mismatch).

        @param fraction Relative std-dev of the perturbation (1.0 = 100%).
        '''
        self.params['M'] = np.random.normal(np.array(self.params['M']), np.abs(fraction * np.array(self.params['M'])))
        self.params['Dl'] = np.random.normal(np.array(self.params['Dl']), np.abs(fraction * np.array(self.params['Dl'])))
        if self.params['drag_model'] == 'quadratic':
            self.params['Dq'] = np.random.normal(np.array(self.params['Dq']), np.abs(fraction * np.array(self.params['Dq'])))
        self.params['restoring'] = np.random.normal(np.array(self.params['restoring']), np.abs(fraction * np.array(self.params['restoring'])))
        self.update_params(self.params)

    @staticmethod
    def from_yaml(path):
        '''@brief Load a MotionModel from a YAML parameter file.'''
        model = MotionModel()

        with open(path, "r") as stream:
            params = yaml.safe_load(stream)
            model.update_params(params)

        return model

    def set_propulsion_model(self, propulsion_model:PropulsionModel):
        '''@brief Attach the propulsion model used for control allocation.'''
        self.propulsion_model = propulsion_model

    def calculate_acceleration(self, twist, orientation, tau_external):
        '''@brief Body-frame acceleration from an external wrench (solves M*a = tau).

        @param twist Body twist [vlin(3), vang(3)].
        @param orientation Body orientation quaternion [x, y, z, w].
        @param tau_external External wrench acting on the body.
        '''
        tau_coriolis = self.calculate_coriolis_wrench(twist)
        tau_drag = self.calculate_damping_wrench(twist)
        tau_restoring = self.calculate_restoring_wrench(orientation)

        tau = tau_external - tau_drag - tau_restoring - tau_coriolis
        return self.mass_matrix_inv @ tau

    def calculate_restoring_wrench(self, orientation):
        '''@brief Gravity/buoyancy restoring wrench (Antonelli 2.45).'''
        rot_bi = quat2rot(orientation).T
        z_b = rot_bi.dot([0, 0, -1])
        regressor_matrix = np.block([[np.transpose([z_b]), np.zeros((3,3))], [np.zeros((3,1)), skew(z_b)]])

        return regressor_matrix.dot(self.restoring_params)

    def calculate_damping_wrench(self, twist):
        '''@brief Hydrodynamic drag wrench for the given twist (zero if no drag model).'''
        if not self.drag_model:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        return self.drag_model.calculate_wrench(twist)

    def calculate_coriolis_wrench(self, twist):
        '''@brief Coriolis/centripetal wrench C(v)*v.'''
        return self.calculate_coriolis_matrix(twist) @ twist

    def calculate_coriolis_matrix(self, twist):
        '''@brief Coriolis/centripetal matrix C(v) (skew-symmetric Fossen form).'''
        twist = np.asarray(twist)
        vlin = twist[0:3]
        vang = twist[3:6]

        m_11 = self.mass_matrix[0:3,0:3]
        m_12 = self.mass_matrix[0:3,3:6]
        m_21 = self.mass_matrix[3:6,0:3]
        m_22 = self.mass_matrix[3:6,3:6]

        c_12 = -1 * skew(m_11@vlin + m_12@vang)
        c_22 = -1 * skew(m_21@vlin + m_22@vang)

        # Lower-left block reuses c_12 (Fossen skew-symmetric parametrisation).
        return np.block([[np.zeros([3,3]), c_12],
                         [c_12,            c_22]])

    def calculate_kinematics(self, twist, orientation):
        '''@brief Pose derivative [pos_dot(3), quat_dot(4)] from twist and orientation.'''
        vlin = twist[0:3]
        vang = twist[3:6]

        rot_ib = quat2rot(orientation)

        pos_dot = rot_ib @ vlin

        # Quaternion propagation: q_dot = 1/2 * Jac(q) * omega.
        quat_x, quat_y, quat_z, quat_w = orientation
        jac_q = 1.0/2.0 * np.array([[   quat_w, -1*quat_z,    quat_y],
                                    [   quat_z,    quat_w, -1*quat_x],
                                    [-1*quat_y,    quat_x,    quat_w],
                                    [-1*quat_x, -1*quat_y, -1*quat_z]])

        ori_dot = jac_q @ vang

        return np.concatenate((pos_dot, ori_dot), axis=None)
