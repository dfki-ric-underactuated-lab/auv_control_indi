import numpy as np
import yaml

from ..common import skew, quat2rot
from ..physics.drag_model import LinearDragModel, QuadraticDragModel
from ..physics.propulsion_model import PropulsionModel

class MotionModel:
    def __init__(self):
        self.params = None
        self.propulsion_model = None
        self.mass_matrix = None
        self.mass_matrix_inv = None
        self.restoring_params = None
        self.drag_model = None

    def copy(self):
        m = MotionModel()
        m.UpdateParams(self.params)

        return m

    def UpdateParams(self, params:dict):
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
        
    def InjectNoise(self, fraction):
        self.params['M'] = np.random.normal(np.array(self.params['M']), np.abs(fraction * np.array(self.params['M'])))
        self.params['Dl'] = np.random.normal(np.array(self.params['Dl']), np.abs(fraction * np.array(self.params['Dl'])))
        if self.params['drag_model'] == 'quadratic':
            self.params['Dq'] = np.random.normal(np.array(self.params['Dq']), np.abs(fraction * np.array(self.params['Dq'])))
        self.params['restoring'] = np.random.normal(np.array(self.params['restoring']), np.abs(fraction * np.array(self.params['restoring'])))
        self.UpdateParams(self.params)

    @staticmethod
    def FromYaml(path):
        model = MotionModel()

        with open(path, "r") as stream:
            params = yaml.safe_load(stream)
            model.UpdateParams(params)

        return model
    
    def set_propulsion_model(self, propulsion_model:PropulsionModel):
        self.propulsion_model = propulsion_model
    
    def CalculateAcceleration(self, twist, orientation, tau_external):
        tau_coriolis = self.CalculateCoriolisWrench(twist)
        tau_drag = self.CalculateDampingWrench(twist)
        tau_restoring = self.CalculateRestoringWrench(orientation)

        tau = tau_external - tau_drag - tau_restoring - tau_coriolis
        return self.mass_matrix_inv @ tau

    # (Antonelli 2.45)
    def CalculateRestoringWrench(self, orientation):
        rot_bi = quat2rot(orientation).T
        z_b = rot_bi.dot([0, 0, -1])
        regressor_matrix = np.block([[np.transpose([z_b]), np.zeros((3,3))], [np.zeros((3,1)), skew(z_b)]])
    
        return regressor_matrix.dot(self.restoring_params)
    
    def CalculateDampingWrench(self, twist):
        if not self.drag_model:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        return self.drag_model.CalculateWrench(twist)
    
    def CalculateCoriolisWrench(self, twist):
        return self.CalculateCoriolisMatrix(twist) @ twist

    def CalculateCoriolisMatrix(self, twist):
        twist = np.asarray(twist)
        vlin = twist[0:3]
        vang = twist[3:6]
    
        m_11 = self.mass_matrix[0:3,0:3]
        m_12 = self.mass_matrix[0:3,3:6]
        m_21 = self.mass_matrix[3:6,0:3]
        m_22 = self.mass_matrix[3:6,3:6]

        c_12 = -1 * skew(m_11@vlin + m_12@vang)
        c_22 = -1 * skew(m_21@vlin + m_22@vang)

        return np.block([[np.zeros([3,3]), c_12],
                         [c_12,            c_22]])

    def CalculateKinematics(self, twist, orientation):
        vlin = twist[0:3]
        vang = twist[3:6]
        
        rot_ib = quat2rot(orientation)
        
        # linear
        pos_dot = rot_ib @ vlin
   
        # angular (quaternion propagation)
        quat_x, quat_y, quat_z, quat_w = orientation
        
        jac_q = 1.0/2.0 * np.array([[   quat_w, -1*quat_z,    quat_y],
                                    [   quat_z,    quat_w, -1*quat_x],
                                    [-1*quat_y,    quat_x,    quat_w],
                                    [-1*quat_x, -1*quat_y, -1*quat_z]])

        ori_dot = jac_q @ vang

        #ori_dot[3] = 0.0
        
        return np.concatenate((pos_dot, ori_dot), axis=None)
    