import numpy as np
import math
import yaml
from scipy.spatial.transform import Rotation as R
from ..common import *

class PropulsionModel:
    def __init__(self, allocation_matrix, coefficient, time_constant):
        self.allocation_matrix = allocation_matrix
        self.coefficient = coefficient
        self.time_constant = time_constant

        self.tcm_pinv = np.linalg.pinv(self.allocation_matrix)
        self.n_inputs = allocation_matrix.shape[1]


    @staticmethod
    def FromYaml(path):
        with open(path, "r") as stream:
            params = yaml.safe_load(stream)
            model = PropulsionModel(np.array(params['allocation_matrix']),
                                    params['thrust_coefficient'],
                                    params['time_constant'])
            model.set_max_acceleration(params['max_acceleration'])

            return model


    def rpm_to_forces(self, n):
        forces = []

        for i in range(n.shape[0]):
            forces.append(self.coefficient * n[i] * np.abs(n[i]))
            
        return np.array(forces)

    def forces_to_body_wrench(self, forces):
        wrench = self.allocation_matrix.dot(forces)
                
        return wrench
    
    def calculate_wrench(self, n):
        '''
        Input: RPM per thruster.
        '''
        return self.forces_to_body_wrench(self.rpm_to_forces(n))
    
    def set_max_acceleration(self, max_acceleration):
         self.max_acceleration = max_acceleration

    def allocate(self, wrench):
        f = self.tcm_pinv.dot(wrench) # thrust
        n = np.sign(f) * np.sqrt(np.abs(f) / self.coefficient)

        return n