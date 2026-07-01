##
# @file
# @brief Hydrodynamic drag models (linear and quadratic damping).

import numpy as np

class DragModel:
    ## @brief Base drag model; total wrench = linear + nonlinear damping.
    def __init__(self, params:dict):
        self.update_params(params)

    def update_params(self, params:dict):
        '''@brief Load damping parameters from a params dict.'''
        pass

    def calculate_wrench(self, twist):
        '''@brief Total drag wrench for the given body twist.'''
        return self.calculate_wrench_lin(twist) + self.calculate_wrench_nonlin(twist)

    def calculate_wrench_lin(self, twist):
        '''@brief Linear (velocity-proportional) damping wrench.'''
        pass

    def calculate_wrench_nonlin(self, twist):
        '''@brief Nonlinear damping wrench (zero for the base model).'''
        return np.zeros(6)

class LinearDragModel(DragModel):
    ## @brief Linear damping only: tau = Dl * v.
    def __init__(self, params:dict):
        self.update_params(params)

    def update_params(self, params:dict):
        self.linear_damping_matrix = np.asarray(params['Dl'])

    def calculate_wrench_lin(self, twist):
        return self.linear_damping_matrix.dot(twist)

class QuadraticDragModel(DragModel):
    ## @brief Linear plus quadratic damping: tau = Dl * v + Dq * |v| * v.
    def __init__(self, params:dict):
        self.update_params(params)

    def update_params(self, params:dict):
        self.linear_damping_matrix = np.asarray(params['Dl'])
        self.quad_damping_matrix = np.asarray(params['Dq'])

    def calculate_wrench_lin(self, twist):
        return self.linear_damping_matrix.dot(twist)

    def calculate_wrench_nonlin(self, twist):
        return self.quad_damping_matrix.dot(np.abs(twist) * twist)
