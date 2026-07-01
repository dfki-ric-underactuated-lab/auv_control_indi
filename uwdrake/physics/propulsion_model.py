##
# @file
# @brief Thruster model: maps RPM to body wrench and allocates a wrench to RPM.

import numpy as np
import yaml

class PropulsionModel:
    ## @brief Static thrust map, first-order actuator dynamics and control allocation.
    def __init__(self, allocation_matrix, coefficient, time_constant, max_rpm, moment_of_inertia):
        '''@brief Build the model from the thruster-configuration matrix and constants.

        @param allocation_matrix (6, n) thruster-configuration matrix (force -> wrench).
        @param coefficient Quadratic thrust coefficient (force = c * n * |n|).
        @param time_constant First-order actuator time constant [s].
        @param max_rpm Per-thruster RPM limit.
        @param moment_of_inertia Rotor moment of inertia (for acceleration wrench).
        '''
        self.allocation_matrix = allocation_matrix

        self.coefficient = coefficient
        self.time_constant = time_constant
        self.max_rpm       = max_rpm
        self.moment_of_inertia = moment_of_inertia

        self.tcm_pinv = np.linalg.pinv(self.allocation_matrix)
        self.n_inputs = allocation_matrix.shape[1]

        # Generic labels by default; overridden from the model file (see from_yaml).
        self.thruster_names = [f"Thruster {i}" for i in range(self.n_inputs)]


    @staticmethod
    def from_yaml(path):
        '''@brief Load a PropulsionModel from a YAML thruster-model file.'''
        with open(path, "r") as stream:
            params = yaml.safe_load(stream)
            model = PropulsionModel(np.array(params['allocation_matrix']),
                                    params['thrust_coefficient'],
                                    params['time_constant'],
                                    params['max_rpm'],
                                    params['moment_of_inertia'])
            model.set_max_acceleration(params['max_acceleration'])

            if 'thruster_names' in params:
                names = params['thruster_names']
                if len(names) != model.n_inputs:
                    raise ValueError(
                        f"thruster_names has {len(names)} entries but the "
                        f"allocation matrix defines {model.n_inputs} thrusters")
                model.thruster_names = list(names)

            return model


    def rpm_to_forces(self, n):
        '''@brief Per-thruster force from RPM via the quadratic thrust curve.'''
        forces = []

        for i in range(n.shape[0]):
            forces.append(self.coefficient * n[i] * np.abs(n[i]))

        return np.array(forces)

    def forces_to_body_wrench(self, forces):
        '''@brief Map per-thruster forces to a body wrench.'''
        return self.allocation_matrix.dot(forces)

    def calculate_wrench(self, n):
        '''@brief Body wrench produced by the given per-thruster RPM vector.'''
        return self.forces_to_body_wrench(self.rpm_to_forces(n))

    def calculate_acceleration_wrench(self, n_dot):
        '''@brief Reaction torque wrench from rotor angular acceleration.'''
        torques = self.allocation_matrix[0:3,:].dot(self.moment_of_inertia * n_dot)

        return np.concatenate((np.zeros((3)), torques))

    def set_max_acceleration(self, max_acceleration):
        '''@brief Set the per-thruster RPM/s^2 acceleration limit.'''
        self.max_acceleration = max_acceleration

    def allocate(self, wrench):
        '''@brief Pseudo-inverse allocation: desired wrench -> per-thruster RPM.'''
        f = self.tcm_pinv.dot(wrench)
        return np.sign(f) * np.sqrt(np.abs(f) / self.coefficient)

    def calculate_acceleration(self, rpm_setpoints, rpm_values):
        '''@brief First-order rotor acceleration towards the setpoint, rate-limited.'''
        rpm_dot = (rpm_setpoints - rpm_values) / (self.time_constant)
        return np.clip(rpm_dot, -self.max_acceleration*60, self.max_acceleration*60)
