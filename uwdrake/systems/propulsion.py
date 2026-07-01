##
# @file
# @brief Drake LeafSystem wrapping the thruster model with actuator dynamics.

from pydrake.all import (
    BasicVector,
    PortDataType,
    InputPortIndex,
    LeafSystem,
    Context
)

from ..physics.propulsion_model import PropulsionModel

import numpy as np

class Propulsion(LeafSystem):
    ## @brief Integrates rotor RPM towards the commanded setpoints and outputs the
    #  resulting body wrench. Disabled thrusters are driven to zero RPM.
    #
    #  Ports: in "rpm_setpoints" (n) -> out "wrench" (6), "rpm_values" (n, state).
    def __init__(self, propulsion_model:PropulsionModel):
        super().__init__()

        self.propulsion_model = propulsion_model

        self.state = self.DeclareContinuousState(self.propulsion_model.n_inputs)

        self.rpm_setpoint_port = self.DeclareInputPort("rpm_setpoints",
                                        PortDataType.kVectorValued,
                                        size=propulsion_model.n_inputs)

        self.wrench_output_port = self.DeclareVectorOutputPort(
            "wrench", 6, self.UpdateOutput
        )

        self.rpm_output_port = self.DeclareStateOutputPort(
                                        "rpm_values", self.state)

        # 1 = thruster produces thrust, 0 = failed/off.
        self.enabled_outputs = np.ones(self.propulsion_model.n_inputs)

    def set_thruster_enabled(self, index, enabled=True):
        '''@brief Enable or disable a single thruster (disabled = no thrust).'''
        self.enabled_outputs[index] = 1.0 if enabled else 0.0

    def is_enabled(self, index):
        '''@brief Whether the thruster at @p index is currently producing thrust.'''
        return bool(self.enabled_outputs[index])

    def reset_thrusters(self):
        '''@brief Re-enable all thrusters.'''
        self.enabled_outputs = np.ones(self.propulsion_model.n_inputs)

    def UpdateOutput(self, context, output):
        '''@brief Output callback: wrench from current RPM plus rotor-acceleration term.'''
        rpm_setpoints = self.rpm_setpoint_port.Eval(context)
        rpm_values = context.get_continuous_state_vector().CopyToVector()
        wrench = self.propulsion_model.calculate_wrench(rpm_values)

        rpm_dot = self.propulsion_model.calculate_acceleration(rpm_setpoints, rpm_values)
        wrench = wrench + self.propulsion_model.calculate_acceleration_wrench(rpm_dot)

        output.SetFromVector(wrench)

    def DoCalcTimeDerivatives(self, context:Context, derivatives):
        '''@brief Rotor RPM dynamics; disabled thrusters get a zero setpoint.'''
        rpm_setpoints = self.rpm_setpoint_port.Eval(context)
        rpm_values = context.get_continuous_state_vector().CopyToVector()

        for i, enabled in enumerate(self.enabled_outputs):
            if not enabled:
                rpm_setpoints[i] = 0.0

        rpm_dot = self.propulsion_model.calculate_acceleration(rpm_setpoints, rpm_values)
        derivatives.get_mutable_vector().SetFromVector(rpm_dot)
