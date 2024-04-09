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
    def __init__(self, propulsion_model:PropulsionModel, pass_through=False):
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
        
        self.pass_through = pass_through

    def _construct_copy(self, other, converter=None):
        Impl._construct(self, other.propulsion_model, converter=converter)

    def UpdateOutput(self, context, output):
        rpm_setpoints = self.rpm_setpoint_port.Eval(context)
        rpm_values = context.get_continuous_state_vector().CopyToVector()

        if self.pass_through:
            wrench = self.propulsion_model.calculate_wrench(rpm_setpoints)
        else:
            wrench = self.propulsion_model.calculate_wrench(rpm_values)
        
        output.SetFromVector(wrench)

    def DoCalcTimeDerivatives(self, context:Context, derivatives):
        rpm_setpoints = self.rpm_setpoint_port.Eval(context)
        rpm_values = context.get_continuous_state_vector().CopyToVector()
        
        # Definition of First order system:
        # x_dot(t) = (K*u(t) - x(t)) / T
        
        #rpm_dot = 
        
        # for i in range(self.propulsion_model.n_inputs):
        #     e = rpm_setpoints[i] - rpm_values[i]
        #     if e > 0:
        #         rpm_dot = np.clip(e)
        
        rpm_dot = (rpm_setpoints - rpm_values) / self.propulsion_model.time_constant
        rpm_dot = np.clip(rpm_dot, -self.propulsion_model.max_acceleration*60, self.propulsion_model.max_acceleration*60)
        derivatives.get_mutable_vector().SetFromVector(rpm_dot)
