from pydrake.systems.framework import BasicVector, LeafSystem, Context, PortDataType

import numpy as np
import math

class ModulatedVectorSource(LeafSystem):
    """
    A simple reference generator for performing modulated excitation motions
    (for example, for model identification or unit tests)

    type can be: [sine, square]
    """

    def __init__(self, gains, freq, phase):
        super().__init__()
        self.gains = np.array(gains)
        self.freq = np.array(freq)
        self.phase = np.array(phase)

        self.wrench_output_port = self.DeclareVectorOutputPort(
            "wrench",
            BasicVector(gains.shape[0]),
            self.Update)
        
    def Update(self, context, output):
        t = context.get_time()

        u = self.gains * np.sin(2*math.pi * self.freq * t + self.phase)

        output.SetFromVector(u)
        
class FlippingVectorSource(LeafSystem):
    def __init__(self, vector_a, vector_b, freq, phase):
        super().__init__()
        self.type = type
        self.vector_a = np.array(vector_a)
        self.vector_b = np.array(vector_b)
        self.freq = np.array(freq)
        self.phase = np.array(phase)

        self.output_port = self.DeclareVectorOutputPort(
            "output",
            BasicVector(self.vector_a.shape[0]),
            self.Update)
        
    def Update(self, context, output):
        t = context.get_time()

        u = np.sin(2*math.pi * self.freq * t + self.phase)
        if u > 0:
            output.SetFromVector(self.vector_a)
        else:
            output.SetFromVector(self.vector_b)