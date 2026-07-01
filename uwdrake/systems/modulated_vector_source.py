##
# @file
# @brief Simple time-driven vector sources used as references / disturbances.

from pydrake.systems.framework import BasicVector, LeafSystem, Context, PortDataType

import numpy as np
import math

class ModulatedVectorSource(LeafSystem):
    ## @brief Sinusoidal excitation source (e.g. for model identification / tests).
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
        '''@brief Output gains * sin(2*pi*freq*t + phase).'''
        t = context.get_time()
        u = self.gains * np.sin(2*math.pi * self.freq * t + self.phase)
        output.SetFromVector(u)

class FlippingVectorSource(LeafSystem):
    ## @brief Square-wave source toggling between two vectors at the given frequency.
    def __init__(self, vector_a, vector_b, freq, phase):
        super().__init__()
        self.vector_a = np.array(vector_a)
        self.vector_b = np.array(vector_b)
        self.freq = np.array(freq)
        self.phase = np.array(phase)

        self.output_port = self.DeclareVectorOutputPort(
            "output",
            BasicVector(self.vector_a.shape[0]),
            self.Update)

    def Update(self, context, output):
        '''@brief Output vector_a while the sine is positive, else vector_b.'''
        t = context.get_time()
        u = np.sin(2*math.pi * self.freq * t + self.phase)
        output.SetFromVector(self.vector_a if u > 0 else self.vector_b)
