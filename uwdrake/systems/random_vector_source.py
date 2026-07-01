##
# @file
# @brief Gaussian white-noise vector source (sensor noise / disturbances).

from pydrake.systems.framework import BasicVector, LeafSystem, Context, PortDataType

import numpy as np

class RandomVectorSource(LeafSystem):
    ## @brief Outputs an i.i.d. Gaussian sample each evaluation.
    #  @p std_dev may be mutated live to change the noise level.
    def __init__(self, mean, std_dev):
        super().__init__()
        self.mean = np.array(mean)
        self.std_dev = np.array(std_dev)

        self.output_port = self.DeclareVectorOutputPort(
            "output",
            BasicVector(mean.shape[0]),
            self.Update)

    def Update(self, context, output):
        '''@brief Output a fresh Gaussian sample N(mean, std_dev).'''
        output.SetFromVector(np.random.normal(self.mean, self.std_dev, self.mean.shape))
