from pydrake.systems.framework import BasicVector, LeafSystem, Context, PortDataType

import numpy as np

class RandomVectorSource(LeafSystem):
    """
    A simple reference generator following a Gaussian distribution.
    """

    def __init__(self, mean, std_dev):
        super().__init__()
        self.mean = np.array(mean)
        self.std_dev = np.array(std_dev)

        self.output_port = self.DeclareVectorOutputPort(
            "output",
            BasicVector(mean.shape[0]),
            self.Update)
        
    def Update(self, context, output):
        output.SetFromVector(np.random.normal(self.mean, self.std_dev, self.mean.shape))
        