##
# @file
# @brief Discrete Bessel low-pass filter as a Drake LeafSystem.

import numpy as np
from scipy.signal import bessel, bilinear
from pydrake.systems.framework import LeafSystem, BasicVector


class DiscreteFilter(LeafSystem):
    ## @brief Per-element discrete Bessel low-pass filter of configurable order.
    #
    #  @param fs Sample rate [Hz].
    #  @param fc Cutoff frequency [Hz].
    #  @param vector_dim Number of independent channels.
    #  @param order Filter order (default 2).
    def __init__(self, fs, fc, vector_dim, order=2):
        super().__init__()
        self.fs = fs
        self.fc = fc
        self.order = order
        self.dim = vector_dim
        self.dt = 1.0 / fs

        w_c = 2 * np.pi * fc
        b_s, a_s = bessel(N=order, Wn=w_c, btype='low', analog=True, norm='phase')
        b_z, a_z = bilinear(b_s, a_s, fs)

        self.n = max(len(a_z), len(b_z))
        self.b = np.pad(b_z, (0, self.n - len(b_z)))
        self.a = np.pad(a_z, (0, self.n - len(a_z)))

        self.DeclareVectorInputPort("u", BasicVector(self.dim))
        self.DeclareVectorOutputPort("y", BasicVector(self.dim), self.CalcOutput)

        self.state_dim = self.dim * 2 * (self.n - 1)
        self.DeclareDiscreteState(self.state_dim)

        self.DeclarePeriodicDiscreteUpdateEvent(self.dt, 0.0, self.Update)

    def Update(self, context, state):
        '''@brief Periodic difference-equation update of the per-channel filter state.'''
        x = np.array(state.get_vector().get_value())
        u_curr = self.get_input_port(0).Eval(context)

        for i in range(self.dim):
            offset = i * 2 * (self.n - 1)
            u_hist = np.concatenate([[u_curr[i]], x[offset:offset + self.n - 1]])
            y_hist = x[offset + self.n - 1:offset + 2 * (self.n - 1)]

            y_out = (np.dot(self.b, u_hist) - np.dot(self.a[1:], y_hist)) / self.a[0]

            x[offset:offset + self.n - 1] = u_hist[:-1]
            x[offset + self.n - 1:offset + 2 * (self.n - 1)] = np.concatenate([[y_out], y_hist[:-1]])

        state.set_value(x)

    def CalcOutput(self, context, output):
        '''@brief Output the latest filtered value of each channel.'''
        x = context.get_discrete_state_vector().get_value()
        y_vec = np.zeros(self.dim)

        for i in range(self.dim):
            offset = i * 2 * (self.n - 1)
            y_vec[i] = x[offset + self.n - 1]

        output.SetFromVector(y_vec)
