##
# @file
# @brief QP-based control allocation (direct and incremental) with thrust limits.

import numpy as np

from cvxpy import Variable, Parameter, Problem, Minimize, sum_squares, OSQP
from cvxpy.error import SolverError


class QpControlAllocation:
    ## @brief Allocates a desired body acceleration/wrench to per-thruster RPM,
    #  respecting thrust saturation, by solving a small weighted least-squares QP.
    def __init__(self, motion_model):
        '''@brief Cache the model-derived constants and pre-build the incremental QP.'''
        self.motion_model = motion_model
        self.n_inputs = motion_model.propulsion_model.n_inputs

        self.gamma = 10000000
        self.W_u = np.diag(np.ones(self.n_inputs))
        self.W_x = np.diag([100.0, 100.0, 100.0, 1000.0, 1000.0, 1.0])

        self.thrust_limit = motion_model.propulsion_model.coefficient\
            * motion_model.propulsion_model.max_rpm ** 2

        self.rpm_limit = motion_model.propulsion_model.max_rpm

        # Last successful allocation, held as a fallback if a solve fails.
        self._last_n = np.zeros(self.n_inputs)

        # Cached, parametrized QPs for allocate() and allocate_incremental().
        # Building a fresh cvxpy Problem on every controller tick re-runs the
        # (expensive) canonicalization each time; instead we build them once
        # here with Parameters and only update their values per call, so
        # solves are fast (and warm-started). W_x is baked into both
        # problems, so changing it must go through set_W_x() to rebuild.
        self._build_direct_problem()
        self._build_incremental_problem()

    def set_W_x(self, W_x):
        '''@brief Set the per-DOF residual weighting and rebuild the cached QPs.'''
        self.W_x = np.asarray(W_x, dtype=float)
        self._build_direct_problem()
        self._build_incremental_problem()

    def _build_direct_problem(self):
        '''@brief Build the parametrized direct QP once (canonicalized on first solve).

        W_x and B are constant for a given model; only the right-hand side
        (nu) and the per-thruster bounds (which encode enabled_thrusters)
        change per call and are exposed as cvxpy Parameters.
        '''
        B = self.motion_model.mass_matrix_inv @ self.motion_model.propulsion_model.allocation_matrix

        u = Variable(self.n_inputs)
        p_nu = Parameter(B.shape[0])
        p_u_min = Parameter(self.n_inputs)
        p_u_max = Parameter(self.n_inputs)

        objective = 0.5*sum_squares(self.W_x @ (B @ u - p_nu))
        constraints = [u >= p_u_min, u <= p_u_max]

        self._dir_var_u = u
        self._dir_p_nu = p_nu
        self._dir_p_u_min = p_u_min
        self._dir_p_u_max = p_u_max
        self._dir_prob = Problem(Minimize(objective), constraints)

    def allocate(self, nu, tau_ext, enabled_thrusters):
        '''@brief Direct allocation: minimise ||W_x (B u - nu)|| s.t. thrust limits (NDI-QP).

        @param nu Desired body acceleration.
        @param tau_ext Feed-forward disturbance wrench to compensate.
        @param enabled_thrusters 0/1 mask; disabled thrusters are constrained to zero.
        @return Per-thruster RPM command.
        '''
        # See https://scaron.info/blog/conversion-from-least-squares-to-quadratic-programming.html
        nu = nu + self.motion_model.mass_matrix_inv.dot(tau_ext)

        enabled_thrusters = np.asarray(enabled_thrusters, dtype=bool)
        u_min = np.where(enabled_thrusters, -self.thrust_limit, 0.0)
        u_max = np.where(enabled_thrusters, self.thrust_limit, 0.0)

        # Update only the parameter values, then re-solve (warm-started).
        self._dir_p_nu.value = np.asarray(nu, dtype=float)
        self._dir_p_u_min.value = u_min
        self._dir_p_u_max.value = u_max

        self._dir_prob.solve(solver=OSQP, eps_rel=0.025, warm_start=True)
        u_opt = self._dir_var_u.value

        n_opt = np.sign(u_opt) * np.sqrt(np.abs(u_opt) / self.motion_model.propulsion_model.coefficient)

        return n_opt


    def _build_incremental_problem(self):
        '''@brief Build the parametrized incremental QP once (canonicalized on first solve).

        W_u, W_x, gamma and B are constant for a given model; only the right-hand
        side (delta_a) and operating point (u0) change per call and are exposed
        as cvxpy Parameters.
        '''
        B = self.motion_model.mass_matrix_inv @ self.motion_model.propulsion_model.allocation_matrix

        u = Variable(self.n_inputs)
        p_delta_a = Parameter(B.shape[0])
        p_u_p = Parameter(self.n_inputs)      # preferential delta u (= -u0)
        p_u_min = Parameter(self.n_inputs)
        p_u_max = Parameter(self.n_inputs)

        objective = sum_squares(self.W_u @ (u - p_u_p)) \
            + self.gamma * sum_squares(self.W_x @ (B @ u - p_delta_a))
        constraints = [u >= p_u_min, u <= p_u_max]

        self._inc_var_u = u
        self._inc_p_delta_a = p_delta_a
        self._inc_p_u_p = p_u_p
        self._inc_p_u_min = p_u_min
        self._inc_p_u_max = p_u_max
        self._inc_prob = Problem(Minimize(objective), constraints)

    def allocate_incremental(self, delta_a, n0):
        '''@brief Incremental allocation around the current thrust operating point (INDI-QP).

        @param delta_a Desired change in body acceleration.
        @param n0 Current per-thruster RPM (operating point).
        @return Per-thruster RPM command; the previous command if the solve fails.
        '''
        try:
            u0 = self.motion_model.propulsion_model.coefficient * n0 * np.abs(n0)

            u_min = -self.thrust_limit - u0
            u_max = self.thrust_limit - u0

            # Update only the parameter values, then re-solve (warm-started).
            self._inc_p_delta_a.value = np.asarray(delta_a, dtype=float)
            self._inc_p_u_p.value = -u0   # preferential delta u towards u = 0
            self._inc_p_u_min.value = u_min
            self._inc_p_u_max.value = u_max

            self._inc_prob.solve(solver=OSQP, eps_rel=0.025, warm_start=True)
            u_opt = self._inc_var_u.value

            if u_opt is None:
                raise SolverError(
                    f"QP allocation returned no solution (status={self._inc_prob.status})")

            u = u0 + u_opt
            n_opt = np.sign(u) * np.sqrt(np.abs(u) / self.motion_model.propulsion_model.coefficient)

            self._last_n = n_opt
            return n_opt

        except SolverError as e:
            # Hold the previous command rather than commanding zero thrust,
            # which would be a surprising failure mode on a solver hiccup.
            print(f"QP allocation failed ({e}); holding previous command.")
            return self._last_n
