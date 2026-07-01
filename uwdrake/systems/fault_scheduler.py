##
# @file
# @brief Time-triggered thruster failure injector for the simulation.

from pydrake.systems.framework import LeafSystem, EventStatus

import numpy as np


class FaultScheduler(LeafSystem):
    """@brief Disables a set of thrusters after a scheduled time.

    A FaultScheduler is always part of the diagram but inert until a fault is
    scheduled: without schedule() it leaves the thrusters untouched (so e.g. the
    interactive GUI can toggle them by hand). Once scheduled, from ``t0`` onwards
    only the thrusters marked active in ``thruster_selection`` keep producing
    thrust.

    The failure is applied through a per-step publish event, which Drake
    reliably evaluates on every simulator step.
    """

    def __init__(self, propulsion_system):
        super().__init__()
        self.propulsion_system = propulsion_system
        self._n_inputs = propulsion_system.propulsion_model.n_inputs

        # Inert by default: no fault, thrusters left as-is.
        self._scheduled = False
        self.t0 = float('inf')
        self.thruster_selection = np.ones(self._n_inputs)

        self.DeclarePerStepPublishEvent(self._apply_fault)

    def schedule(self, t_fail, thruster_configuration):
        '''@brief Fail every thruster not marked active (1) in @p thruster_configuration at @p t_fail.'''
        selection = np.asarray(thruster_configuration, dtype=float)
        if selection.shape != (self._n_inputs,):
            raise ValueError(
                f"thruster_configuration must have {self._n_inputs} entries, "
                f"got {selection.shape[0]}")
        if any(v not in (0.0, 1.0) for v in selection):
            raise ValueError(
                f"thruster_configuration must contain only 0/1 entries, "
                f"got {thruster_configuration}")
        if t_fail < 0:
            raise ValueError(f"t_fail must be >= 0, got {t_fail}")

        self.t0 = t_fail
        self.thruster_selection = selection
        self._scheduled = True

    def _apply_fault(self, context):
        '''@brief Per-step publish: apply the failed-thruster mask once t >= t0.'''
        if self._scheduled and context.get_time() >= self.t0:
            self.propulsion_system.enabled_outputs = self.thruster_selection
        return EventStatus.Succeeded()
