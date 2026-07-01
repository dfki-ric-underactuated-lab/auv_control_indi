'''
Headless wrapper around the orbit-inspection diagram for the interactive GUI.

`OrbitSimDriver` owns the built diagram, the simulator and its context, and
advances the simulation by real wall-clock time. It exposes the live-tunable
knobs (orbit/motion, target, accelerometer noise) as plain method calls and
reports the pose + tracking metrics the GUI needs each frame.

The class is deliberately GUI-agnostic (no Qt / VTK imports) so it can be
exercised in a headless smoke test.
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..common import RigidBodyState
from ..controllers.polar_control_state import OrbitControlStatus
from ..evaluation import calculate_conic_angle_error
from ..systems.orbit_simulation import (
    OrbitSimConfig, build_orbit_diagram, make_orbit_simulator)

# Human-readable labels for the pose-controller state machine.
_CONTROL_STATUS_LABELS = {
    OrbitControlStatus.WAITING: 'WAITING',
    OrbitControlStatus.RUNNING: 'RUNNING',
    OrbitControlStatus.ALIGNMENT: 'ALIGNMENT',
}

# Rotate the *displayed* body frame 180 deg about its own y-axis so the blue
# z-axis points toward the inspected object (matches the sensor view direction).
# Rendering only -- never touches the simulation/control.
_DISPLAY_FLIP = R.from_euler('y', np.pi)

# Cap sim advance per frame (s) so a slow redraw can't make the sim lurch.
MAX_CATCHUP = 0.25


class OrbitSimDriver:
    '''Advance the orbit simulation in real time and surface its live state.'''

    def __init__(self, config: OrbitSimConfig):
        self.config = config
        self.sim_time = 0.0
        self._build()

    # -- construction / lifecycle -----------------------------------------

    def _build(self):
        '''(Re)build the diagram and a fresh simulator from the current config.'''
        self.sim = build_orbit_diagram(self.config)
        # World-fixed altitude is the only correct setting for the live demo.
        self.sim.pose_controller.polar_state.fix_altitude_relative_to_ground_plane = True
        self.simulator, self.context = make_orbit_simulator(self.sim, self.config)
        self._vehicle_ctx = self.sim.vehicle.GetMyContextFromRoot(self.context)
        self._pose_ctx = self.sim.pose_controller.GetMyContextFromRoot(self.context)
        self.sim_time = 0.0

    def reset(self):
        '''Re-enable thrusters and return pose/time/filters to the initial state.'''
        self.sim.thrusters.reset_thrusters()
        self.simulator, self.context = make_orbit_simulator(self.sim, self.config)
        self._vehicle_ctx = self.sim.vehicle.GetMyContextFromRoot(self.context)
        self._pose_ctx = self.sim.pose_controller.GetMyContextFromRoot(self.context)
        self.sim_time = 0.0

    def rebuild(self):
        '''Full diagram rebuild -- needed for params baked in at build time
        (e.g. actuator delay steps). Returns to the initial state.'''
        self._build()

    def step(self, wall_dt, realtime_factor=1.0):
        '''Advance the simulation by `wall_dt` seconds of wall time (clamped).'''
        self.sim_time += min(wall_dt, MAX_CATCHUP) * realtime_factor
        self.simulator.AdvanceTo(self.sim_time)

    # -- live readouts -----------------------------------------------------

    def _state(self):
        '''@brief Current full vehicle state vector.'''
        return self.sim.vehicle.state_output_port.Eval(self._vehicle_ctx)

    def pose(self):
        '''Display pose [x, y, z, qx, qy, qz, qw] with the rendering flip applied.'''
        state = self._state()
        pos = RigidBodyState.position(state)
        quat = (R.from_quat(RigidBodyState.quaternion(state)) * _DISPLAY_FLIP).as_quat()
        return np.concatenate((pos, quat))

    def position(self):
        '''World position [x, y, z] (no display flip; used for the trail).'''
        return np.asarray(RigidBodyState.position(self._state()), dtype=float)

    def metrics(self):
        '''Current tracking metrics, evaluated from the pose-controller ports.

        Returns a dict: cone_error [deg], radius_error [m], altitude_error [m],
        tangential_velocity [m/s].
        '''
        pc = self.sim.pose_controller
        ori_ref = np.asarray(pc.ori_ref_output_port.Eval(self._pose_ctx))
        ori_cur = RigidBodyState.quaternion(self._state())
        cone = calculate_conic_angle_error(ori_ref[None, :], np.asarray(ori_cur)[None, :])[0]

        polar_ref = np.asarray(pc.polar_ref_output_port.Eval(self._pose_ctx))
        polar = np.asarray(pc.polar_output_port.Eval(self._pose_ctx))
        polar_err = polar_ref - polar

        polar_vel = np.asarray(pc.polar_velocity_output_port.Eval(self._pose_ctx))

        return {
            'cone_error': float(cone),
            'radius_error': float(polar_err[0]),
            'altitude_error': float(polar_err[2]),
            'tangential_velocity': float(polar_vel[1]),
        }

    def control_status(self):
        '''Current pose-controller state-machine status as a label string.'''
        status = self.sim.pose_controller.polar_state.status
        return _CONTROL_STATUS_LABELS.get(status, 'UNKNOWN')

    # -- thrusters ---------------------------------------------------------

    @property
    def thruster_names(self):
        '''@brief Human-readable thruster names.'''
        return self.sim.thrusters.propulsion_model.thruster_names

    @property
    def n_thrusters(self):
        '''@brief Number of thrusters.'''
        return len(self.thruster_names)

    def is_enabled(self, index):
        '''@brief Whether thruster @p index is producing thrust.'''
        return self.sim.thrusters.is_enabled(index)

    def set_thruster_enabled(self, index, enabled):
        '''@brief Enable/disable thruster @p index.'''
        self.sim.thrusters.set_thruster_enabled(index, enabled)

    def n_failed(self):
        '''@brief Count of currently disabled thrusters.'''
        return int(np.sum(self.sim.thrusters.enabled_outputs == 0))

    # -- live configuration ------------------------------------------------

    def set_orbit_params(self, *, radius=None, altitude=None,
                         tangential_velocity=None, continuous_motion=None,
                         azimuth=None):
        '''Mutate the pose-controller command in place (read fresh each step).

        `tangential_velocity` is signed: a negative value reverses the orbit
        direction in continuous motion.
        '''
        cmd = self.sim.pose_controller.command
        if radius is not None:
            cmd.radius = radius
            self.config.radius = radius
        if altitude is not None:
            cmd.altitude = altitude
            self.config.altitude = altitude
        if tangential_velocity is not None:
            cmd.tangential_velocity = tangential_velocity
            self.config.tangential_velocity = tangential_velocity
        if continuous_motion is not None:
            cmd.continuous_motion = continuous_motion
            self.config.continuous_motion = continuous_motion
        if azimuth is not None:
            cmd.azimuth = azimuth
            self.config.azimuth = azimuth

    def set_target_position(self, target):
        '''Move the inspected point (affects control and the rendered marker).'''
        target = np.asarray(target, dtype=float)
        self.sim.pose_controller.world_target_position = target
        self.config.target_position = target

    def set_accelerometer_stddev(self, stddev):
        '''Set the per-axis accelerometer noise stddev (scalar or length-6).'''
        stddev = np.broadcast_to(np.asarray(stddev, dtype=float), (6,)).copy()
        self.sim.accelerometer_disturbance.std_dev = stddev
        self.config.acc_disturbance_stddev = stddev

    def set_actuator_delay_steps(self, steps):
        '''Set actuator delay (baked into the diagram -> requires rebuild()).'''
        self.config.act_delay_steps = int(steps)
