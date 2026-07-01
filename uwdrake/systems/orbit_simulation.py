'''
Reusable builder for the 360 degrees orbit-inspection simulation.

The two orbit example scripts (cuttlefish_orbit_indi_qp.py / _ndi_qp.py) and the
interactive demo all use the *same* diagram; they differ only in configuration
and in how the simulation is driven and visualized. This module centralizes the
diagram construction so those scripts stay small and consistent.

Run duration and thruster-failure injection are per-experiment concerns, so
they live in the calling script, not in OrbitSimConfig: schedule a fault on the
diagram's FaultScheduler and choose the AdvanceTo() horizon yourself.

Typical use:

    config = OrbitSimConfig.from_yaml('configs/orbit_indi.yml')
    sim = build_orbit_diagram(config)
    sim.fault_scheduler.schedule(t_fail=20.0,
                                 thruster_configuration=[0,1,1,1,0,0,1,1])
    simulator, context = make_orbit_simulator(sim, config)
    simulator.AdvanceTo(120.0)
    log = sim.loggers['state'].FindLog(context)
'''

from dataclasses import dataclass

import numpy as np

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput, Adder, DiscreteTimeDelay
from pydrake.systems.analysis import Simulator, ResetIntegratorFromFlags

from .orbit_config import OrbitSimConfig
from ..common import calculate_lookat_orientation
from ..physics.motion_model import MotionModel
from ..physics.propulsion_model import PropulsionModel
from ..systems.underwater_vehicle import UnderwaterVehicle
from ..systems.propulsion import Propulsion
from ..systems.discrete_filter import DiscreteFilter
from ..systems.fault_scheduler import FaultScheduler
from ..systems.modulated_vector_source import FlippingVectorSource
from ..systems.random_vector_source import RandomVectorSource
from ..controllers.polar_controller import PolarController, OrbitControlCommand
from ..controllers.indi_twist_controller import IndiTwistController
from ..controllers.ndi_twist_controller import NdiTwistController

@dataclass
class OrbitDiagram:
    '''@brief The built diagram plus handles to the subsystems and loggers a driver needs.'''
    diagram: object
    vehicle: object
    thrusters: object
    modeled_thrusters: object
    pose_controller: object
    velocity_controller: object
    fault_scheduler: object  # inert until the caller schedules a fault on it
    motion_model_simulation: object
    motion_model_controller: object
    # Exposed so a live driver can mutate its stddev without rebuilding the
    # diagram (RandomVectorSource reads std_dev each step). Only the accelerometer
    # disturbance is exposed: the rpm sensor disturbance is bypassed whenever the
    # thruster model is enabled, so it has no effect in the default configuration.
    accelerometer_disturbance: object
    loggers: dict


def _build_models(config: OrbitSimConfig):
    '''@brief Load the simulation/controller motion models and thruster models.

    The controller model is perturbed (inject_noise) to create model mismatch.
    '''
    motion_model_simulation = MotionModel.from_yaml(
        config._resolve(config.model_path_simulation))
    motion_model_controller = MotionModel.from_yaml(
        config._resolve(config.model_path_controller))

    np.random.seed(config.seed)
    motion_model_controller.inject_noise(config.motion_model_controller_randomize)

    propulsion_model = PropulsionModel.from_yaml(
        config._resolve(config.thruster_model_path))
    propulsion_model_model = PropulsionModel.from_yaml(
        config._resolve(config.thruster_model_model_path))

    motion_model_simulation.set_propulsion_model(propulsion_model_model)
    motion_model_controller.set_propulsion_model(propulsion_model)

    return (motion_model_simulation, motion_model_controller,
            propulsion_model, propulsion_model_model)


def build_orbit_diagram(config: OrbitSimConfig) -> OrbitDiagram:
    '''@brief Build the orbit-inspection diagram and return it with subsystem handles.

    @param config Persistent simulation setup (gains, models, noise, ...).
    @return An OrbitDiagram with the built diagram and subsystem/logger handles.
    '''
    (motion_model_simulation, motion_model_controller,
     propulsion_model, propulsion_model_model) = _build_models(config)

    builder = DiagramBuilder()

    # NOTE: keep the AddSystem order (vehicle, thrusters, modeled_thrusters, ...)
    # stable; make_orbit_simulator() relies on it for the initial-state layout.
    vehicle = builder.AddSystem(UnderwaterVehicle(motion_model_simulation))
    thrusters = builder.AddSystem(Propulsion(propulsion_model))
    modeled_thrusters = builder.AddSystem(Propulsion(propulsion_model_model))

    pose_controller = builder.AddSystem(
        PolarController(np.diag(config.K_polar), config.vlin_max, config.vang_max,
                        config.observer_rotmat))
    command = OrbitControlCommand()
    command.radius = config.radius
    command.azimuth = config.azimuth
    command.altitude = config.altitude
    command.continuous_motion = config.continuous_motion
    command.tangential_velocity = config.tangential_velocity
    command.target_point = config.target_position
    pose_controller.command = command
    pose_controller.world_target_position = config.target_position

    disturbance = builder.AddSystem(
        FlippingVectorSource(np.zeros(6), np.zeros(6), 0.05, 0))
    disturbance_adder = builder.AddSystem(Adder(2, 6))
    rpm_sensor_disturbance = builder.AddSystem(RandomVectorSource(
        config.rpm_sensor_disturbance_mean, config.rpm_sensor_disturbance_stddev))
    rpm_sensor_disturbance_adder = builder.AddSystem(Adder(2, 8))
    accelerometer_disturbance = builder.AddSystem(RandomVectorSource(
        config.acc_disturbance_mean, config.acc_disturbance_stddev))
    accelerometer_disturbance_adder = builder.AddSystem(Adder(2, 6))
    actuator_delay = builder.AddSystem(
        DiscreteTimeDelay(config.sim_time_step, config.act_delay_steps, 8))

    # Always present but inert; the calling script schedules a fault on it via
    # sim.fault_scheduler.schedule(t_fail, thruster_configuration).
    fault_scheduler = builder.AddSystem(FaultScheduler(thrusters))

    # Config stores the (diagonal) gains/weights as vectors; expand to matrices.
    Kp = np.diag(config.Kp)
    Kd = np.diag(config.Kd)
    if config.is_indi:
        velocity_controller = builder.AddSystem(IndiTwistController(
            config.sim_time_step, motion_model_controller, Kp, Kd))
    else:
        velocity_controller = builder.AddSystem(NdiTwistController(
            motion_model_controller, Kp, Kd, np.diag(config.K_theta)))

    velocity_controller.control_allocation.set_W_x(np.diag(config.W_x))
    velocity_controller.enable_qp = True

    state_filter = builder.AddSystem(
        DiscreteFilter(1.0 / config.sim_time_step, config.filter_cutoff, 13))
    acceleration_filter = builder.AddSystem(
        DiscreteFilter(1.0 / config.sim_time_step, config.filter_cutoff, 6))
    rpm_input_filter = builder.AddSystem(
        DiscreteFilter(1.0 / config.sim_time_step, config.filter_cutoff, 8))

    #
    # Wiring
    #
    builder.Connect(vehicle.state_output_port, pose_controller.state_input_port)
    builder.Connect(pose_controller.twist_output_port, velocity_controller.ref_input_port)

    builder.Connect(velocity_controller.control_output_port, actuator_delay.get_input_port())
    builder.Connect(actuator_delay.get_output_port(), thrusters.rpm_setpoint_port)
    builder.Connect(velocity_controller.control_output_port, modeled_thrusters.rpm_setpoint_port)

    builder.Connect(thrusters.wrench_output_port, disturbance_adder.get_input_port(0))
    builder.Connect(disturbance.get_output_port(0), disturbance_adder.get_input_port(1))
    builder.Connect(disturbance_adder.get_output_port(0), vehicle.wrench_input_port)

    builder.Connect(rpm_sensor_disturbance.get_output_port(0), rpm_sensor_disturbance_adder.get_input_port(0))
    builder.Connect(thrusters.rpm_output_port, rpm_sensor_disturbance_adder.get_input_port(1))

    if config.enable_thruster_model:
        builder.Connect(modeled_thrusters.rpm_output_port, rpm_input_filter.get_input_port())
    else:
        builder.Connect(rpm_sensor_disturbance_adder.get_output_port(0), rpm_input_filter.get_input_port())
    builder.Connect(rpm_input_filter.get_output_port(), velocity_controller.rpm_input_port)

    builder.Connect(vehicle.state_output_port, state_filter.get_input_port())
    builder.Connect(state_filter.get_output_port(), velocity_controller.state_input_port)

    builder.Connect(accelerometer_disturbance.get_output_port(0), accelerometer_disturbance_adder.get_input_port(0))
    builder.Connect(vehicle.imu_acc_out_port, accelerometer_disturbance_adder.get_input_port(1))
    builder.Connect(accelerometer_disturbance_adder.get_output_port(0), acceleration_filter.get_input_port())
    builder.Connect(acceleration_filter.get_output_port(), velocity_controller.imu_input_port)

    #
    # Loggers
    #
    loggers = {
        'rpm': LogVectorOutput(thrusters.rpm_output_port, builder),
        'model_rpm': LogVectorOutput(modeled_thrusters.rpm_output_port, builder),
        'model_rpm_f': LogVectorOutput(rpm_input_filter.get_output_port(), builder),
        'wrench': LogVectorOutput(thrusters.wrench_output_port, builder),
        'state': LogVectorOutput(vehicle.get_output_port(0), builder),
        'ori_ref': LogVectorOutput(pose_controller.ori_ref_output_port, builder),
        'polar_ref': LogVectorOutput(pose_controller.polar_ref_output_port, builder),
        'polar': LogVectorOutput(pose_controller.polar_output_port, builder),
        'polar_velocity': LogVectorOutput(pose_controller.polar_velocity_output_port, builder),
    }

    diagram = builder.Build()

    return OrbitDiagram(
        diagram=diagram,
        vehicle=vehicle,
        thrusters=thrusters,
        modeled_thrusters=modeled_thrusters,
        pose_controller=pose_controller,
        velocity_controller=velocity_controller,
        fault_scheduler=fault_scheduler,
        motion_model_simulation=motion_model_simulation,
        motion_model_controller=motion_model_controller,
        accelerometer_disturbance=accelerometer_disturbance,
        loggers=loggers,
    )


def initial_state(config: OrbitSimConfig, motion_model_controller):
    '''@brief Assemble the initial state vector (pose, twist, rpms, [adaptive params]).'''
    ori_0 = calculate_lookat_orientation(
        target_position=np.asarray(config.target_position),
        observer_position=np.asarray(config.pos_0),
        observer_rotmat=config.observer_rotmat,
    )

    state_0 = np.concatenate(
        (config.pos_0, ori_0, config.vlin_0, config.vang_0, config.rpm_0), axis=None)
    if not config.is_indi:
        # NDI carries 4 additional adaptive restoring parameters as state.
        state_0 = np.concatenate(
            (state_0, motion_model_controller.restoring_params), axis=None)
    return state_0


def make_orbit_simulator(sim: OrbitDiagram, config: OrbitSimConfig):
    '''@brief Create a fixed-step (explicit-Euler) simulator with the initial state.

    @return Tuple (simulator, context).
    '''
    context = sim.diagram.CreateDefaultContext()
    state = context.get_mutable_continuous_state_vector()

    state_0 = initial_state(config, sim.motion_model_controller)
    state_0 = np.pad(state_0, (0, state.size() - len(state_0)))
    state.SetFromVector(state_0)

    simulator = Simulator(sim.diagram, context)
    simulator.Initialize()
    ResetIntegratorFromFlags(simulator=simulator, scheme="explicit_euler",
                             max_step_size=config.sim_time_step)
    integrator = simulator.get_mutable_integrator()
    integrator.set_fixed_step_mode(True)
    integrator.set_maximum_step_size(config.sim_time_step)
    integrator.set_requested_minimum_step_size(config.sim_time_step)

    return simulator, context
