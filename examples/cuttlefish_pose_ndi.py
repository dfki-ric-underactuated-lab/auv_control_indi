import sys
import os
import numpy as np

import matplotlib.pyplot as plt

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput, FirstOrderLowPassFilter, Adder, ConstantVectorSource
from pydrake.systems.analysis import *

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from uwdrake.physics.motion_model import MotionModel
from uwdrake.systems.underwater_vehicle import UnderwaterVehicle
from uwdrake.controllers.ndi_twist_controller import NdiTwistController
from uwdrake.controllers.indi_twist_controller import IndiTwistController
from uwdrake.controllers.pose_controller import PoseController
from uwdrake.visualization import RigidBodyTrajectoryPlot
from uwdrake.physics.propulsion_model import PropulsionModel
from uwdrake.systems.propulsion import *
from uwdrake.systems.modulated_vector_source import FlippingVectorSource
from uwdrake.systems.random_vector_source import RandomVectorSource

'''
Parameters
'''

controller_type = 'NDI' # 'NDI' / 'INDI'

# Initial position (x, y, z)
pos_0    = [0, 0, -3]

# Initial orientation quaternion
ori_0    = [0, 0, 0, 1]

# Initial linear velocity (m/s)
vlin_0   = [0, 0, 0]

# Initial angular velocity (rad/s)
vang_0   = [0, 0, 0]

# Initial thruster rates (rev/s)
rpm_0    = [0, 0, 0, 0, 0, 0, 0, 0]

# Position setpoint (x, y, z)
pos_ref  = [0, 0, -3]

# Orientation setpoint quaternion
ori_ref  = [0, -np.sqrt(2)/2, 0, np.sqrt(2)/2]

# Orientation reference freq (Hz)
ori_ref_freq = 1 / (40.0)

# Pose controller parameters
K_pose   = np.diag([0.5, 0.5, 0.5, 0.15, 0.15, 0.15])
vlin_max = 0.5
vang_max = 0.2

# Proportional gain of the velocity controller
Kp       = np.diag((0.3, 0.3, 0.3, 1, 0.7, 0.7))

# Derivative gain of the velocity controller
Kd       = np.diag((0, 0, 0, 0, 0, 0))

# NDI Only: Gain of restoring parameter adaptation (0 = disable adaptive control)
K_theta  = 0 * np.diag((1, 1, 1, 1))
# K_theta  = 100 * np.diag((1, 1, 1, 1))

# Simulation Time Step
sim_time_step = 0.02

# Simulation Duration
sim_time_end  = 80.0 # seconds

# Fist-order Low-Pass Filters
acceleration_lowpass_cutoff = 20.0
state_lowpass_cutoff        = 20.0
rpm_sensor_lowpass_cutoff   = 20.0

# Artificial IMU Acceleration Sensor Noise
acc_disturbance_mean   = np.zeros((6))
acc_disturbance_stddev = 0.0 * np.ones((6))

# Artificial RPM Sensor Noise
rpm_sensor_disturbance_mean = np.zeros((8))
rpm_sensor_disturbance_stddev = 0.0 * np.ones((8))

# Motion model for the simulated vehicle
# model_path_simulation = 'models/cuttlefish/cuttlefish_linear_model.yml'
model_path_simulation = 'models/cuttlefish/cuttlefish_quadratic_model.yml'

# Motion model for the controller
# model_path_controller = 'models/cuttlefish/cuttlefish_linear_model.yml'
model_path_controller = 'models/cuttlefish/cuttlefish_quadratic_model.yml'
motion_model_controller_randomize = 0.1 # 1.0 = 100 percent
np.random.seed(123)

# Thruster model
thruster_model_path   = 'models/cuttlefish/cuttlefish_thrusters.yml'

'''
Load motion models 
'''

motion_model_simulation = MotionModel.FromYaml(os.path.join(os.path.dirname(__file__), '../', model_path_simulation))
motion_model_controller = MotionModel.FromYaml(os.path.join(os.path.dirname(__file__), '../', model_path_controller))
motion_model_controller.InjectNoise(motion_model_controller_randomize)

propulsion_model = PropulsionModel.FromYaml(thruster_model_path)
motion_model_simulation.set_propulsion_model(propulsion_model)
motion_model_controller.set_propulsion_model(propulsion_model)


'''
Set up Diagram
'''

builder = DiagramBuilder()
vehicle = builder.AddSystem(UnderwaterVehicle(motion_model_simulation))
propulsion = builder.AddSystem(Propulsion(propulsion_model))

pos_reference = builder.AddSystem(ConstantVectorSource(pos_ref))
ori_reference = builder.AddSystem(FlippingVectorSource(ori_0, ori_ref, ori_ref_freq, 0))

pose_controller = builder.AddSystem(PoseController(K_pose, vlin_max, vang_max))

if controller_type == 'INDI':
    velocity_controller = builder.AddSystem(IndiTwistController(sim_time_step, motion_model_controller, Kp, Kd))
elif controller_type == 'NDI':
    velocity_controller = builder.AddSystem(NdiTwistController(motion_model_controller, Kp, Kd, K_theta))

state_filter = builder.AddSystem(FirstOrderLowPassFilter(1.0/state_lowpass_cutoff, 13))
acceleration_filter = builder.AddSystem(FirstOrderLowPassFilter(1.0/acceleration_lowpass_cutoff, 6))
rpm_input_filter = builder.AddSystem(FirstOrderLowPassFilter(1.0/rpm_sensor_lowpass_cutoff, 8))

accelerometer_disturbance = builder.AddSystem(RandomVectorSource(acc_disturbance_mean, acc_disturbance_stddev))
accelerometer_disturbance_adder = builder.AddSystem((Adder(2, 6)))

rpm_sensor_disturbance = builder.AddSystem(RandomVectorSource(rpm_sensor_disturbance_mean, rpm_sensor_disturbance_stddev))
rpm_sensor_disturbance_adder = builder.AddSystem((Adder(2, 8)))

builder.Connect(pos_reference.get_output_port(0), pose_controller.position_ref_input_port)
builder.Connect(ori_reference.get_output_port(0), pose_controller.orientation_ref_input_port)
builder.Connect(vehicle.state_output_port, pose_controller.state_input_port)

builder.Connect(velocity_controller.control_output_port, propulsion.rpm_setpoint_port)

builder.Connect(propulsion.wrench_output_port, vehicle.wrench_input_port)
builder.Connect(pose_controller.twist_output_port, velocity_controller.ref_input_port)

builder.Connect(rpm_sensor_disturbance.get_output_port(0), rpm_sensor_disturbance_adder.get_input_port(0))
builder.Connect(propulsion.rpm_output_port, rpm_sensor_disturbance_adder.get_input_port(1))
builder.Connect(rpm_sensor_disturbance_adder.get_output_port(0), rpm_input_filter.get_input_port())
builder.Connect(rpm_input_filter.get_output_port(), velocity_controller.rpm_input_port)
#builder.Connect(rpm_sensor_disturbance_adder.get_output_port(0), velocity_controller.rpm_input_port)

builder.Connect(vehicle.state_output_port, state_filter.get_input_port())
builder.Connect(state_filter.get_output_port(), velocity_controller.state_input_port)
#builder.Connect(vehicle.state_output_port, velocity_controller.state_input_port)

builder.Connect(accelerometer_disturbance.get_output_port(0), accelerometer_disturbance_adder.get_input_port(0))
builder.Connect(vehicle.imu_acc_out_port, accelerometer_disturbance_adder.get_input_port(1))
builder.Connect(accelerometer_disturbance_adder.get_output_port(0), acceleration_filter.get_input_port())
builder.Connect(acceleration_filter.get_output_port(), velocity_controller.imu_input_port)

input_logger = LogVectorOutput(propulsion.get_output_port(0), builder)
state_logger = LogVectorOutput(vehicle.get_output_port(0), builder)
ori_ref_logger = LogVectorOutput(ori_reference.get_output_port(0), builder)
twist_ref_logger = LogVectorOutput(pose_controller.twist_output_port, builder)
diagram = builder.Build()

'''
Simulation and Logging
'''

# Set the initial conditions, x(0).
context = diagram.CreateDefaultContext()
state = context.get_mutable_continuous_state_vector()

if controller_type == 'INDI':
    state_0 = np.concatenate((pos_0, ori_0, vlin_0, vang_0, rpm_0, np.zeros((6+13+8))), axis=None)
elif controller_type == 'NDI':
    state_0 = np.concatenate((pos_0, ori_0, vlin_0, vang_0, rpm_0, \
                            motion_model_controller.restoring_params, np.zeros((6+13+8))), axis=None)

state.SetFromVector(state_0)

# Create the simulator, with fixed step-size integration.
simulator = Simulator(diagram, context)
simulator.Initialize()
ResetIntegratorFromFlags(simulator=simulator, scheme="explicit_euler", max_step_size=sim_time_step)
integrator = simulator.get_mutable_integrator()
integrator.set_fixed_step_mode(True)
integrator.set_maximum_step_size(sim_time_step)
integrator.set_requested_minimum_step_size(sim_time_step)

# Simulate
simulator.AdvanceTo(sim_time_end)

'''
Plotting
'''
input_log = input_logger.FindLog(context)
state_log = state_logger.FindLog(context)
ori_ref_log = ori_ref_logger.FindLog(context)

workspace = [5, 5, 5]
plot = RigidBodyTrajectoryPlot(state_log.sample_times(), state_log.data().transpose(), input_log.data().transpose(), workspace, quat_reference=ori_ref_log.data().transpose())
plt.show(block=True)
