import sys
import os
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from uwdrake.systems.orbit_simulation import (
    OrbitSimConfig, build_orbit_diagram, make_orbit_simulator)
from uwdrake.evaluation import calculate_conic_angle_error, plot_thrusters_screen
from uwdrake.visualization import RigidBodyTrajectoryPlot

# The persistent simulation setup (gains, models, noise, ...) lives in the YAML.
config = OrbitSimConfig.from_yaml('configs/orbit_indi.yml')

# Per-experiment settings live here in the script.
sim_time_end = 120.0

# Thruster failure: at t_fail, keep only the thrusters marked active (1).
# | Index | Thruster               |
# |-------|------------------------|
# | 0     | Vertical front left    |
# | 1     | Vertical front right   |
# | 2     | Vertical tail right    |
# | 3     | Vertical tail left     |
# | 4     | Horizontal front left  |
# | 5     | Horizontal front right |
# | 6     | Horizontal tail right  |
# | 7     | Horizontal tail left   |
t_fail = 20.0
thruster_configuration = [0, 1, 1, 1, 0, 0, 1, 1]

'''
Build and simulate.
'''
sim = build_orbit_diagram(config)
sim.fault_scheduler.schedule(t_fail=t_fail, thruster_configuration=thruster_configuration)
simulator, context = make_orbit_simulator(sim, config)
simulator.AdvanceTo(sim_time_end)

'''
Plotting
'''
sample_times = sim.loggers['state'].FindLog(context).sample_times()
state_log = sim.loggers['state'].FindLog(context).data().transpose()
rpm_log = sim.loggers['rpm'].FindLog(context).data().transpose()
model_rpm_log = sim.loggers['model_rpm'].FindLog(context).data().transpose()
ori_ref_log = sim.loggers['ori_ref'].FindLog(context).data().transpose()
wrench_log = sim.loggers['wrench'].FindLog(context).data().transpose()

conic_error = calculate_conic_angle_error(ori_ref_log, state_log[:, 3:7])

plot_thrusters_screen(sim.thrusters.propulsion_model, sample_times, rpm_log,
                      model_rpm_log, sim.fault_scheduler, plot_thrust=True)
plt.show(block=False)

workspace = [[-5, 5], [-5, 5], [-5, 0]]
plot = RigidBodyTrajectoryPlot(sample_times, state_log, wrench_log, workspace,
                               quat_reference=ori_ref_log)
plt.show(block=True)
