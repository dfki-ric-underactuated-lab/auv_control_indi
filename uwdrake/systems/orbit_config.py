'''
Configuration schema for the orbit-inspection simulation.

`OrbitSimConfig` is the single, typed source of truth for every tunable
parameter. Values can come from three places, in increasing precedence:

  1. the dataclass defaults below (sane fallbacks),
  2. a YAML file loaded with `OrbitSimConfig.from_yaml(path)`,
  3. explicit keyword overrides passed to the constructor or to `from_yaml`.

`to_yaml()` dumps the fully-resolved config so a run can archive the exact
parameters it used next to its results (reproducibility).
'''

import os
import dataclasses
from dataclasses import dataclass, field

import numpy as np
import yaml

# Repo root, so that the default (repo-relative) model / config paths resolve
# regardless of the current working directory.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

_VALID_CONTROLLERS = ('INDI', 'NDI', 'INDI_FTC', 'NDI_FTC')

# Fields stored as numpy arrays; coerced from plain lists (e.g. loaded from
# YAML) in __post_init__ so downstream matrix math always sees ndarrays.
_ARRAY_FIELDS = (
    'observer_rotmat', 'target_position',
    'W_x', 'Kp', 'Kd', 'K_theta', 'K_polar',
    'acc_disturbance_mean', 'acc_disturbance_stddev',
    'rpm_sensor_disturbance_mean', 'rpm_sensor_disturbance_stddev',
)


@dataclass
class OrbitSimConfig:
    '''All tunable parameters for the orbit-inspection simulation.'''

    # Controller: 'INDI' / 'NDI' (and their '..._FTC' variants).
    controller_type: str = 'INDI'
    enable_thruster_model: bool = True

    # Integrator step. (Run duration and fault injection are per-experiment and
    # live in the test scripts, not here.)
    sim_time_step: float = 0.005

    # Initial state
    pos_0: list = field(default_factory=lambda: [2.0, 0.0, -2.0])
    vlin_0: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vang_0: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpm_0: list = field(default_factory=lambda: [0.0] * 8)
    observer_rotmat: np.ndarray = field(default_factory=lambda: np.array(
        [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]))
    target_position: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -5.0]))

    # Velocity controller. Gains/weights are diagonal; stored as the diagonal
    # vector and turned into diagonal matrices by build_orbit_diagram.
    W_x: np.ndarray = field(default_factory=lambda: np.array(
        [100.0, 100.0, 100.0, 1000.0, 1000.0, 1.0]))
    Kp: np.ndarray = field(default_factory=lambda: np.array(
        [1.0, 1.0, 1.0, 1.5, 2.0, 1.5]))
    Kd: np.ndarray = field(default_factory=lambda: np.zeros(6))
    K_theta: np.ndarray = field(default_factory=lambda: np.zeros(4))
    filter_cutoff: float = 10.0

    # Pose (polar) controller (diagonal gain vector).
    K_polar: np.ndarray = field(default_factory=lambda: np.array(
        [0.3, 0.3, 0.3, 0.4, 0.3, 0.2]))
    vlin_max: float = 0.6
    vang_max: float = 0.2
    radius: float = 2.0
    # Fixed azimuth setpoint [rad], only used when continuous_motion is False.
    azimuth: float = 0.0
    altitude: float = 3.0
    # When True, orbit continuously; the sign of tangential_velocity sets the
    # direction (negative = reverse). When False, track the fixed azimuth above.
    continuous_motion: bool = True
    tangential_velocity: float = 0.1

    # Disturbances + sensor noise
    act_delay_steps: int = 20
    acc_disturbance_mean: np.ndarray = field(default_factory=lambda: np.zeros(6))
    acc_disturbance_stddev: np.ndarray = field(
        default_factory=lambda: 0.02 * np.ones(6))
    rpm_sensor_disturbance_mean: np.ndarray = field(
        default_factory=lambda: np.zeros(8))
    rpm_sensor_disturbance_stddev: np.ndarray = field(
        default_factory=lambda: 0.0 * np.ones(8))

    # Models
    model_path_simulation: str = 'models/cuttlefish/cuttlefish_linear_model.yml'
    model_path_controller: str = 'models/cuttlefish/cuttlefish_linear_model.yml'
    thruster_model_path: str = 'models/cuttlefish/cuttlefish_thrusters.yml'
    thruster_model_model_path: str = 'models/cuttlefish/cuttlefish_thrusters_model.yml'
    # Visual vehicle mesh for the interactive GUI (empty = triad only).
    vehicle_mesh_path: str = 'models/cuttlefish/meshes/cuttlefish_base.ply'
    motion_model_controller_randomize: float = 0.1
    seed: int = 100

    def __post_init__(self):
        '''@brief Coerce array-like fields (lists from YAML) to ndarrays, then validate.'''
        for name in _ARRAY_FIELDS:
            setattr(self, name, np.asarray(getattr(self, name), dtype=float))
        self.validate()

    def validate(self):
        '''@brief Raise ValueError on an inconsistent configuration.'''
        if self.controller_type not in _VALID_CONTROLLERS:
            raise ValueError(
                f"controller_type must be one of {_VALID_CONTROLLERS}, "
                f"got {self.controller_type!r}")
        if self.sim_time_step <= 0:
            raise ValueError(f"sim_time_step must be > 0, got {self.sim_time_step}")
        if self.vlin_max <= 0 or self.vang_max <= 0:
            raise ValueError("vlin_max and vang_max must be > 0")
        if self.radius < 0:
            raise ValueError(f"radius must be >= 0, got {self.radius}")

    @classmethod
    def from_yaml(cls, path, **overrides):
        '''@brief Load a config from YAML, with optional keyword overrides.

        Unknown keys (in the file or the overrides) raise, so typos surface
        instead of being silently ignored.
        '''
        with open(cls._resolve(path), 'r') as stream:
            data = yaml.safe_load(stream) or {}
        data.update(overrides)

        valid = {f.name for f in dataclasses.fields(cls)}
        unknown = set(data) - valid
        if unknown:
            raise ValueError(
                f"Unknown config keys {sorted(unknown)} in {path}; "
                f"valid keys are {sorted(valid)}")
        return cls(**data)

    def to_yaml(self, path):
        '''@brief Dump the fully-resolved config to YAML (arrays as lists).'''
        data = {}
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            data[f.name] = value.tolist() if isinstance(value, np.ndarray) else value
        with open(path, 'w') as stream:
            # default_flow_style=None keeps scalar lists inline (e.g. Kp: [1, 1,
            # ...]) so the files stay compact, while nesting stays block-style.
            yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=None)

    @staticmethod
    def _resolve(path):
        '''@brief Resolve a possibly repo-relative path against the repo root.'''
        return path if os.path.isabs(path) else os.path.join(_REPO_ROOT, path)

    @property
    def is_indi(self):
        '''@brief True for INDI controller variants.'''
        return self.controller_type in ('INDI', 'INDI_FTC')
