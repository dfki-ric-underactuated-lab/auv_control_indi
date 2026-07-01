'''
PyVista (VTK) 3D viewport for the orbit-inspection GUI.

`Viewport3D` wraps a `pyvistaqt.QtInteractor` and owns the scene: the reference
orbit ring, the target marker, the vehicle orientation triad and the travelled
trail. The static actors are built once; the triad and trail are updated cheaply
each frame (the triad via a per-actor user matrix, the trail by overwriting a
PolyData's points).

The viewport is mesh-ready: `set_vehicle_mesh(path)` loads a mesh and rigidly
attaches it to the vehicle frame so it follows the triad (hook for a future
cuttlefish mesh; no asset exists yet).
'''

import numpy as np
import pyvista as pv
from scipy.spatial.transform import Rotation as R
from pyvistaqt import QtInteractor

# The trail starts empty (the polyline grows as the vehicle moves); allow the
# empty PolyData actor to be added before it has any points.
pv.global_theme.allow_empty_mesh = True

_AXIS_COLORS = ('#d62728', '#2ca02c', '#1f77b4')  # x, y, z
_AXIS_DIRS = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# 180 deg about body y: the displayed body frame has +z pointing away from the
# target (opposite the raw pose z). The vehicle mesh uses the same flip.
_DISPLAY_FLIP = np.diag([-1.0, 1.0, -1.0, 1.0])

_FLOOR_Z = -8.0       # world z of the seafloor plane
_TUBE_RADIUS = 0.2   # inspected-tube radius [m]
_LOS_HALF_ANGLE_DEG = 15.0   # line-of-sight light-cone opening half-angle


def _transform_matrix(pose):
    '''4x4 rigid transform from a [x,y,z, qx,qy,qz,qw] pose.'''
    T = np.eye(4)
    T[:3, :3] = R.from_quat(pose[3:7]).as_matrix()
    T[:3, 3] = pose[0:3]
    return T


def _polar_grid(max_radius, ring_step, n_spokes, z):
    '''Polar grid (concentric rings + radial spokes) in the plane at height z,
    centred on the origin, returned as a single merged PolyData.'''
    parts = []
    theta = np.linspace(0.0, 2.0 * np.pi, 180)
    r = ring_step
    while r <= max_radius + 1e-9:
        pts = np.column_stack((r * np.cos(theta), r * np.sin(theta),
                               np.full_like(theta, z)))
        parts.append(pv.lines_from_points(pts, close=True))
        r += ring_step
    for k in range(n_spokes):
        ang = 2.0 * np.pi * k / n_spokes
        pts = np.array([[0.0, 0.0, z],
                        [max_radius * np.cos(ang), max_radius * np.sin(ang), z]])
        parts.append(pv.lines_from_points(pts))
    grid = parts[0]
    for part in parts[1:]:
        grid = grid.merge(part)
    return grid


class Viewport3D:
    ## @brief Owns the 3D scene (floor, orbit ring, target, vehicle triad, trail,
    #  line-of-sight cone) and updates the dynamic actors each frame.
    def __init__(self, config, parent=None, triad_length=0.8, trail_length=1800):
        '''@brief Build the static scene and the per-frame actors from @p config.'''
        self.plotter = QtInteractor(parent)
        self.triad_length = triad_length
        # Trail is capped by frame count; the render timer runs at ~60 Hz, so
        # 1800 frames is about a 30 s trailing window (matching the charts).
        self.trail_length = trail_length
        self._trail = []
        self._vehicle_actors = []  # (actor, local_offset_matrix) attached to the body
        # Observer/camera line of sight: the third column is the observer frame's
        # z axis expressed in the base link (observer_rotmat maps observer -> base
        # link). The LOS cone/line exit the vehicle along this body-frame axis.
        self._observer_rotmat = np.asarray(config.observer_rotmat, dtype=float)

        self.plotter.set_background('#1e1e24', top='#2b2b35')
        self.plotter.add_axes()

        # Seafloor: filled black surface with a polar grid (rings every 1 m and
        # radial spokes), centred on the world origin.
        floor = pv.Plane(
            center=(0.0, 0.0, _FLOOR_Z),
            direction=(0.0, 0.0, 1.0),
            i_size=40.0,
            j_size=40.0,
        )
        self.plotter.add_mesh(floor, color="black")
        self.plotter.add_mesh(
            _polar_grid(max_radius=20.0, ring_step=1.0, n_spokes=24, z=_FLOOR_Z),
            color="gray", line_width=1)

        self.build_static_scene(config)

        # Vehicle orientation triad (three arrows, transformed together each frame).
        self._triad_actors = []
        for direction, color in zip(_AXIS_DIRS, _AXIS_COLORS):
            arrow = pv.Arrow(direction=direction, scale=self.triad_length,
                             tip_length=0.25, tip_radius=0.08, shaft_radius=0.03)
            actor = self.plotter.add_mesh(arrow, color=color)
            self._triad_actors.append(actor)

        # Travelled trail (a PolyData polyline updated in place).
        self._trail_mesh = pv.PolyData()
        self._trail_actor = self.plotter.add_mesh(
            self._trail_mesh, color='#3182bd', line_width=2.0)

        # Light cone along the line of sight: apex at the vehicle, opening down
        # to the target on the floor plane. Regenerated each frame (the shape
        # changes as the vehicle moves), drawn unlit + translucent so it reads
        # as a glowing beam. Two nested layers give a soft falloff.
        self._los_mesh = pv.PolyData()
        self._los_actors = [
            self.plotter.add_mesh(
                self._los_mesh, color='#ffe9a8', opacity=0.12, lighting=False),
            self.plotter.add_mesh(
                self._los_mesh, color='#fff3c4', opacity=0.22, lighting=False),
        ]

        # Line of sight: a plain black line from the vehicle (cone apex) to the
        # target point (updated per frame).
        self._los_line_mesh = pv.PolyData()
        self._los_line_actor = self.plotter.add_mesh(
            self._los_line_mesh, color='black', line_width=1.5)

        self.plotter.camera_position = 'iso'
        self.plotter.enable_terrain_style(
            mouse_wheel_zooms=True,
            shift_pans=True,
        )
        self.plotter.reset_camera()

        self.plotter.camera.zoom(5.0)
    # -- scene construction ------------------------------------------------

    def build_static_scene(self, config):
        '''Build (or rebuild) the target marker and reference orbit ring.'''
        if getattr(self, '_target_actor', None) is not None:
            self.plotter.remove_actor(self._target_actor)
        if getattr(self, '_orbit_actor', None) is not None:
            self.plotter.remove_actor(self._orbit_actor)

        target = np.asarray(config.target_position, dtype=float)
        self._target_pos = target
        # Tube lying horizontally, centred on the target position (x/y/z).
        marker = pv.Cylinder(
            center=(target[0], target[1], target[2]),
            direction=(1.0, 0.0, 0.0), radius=_TUBE_RADIUS, height=1.0)
        self._target_actor = self.plotter.add_mesh(
            marker, color='#e8821e', smooth_shading=True)

        # World-fixed altitude: the orbit plane is at absolute world z = altitude,
        # centred on the target's x/y (radius/azimuth are relative to the target).
        theta = np.linspace(0.0, 2.0 * np.pi, 200)
        orbit_z = config.altitude
        ring_pts = np.column_stack((
            target[0] + config.radius * np.cos(theta),
            target[1] + config.radius * np.sin(theta),
            np.full_like(theta, orbit_z)))
        ring = pv.lines_from_points(ring_pts, close=True)
        self._orbit_actor = self.plotter.add_mesh(
            ring, color='#888888', line_width=1.5)

    def update_orbit(self, config):
        '''Refresh the orbit ring / target after a live config change.'''
        self.build_static_scene(config)

    # -- per-frame updates -------------------------------------------------

    def update_pose(self, pose):
        '''@brief Move the triad/mesh, extend the trail and update the LOS cone.'''
        T = _transform_matrix(pose)
        # Display frame: +z away from the target, matching the vehicle mesh.
        Td = T @ _DISPLAY_FLIP
        for actor in self._triad_actors:
            actor.user_matrix = Td
        for actor, offset in self._vehicle_actors:
            actor.user_matrix = T @ offset

        self._update_los_cone(Td)

        pos = np.asarray(pose[0:3], dtype=float)
        self._trail.append(pos)
        if len(self._trail) > self.trail_length:
            del self._trail[0]
        if len(self._trail) >= 2:
            self._trail_mesh.copy_from(pv.lines_from_points(np.array(self._trail)))

    def _update_los_cone(self, transform):
        '''Rebuild the light cone and line of sight: apex at the vehicle, emitted
        along the observer frame's z axis (observer_rotmat), with the footprint
        lying flat on the floor plane where the beam lands (the projected
        spotlight).'''
        apex = transform[:3, 3]
        # Observer/camera line of sight = observer frame z axis in the base link,
        # rotated into world (transform[:3, :3] is the raw body rotation).
        beam = transform[:3, :3] @ (self._observer_rotmat @ np.array([0.0, 0.0, 1.0]))
        norm = float(np.linalg.norm(beam))
        if norm < 1e-9:
            self._los_mesh.copy_from(pv.PolyData())
            self._los_line_mesh.copy_from(pv.PolyData())
            return
        beam = beam / norm
        # Beam must point downward to reach the floor; otherwise hide the cone/line.
        if beam[2] > -1e-6:
            self._los_mesh.copy_from(pv.PolyData())
            self._los_line_mesh.copy_from(pv.PolyData())
            return
        t = (_FLOOR_Z - apex[2]) / beam[2]   # distance along beam to the floor
        center = apex + t * beam             # footprint centre on the floor
        radius = t * np.tan(np.radians(_LOS_HALF_ANGLE_DEG))

        # Line of sight: from the vehicle (apex) along the beam to the floor.
        self._los_line_mesh.copy_from(pv.lines_from_points(np.array([apex, center])))

        n = 48
        phi = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        rim = np.column_stack((
            center[0] + radius * np.cos(phi),
            center[1] + radius * np.sin(phi),
            np.full(n, _FLOOR_Z)))          # footprint flat on the floor plane
        pts = np.vstack((apex, rim))         # point 0 = apex, 1..n = rim
        faces = np.empty((n, 4), dtype=np.int64)
        faces[:, 0] = 3
        faces[:, 1] = 0
        faces[:, 2] = 1 + np.arange(n)
        faces[:, 3] = 1 + (np.arange(n) + 1) % n
        self._los_mesh.copy_from(pv.PolyData(pts, faces.ravel()))

    def clear_trail(self):
        '''@brief Remove the travelled-path trail.'''
        self._trail = []
        self._trail_mesh.copy_from(pv.PolyData())

    # -- mesh hook (future) ------------------------------------------------

    def set_vehicle_mesh(self, path, offset_matrix=None, **mesh_kwargs):
        '''Load a mesh and rigidly attach it to the vehicle frame.

        `offset_matrix` is an optional 4x4 transform from mesh frame to body
        frame (e.g. to align/scale the asset). The mesh then follows the triad.
        '''
        mesh = pv.read(path)
        # If the mesh carries baked per-vertex colours (uint8 RGB/RGBA), render
        # them directly instead of a flat colour.
        for name in mesh.point_data.keys():
            arr = mesh.point_data[name]
            if arr.dtype == np.uint8 and arr.ndim == 2 and arr.shape[1] in (3, 4):
                mesh_kwargs.pop('color', None)
                mesh_kwargs['scalars'] = name
                mesh_kwargs['rgb'] = True
                break
        actor = self.plotter.add_mesh(mesh, **mesh_kwargs)
        offset = np.eye(4) if offset_matrix is None else np.asarray(offset_matrix)
        self._vehicle_actors.append((actor, offset))
        return actor

    def render(self):
        '''@brief Redraw the viewport.'''
        self.plotter.render()
