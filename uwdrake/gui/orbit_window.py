'''
Main window for the interactive orbit-inspection demo.

Assembles the 3D viewport (centre), the config panel (right dock) and the live
charts (bottom dock), with a row of per-thruster toggle buttons and a
Play/Pause/Reset toolbar. A `QTimer` drives the simulation in real time using
the wall-clock delta between frames, so the sim tracks real time regardless of
redraw speed.
'''

import os
import time

import numpy as np
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton,
    QLabel, QDockWidget, QToolBar, QScrollArea)

from .sim_driver import OrbitSimDriver
from .viewport3d import Viewport3D
from .charts import LiveChart
from .config_panel import ConfigPanel

_ENABLED_STYLE = 'background-color: #74c476; color: black; font-weight: bold;'
_DISABLED_STYLE = 'background-color: #e06666; color: black; font-weight: bold;'

# Cadence (frames) at which samples are pushed to the rolling charts.
_CHART_EVERY = 3


class OrbitWindow(QMainWindow):
    ## @brief Main window: 3D viewport + config/charts/thruster docks, driven by a
    #  ~60 Hz timer that advances the simulation in real time.
    def __init__(self, config):
        '''@brief Assemble the docks and start the real-time simulation timer.'''
        super().__init__()
        self.setWindowTitle('Cuttlefish — interactive INDI-QP orbit')
        self.resize(1920, 1080)

        self.driver = OrbitSimDriver(config)
        self._paused = False
        self._last_wall = None
        self._frame = 0

        # 3D viewport fills the centre; thrusters live in a right-side dock.
        self.viewport = Viewport3D(config, parent=self)
        self._load_vehicle_mesh(config)
        self.setCentralWidget(self.viewport.plotter)

        self._build_toolbar()
        self._build_config_dock()
        self._build_charts_dock()
        self._build_thruster_dock()

        self.viewport.update_pose(self.driver.pose())

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)  # ~60 Hz

    # -- widget construction ----------------------------------------------

    def _load_vehicle_mesh(self, config):
        '''Attach the visual vehicle mesh if configured and available.

        The mesh follows the same (display-flipped) body frame as the triad. If
        it loads upside-down/back-to-front, pass an `offset_matrix` to
        `Viewport3D.set_vehicle_mesh` to correct it.
        '''
        path = getattr(config, 'vehicle_mesh_path', None)
        if not path:
            return
        path = config._resolve(path)
        if not os.path.exists(path):
            return
        # Rotate the mesh 180 deg about its body y-axis so its z-axis points away
        # from the target (opposite the display-flipped triad's z).
        flip_y = np.diag([-1.0, 1.0, -1.0, 1.0])
        try:
            self.viewport.set_vehicle_mesh(
                path, offset_matrix=flip_y, color='#c9d1d9',
                smooth_shading=True, opacity=1.0)
        except Exception as exc:  # noqa: BLE001 - degrade to triad-only on failure
            print(f'[orbit_window] could not load vehicle mesh {path}: {exc}')

    # Thruster button layout, top view with the bow ("front") up. Each corner
    # gives its grid cell (row, col) and its buttons left-to-right as
    # (thruster index, label): horizontal thruster on the outer edge, vertical
    # toward the centre. Indices follow the model's thruster order (see
    # cuttlefish_thrusters_model).
    _CORNER_LAYOUT = (
        ('Front Left',  0, 0, ((4, 'H'), (0, 'V'))),
        ('Front Right', 0, 2, ((1, 'V'), (5, 'H'))),
        ('Tail Left',   2, 0, ((7, 'H'), (3, 'V'))),
        ('Tail Right',  2, 2, ((2, 'V'), (6, 'H'))),
    )

    def _build_thruster_row(self):
        '''@brief Build the corner-arranged grid of per-thruster toggle buttons.'''
        # Keep the grid at its natural height (top-aligned) so spare vertical
        # space in the dock goes to the trailing stretch, not the buttons.
        panel = QWidget()
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(8, 6, 8, 6)
        grid = QGridLayout()
        grid.setHorizontalSpacing(24)
        grid.setVerticalSpacing(4)
        outer.addLayout(grid)
        outer.addStretch(1)

        names = self.driver.thruster_names
        self._thruster_buttons = [None] * self.driver.n_thrusters
        for label_text, r, c, indices in self._CORNER_LAYOUT:
            align = (Qt.AlignRight if c == 0 else Qt.AlignLeft) | \
                    (Qt.AlignBottom if r == 0 else Qt.AlignTop)
            box = QVBoxLayout()
            box.setSpacing(4)
            label = QLabel(label_text)
            label.setStyleSheet('font-weight: bold; color: #888;')
            label.setAlignment(Qt.AlignHCenter)
            box.addWidget(label)
            btn_row = QHBoxLayout()  # square buttons side by side under the label
            btn_row.setSpacing(4)
            for i, axis in indices:
                btn = QPushButton(axis)
                btn.setToolTip(names[i])
                btn.setCheckable(True)
                btn.setFixedSize(64, 64)  # 1:1 aspect ratio
                btn.setChecked(self.driver.is_enabled(i))
                btn.toggled.connect(
                    lambda checked, idx=i: self._toggle_thruster(idx, checked))
                self._thruster_buttons[i] = btn
                btn_row.addWidget(btn)
                self._style_thruster(i)
            box.addLayout(btn_row)
            grid.addLayout(box, r, c, alignment=align)

        # A "▲ front" marker in the centre-top cell orients the layout.
        front = QLabel('▲ front')
        front.setAlignment(Qt.AlignCenter)
        front.setStyleSheet('color: #888;')
        grid.addWidget(front, 0, 1, alignment=Qt.AlignTop)

        grid.setColumnStretch(1, 1)
        grid.setRowMinimumHeight(1, 72)  # white space between front and tail
        return panel

    def _build_toolbar(self):
        '''@brief Build the Pause/Reset toolbar with the state and HUD labels.'''
        bar = QToolBar('Controls')
        self.addToolBar(bar)
        self.play_action = bar.addAction('Pause', self._toggle_pause)
        bar.addAction('Reset', self._reset)
        bar.addSeparator()
        self.state_label = QLabel('')
        bar.addWidget(self.state_label)
        bar.addSeparator()
        self.hud = QLabel('')
        bar.addWidget(self.hud)

    # Toolbar colour per controller state-machine status.
    _STATE_COLORS = {
        'RUNNING': '#74c476',
        'WAITING': '#cccccc',
        'ALIGNMENT': '#e0a020',
    }

    def _build_config_dock(self):
        '''@brief Build the right-side configuration dock.'''
        self.config_panel = ConfigPanel(self.driver)
        self.config_panel.liveChanged.connect(
            lambda: self.viewport.update_orbit(self.driver.config))
        self.config_panel.rebuildRequested.connect(self._rebuild)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.config_panel)
        dock = QDockWidget('Configuration', self)
        dock.setWidget(scroll)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self._config_dock = dock

    def _build_thruster_dock(self):
        '''@brief Build the thruster-toggle dock under the configuration dock.'''
        dock = QDockWidget('Thrusters', self)
        panel = self._build_thruster_row()
        # Cap the panel to its natural height so the thruster dock stays compact
        # and the rest of the right column goes to the configuration panel
        # (which then no longer needs a scrollbar).
        panel.setMaximumHeight(panel.sizeHint().height())
        dock.setWidget(panel)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        # Stack it underneath the configuration dock in the right column.
        self.splitDockWidget(self._config_dock, dock, Qt.Vertical)

    def _build_charts_dock(self):
        '''@brief Build the bottom dock with the tracking-error/cone/velocity charts.'''
        self.error_chart = LiveChart(
            'Tracking error', [
                ('Radius error [m]', '#2ca02c'),
                ('Altitude error [m]', '#1f77b4')],
            parent=self, y_label='error [m]', y_range=(-1.0, 1.0))
        self.cone_chart = LiveChart(
            'Cone error', [
                ('Cone error [deg]', '#d62728')],
            parent=self, y_label='deg', y_range=(-45.0, 45.0))
        self.vel_chart = LiveChart(
            'Tangential velocity', [
                ('v_tangential [m/s]', '#1f77b4'),
                ('setpoint [m/s]', '#aaaaaa')],
            parent=self, y_label='m/s', y_range=(-0.5, 0.5))

        container = QWidget()
        container.setMaximumHeight(240)  # keep the plot dock reasonably short
        hbox = QHBoxLayout(container)
        hbox.setContentsMargins(0, 0, 0, 0)
        hbox.addWidget(self.error_chart.widget, 1)
        hbox.addWidget(self.cone_chart.widget, 1)
        hbox.addWidget(self.vel_chart.widget, 1)

        dock = QDockWidget('Live plots', self)
        dock.setWidget(container)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

    # -- simulation loop ---------------------------------------------------

    def _tick(self):
        '''@brief Timer callback: advance the sim by wall-clock dt and refresh views.'''
        now = time.perf_counter()
        if self._last_wall is None:
            self._last_wall = now
        elapsed = now - self._last_wall
        self._last_wall = now

        if not self._paused:
            self.driver.step(elapsed)

        self.viewport.update_pose(self.driver.pose())
        self.viewport.render()

        self._frame += 1
        if self._frame % _CHART_EVERY == 0:
            m = self.driver.metrics()
            t = self.driver.sim_time
            self.error_chart.append(t, (m['radius_error'], m['altitude_error']))
            self.cone_chart.append(t, (m['cone_error'],))
            setpoint = self.driver.sim.pose_controller.command.tangential_velocity
            self.vel_chart.append(t, (m['tangential_velocity'], setpoint))

        status = self.driver.control_status()
        color = self._STATE_COLORS.get(status, '#ffffff')
        self.state_label.setText(f'  state: {status}  ')
        self.state_label.setStyleSheet(f'color: {color}; font-weight: bold;')

        self.hud.setText(
            f"  t = {self.driver.sim_time:5.1f} s    "
            f"failed thrusters: {self.driver.n_failed()}/{self.driver.n_thrusters}"
            + ('    [PAUSED]' if self._paused else ''))

    # -- actions -----------------------------------------------------------

    def _toggle_thruster(self, index, checked):
        '''@brief Enable/disable a thruster from its button and restyle it.'''
        self.driver.set_thruster_enabled(index, checked)
        self._style_thruster(index)

    def _style_thruster(self, index):
        '''@brief Colour a thruster button green (enabled) or red (failed).'''
        enabled = self.driver.is_enabled(index)
        self._thruster_buttons[index].setStyleSheet(
            _ENABLED_STYLE if enabled else _DISABLED_STYLE)

    def _toggle_pause(self):
        '''@brief Toggle the paused state and update the toolbar label.'''
        self._paused = not self._paused
        self.play_action.setText('Resume' if self._paused else 'Pause')

    def _reset(self):
        '''@brief Reset the simulation to its initial state.'''
        self.driver.reset()
        self._after_restart()

    def _rebuild(self):
        '''@brief Rebuild the diagram (e.g. after an actuator-delay change).'''
        self.driver.rebuild()
        self.viewport.update_orbit(self.driver.config)
        self._after_restart()

    def _after_restart(self):
        '''@brief Sync buttons and clear trail/charts after a reset or rebuild.'''
        for i, btn in enumerate(self._thruster_buttons):
            btn.blockSignals(True)
            btn.setChecked(self.driver.is_enabled(i))
            btn.blockSignals(False)
            self._style_thruster(i)
        self.viewport.clear_trail()
        self.error_chart.reset()
        self.cone_chart.reset()
        self.vel_chart.reset()
        self._last_wall = None
        self.viewport.update_pose(self.driver.pose())

    def closeEvent(self, event):
        '''@brief Stop the timer and close the embedded VTK render windows.'''
        self.timer.stop()
        self.viewport.plotter.close()
        self.error_chart.plotter.close()
        self.cone_chart.plotter.close()
        self.vel_chart.plotter.close()
        super().closeEvent(event)
