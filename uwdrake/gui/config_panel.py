'''
Configuration side panel for the orbit GUI.

Exposes the live-tunable orbit/motion and noise parameters as native Qt widgets.
Orbit/motion, target and accelerometer-noise changes are applied to the driver
immediately and announce `liveChanged` (so the window can refresh the rendered
orbit ring / target). The actuator delay is baked into the diagram at build time,
so it sits behind an "Apply (restart)" button that emits `rebuildRequested`.
'''

import numpy as np
from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QGroupBox, QDoubleSpinBox, QSpinBox,
    QCheckBox, QPushButton, QHBoxLayout, QSlider, QLabel)


def _dspin(minimum, maximum, step, value, decimals=2):
    '''@brief Convenience constructor for a configured QDoubleSpinBox.'''
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setSingleStep(step)
    box.setDecimals(decimals)
    box.setValue(value)
    return box


class FloatSlider(QWidget):
    '''A horizontal slider over a float range with a live value readout.

    Exposes the same `value()` / `valueChanged` interface as a spin box so it
    drops into the existing handlers. `QSlider` is integer-only, so positions
    are mapped to floats via a fixed step.
    '''
    valueChanged = Signal(float)

    def __init__(self, minimum, maximum, step, value, decimals=3, parent=None):
        super().__init__(parent)
        self._min = minimum
        self._step = step
        self._decimals = decimals
        self._steps = int(round((maximum - minimum) / step))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self._steps)
        self.readout = QLabel()
        self.readout.setMinimumWidth(48)
        self.readout.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.slider, 1)
        row.addWidget(self.readout)

        self.slider.valueChanged.connect(self._on_slider)
        self.setValue(value)

    def value(self):
        '''@brief Current slider value as a float.'''
        return self._min + self.slider.value() * self._step

    def setValue(self, value):
        '''@brief Set the slider to the nearest step to @p value.'''
        pos = int(round((value - self._min) / self._step))
        self.slider.setValue(max(0, min(self._steps, pos)))
        self.readout.setText(f'{self.value():.{self._decimals}f}')

    def _on_slider(self, _pos):
        self.readout.setText(f'{self.value():.{self._decimals}f}')
        self.valueChanged.emit(self.value())


class ConfigPanel(QWidget):
    ## @brief Side panel of live-tunable orbit/noise widgets bound to the driver.
    liveChanged = Signal()          # a live param was applied to the driver
    rebuildRequested = Signal()     # actuator delay changed -> rebuild needed

    def __init__(self, driver, parent=None):
        '''@brief Build the widgets from the driver's current config.'''
        super().__init__(parent)
        self.driver = driver
        cfg = driver.config

        layout = QVBoxLayout(self)

        # -- Orbit & motion ------------------------------------------------
        orbit = QGroupBox('Orbit && motion')
        form = QFormLayout(orbit)

        self.radius = _dspin(0.5, 6.0, 0.1, float(cfg.radius))
        self.altitude = _dspin(-10.0, 10.0, 0.1, float(cfg.altitude))
        # Tangential speed magnitude; its direction is set by the Reverse box.
        self.tangential = FloatSlider(0.0, 0.5, 0.01, abs(float(cfg.tangential_velocity)), 3)
        self.continuous = QCheckBox()
        self.continuous.setChecked(bool(cfg.continuous_motion))

        # Reverse the orbit direction (negates the tangential velocity); only
        # meaningful while continuous motion is on.
        self.reverse = QCheckBox()
        self.reverse.setChecked(float(cfg.tangential_velocity) < 0.0)

        # Fixed azimuth setpoint [deg], only used when continuous motion is off.
        az_deg = (np.degrees(float(cfg.azimuth)) + 180.0) % 360.0 - 180.0
        self.azimuth = FloatSlider(-180.0, 180.0, 1.0, az_deg, 0)

        target = np.asarray(cfg.target_position, dtype=float)
        self.target_x = _dspin(-10.0, 10.0, 0.5, target[0])
        self.target_y = _dspin(-10.0, 10.0, 0.5, target[1])
        self.target_z = _dspin(-10.0, 10.0, 0.5, target[2])

        form.addRow('Radius [m]', self.radius)
        form.addRow('Altitude [m]', self.altitude)
        form.addRow('Tangential vel. [m/s]', self.tangential)
        form.addRow('Continuous motion', self.continuous)
        self._reverse_label = QLabel('Reverse direction')
        form.addRow(self._reverse_label, self.reverse)
        self._azimuth_label = QLabel('Azimuth [deg]')
        form.addRow(self._azimuth_label, self.azimuth)
        form.addRow('Target x [m]', self.target_x)
        form.addRow('Target y [m]', self.target_y)
        form.addRow('Target z [m]', self.target_z)
        layout.addWidget(orbit)

        for box in (self.radius, self.altitude, self.tangential, self.azimuth,
                    self.target_x, self.target_y, self.target_z):
            box.valueChanged.connect(self._apply_orbit)
        self.reverse.toggled.connect(self._apply_orbit)
        self.continuous.toggled.connect(self._apply_orbit)
        self.continuous.toggled.connect(self._update_mode_enabled)
        self._update_mode_enabled()

        # -- Noise & disturbances -----------------------------------------
        noise = QGroupBox('Noise && disturbances')
        nform = QFormLayout(noise)

        self.acc_noise = FloatSlider(0.0, 2.0, 0.005, float(np.ravel(cfg.acc_disturbance_stddev)[0]), 3)
        self.acc_noise.valueChanged.connect(self._apply_noise)

        nform.addRow('Accel. noise σ', self.acc_noise)

        # Actuator delay needs a rebuild -> grouped with its own apply button.
        self.delay = QSpinBox()
        self.delay.setRange(0, 100)
        self.delay.setValue(int(cfg.act_delay_steps))
        apply_row = QHBoxLayout()
        apply_btn = QPushButton('Apply (restart)')
        apply_btn.clicked.connect(self._apply_delay)
        apply_row.addWidget(self.delay)
        apply_row.addWidget(apply_btn)
        nform.addRow('Actuator delay [steps]', apply_row)

        layout.addWidget(noise)
        layout.addStretch(1)

    # -- handlers ----------------------------------------------------------

    def _apply_orbit(self, *_):
        '''@brief Push orbit/target widget values to the driver and announce the change.'''
        tangential = self.tangential.value()
        if self.reverse.isChecked():
            tangential = -tangential
        self.driver.set_orbit_params(
            radius=self.radius.value(),
            altitude=self.altitude.value(),
            tangential_velocity=tangential,
            continuous_motion=self.continuous.isChecked(),
            azimuth=np.radians(self.azimuth.value()))
        self.driver.set_target_position(
            (self.target_x.value(), self.target_y.value(), self.target_z.value()))
        self.liveChanged.emit()

    def _update_mode_enabled(self, *_):
        '''Reverse only applies while orbiting; the fixed azimuth setpoint only
        applies when continuous motion is off. Enable each in its own mode.'''
        continuous = self.continuous.isChecked()
        self.reverse.setEnabled(continuous)
        self._reverse_label.setEnabled(continuous)
        self.azimuth.setEnabled(not continuous)
        self._azimuth_label.setEnabled(not continuous)

    def _apply_noise(self, *_):
        '''@brief Push the accelerometer-noise value to the driver.'''
        self.driver.set_accelerometer_stddev(self.acc_noise.value())

    def _apply_delay(self):
        '''@brief Apply the actuator delay (requires a diagram rebuild).'''
        self.driver.set_actuator_delay_steps(self.delay.value())
        self.rebuildRequested.emit()
