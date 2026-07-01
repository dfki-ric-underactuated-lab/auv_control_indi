'''
Interactive 360 degrees orbit-inspection demo.

Runs the orbit simulation in real time in a Qt + VTK desktop GUI. You can:

  * enable/disable individual thrusters live (green = enabled, red = failed) to
    watch INDI-QP reallocate control effort under (passive) actuator failure;
  * reconfigure the orbit (radius, altitude, tangential velocity, relative
    motion, target position) and the accelerometer noise on the fly;
  * change the actuator delay and restart;
  * watch live tracking-error and tangential-velocity plots.

    python3 start_interactive_sim.py

Layout: 3D viewport (centre) with the vehicle orientation triad, target object,
reference orbit and travelled trail; configuration panel (right); live charts
(bottom); thruster toggles and a Play/Pause/Reset toolbar.

Requires PySide6, pyvista and pyvistaqt (see requirements.txt).
'''

import sys
import os

# On Wayland, Qt would use the wayland platform plugin while VTK creates an
# X/GLX window; the mismatch crashes on the first window configure
# ("X Error ... BadWindow ... X_ConfigureWindow"). Force Qt onto xcb so both
# render through X. Must run before any Qt import. Override by setting
# QT_QPA_PLATFORM yourself.
if os.environ.get('XDG_SESSION_TYPE') == 'wayland' and 'QT_QPA_PLATFORM' not in os.environ:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from uwdrake.systems.orbit_simulation import OrbitSimConfig
from uwdrake.gui import OrbitWindow


def apply_dark_theme(app):
    '''Dark Fusion palette for all Qt widgets (3D/plot views keep own colours).'''
    app.setStyle('Fusion')
    p = QPalette()
    bg, base, text = QColor(45, 45, 45), QColor(30, 30, 30), QColor(220, 220, 220)
    p.setColor(QPalette.Window, bg)
    p.setColor(QPalette.WindowText, text)
    p.setColor(QPalette.Base, base)
    p.setColor(QPalette.AlternateBase, bg)
    p.setColor(QPalette.ToolTipBase, base)
    p.setColor(QPalette.ToolTipText, text)
    p.setColor(QPalette.Text, text)
    p.setColor(QPalette.Button, bg)
    p.setColor(QPalette.ButtonText, text)
    p.setColor(QPalette.Highlight, QColor(53, 132, 228))
    p.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    # Disabled group, so greyed-out widgets (e.g. the azimuth slider when
    # continuous motion is on) actually look disabled.
    dim = QColor(110, 110, 110)
    p.setColor(QPalette.Disabled, QPalette.Text, dim)
    p.setColor(QPalette.Disabled, QPalette.ButtonText, dim)
    p.setColor(QPalette.Disabled, QPalette.WindowText, dim)
    p.setColor(QPalette.Disabled, QPalette.Button, QColor(40, 40, 40))
    p.setColor(QPalette.Disabled, QPalette.Base, QColor(40, 40, 40))
    p.setColor(QPalette.Disabled, QPalette.Highlight, QColor(70, 70, 70))
    p.setColor(QPalette.Disabled, QPalette.HighlightedText, dim)
    app.setPalette(p)

# Persistent setup for the live demo lives in configs/interactive.yml. Faults
# are driven by the buttons here, so no fault is scheduled on the FaultScheduler
# (it stays inert). Note: altitude is world-fixed in the live demo (absolute
# world z), so altitude -5 m = 5 m depth, and the target z sits on the seafloor.
config = OrbitSimConfig.from_yaml('configs/interactive.yml')

app = QApplication(sys.argv)
apply_dark_theme(app)
window = OrbitWindow(config)
window.show()
sys.exit(app.exec())
