'''
Qt + VTK desktop GUI for the interactive orbit-inspection demo.

Public entry point:

    from uwdrake.gui import OrbitWindow
    from uwdrake.systems.orbit_simulation import OrbitSimConfig

    app = QApplication(sys.argv)
    win = OrbitWindow(OrbitSimConfig(controller_type='INDI'))
    win.show()
    app.exec()
'''

from .orbit_window import OrbitWindow
from .sim_driver import OrbitSimDriver

__all__ = ['OrbitWindow', 'OrbitSimDriver']
