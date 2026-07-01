'''
Live line charts for the orbit GUI.

`LiveChart` wraps a `pyvistaqt.QtInteractor` hosting a `pyvista.ChartXY` (which
is a thin wrapper over VTK's `vtkChartXY`). Using the same pyvistaqt embedding as
the 3D viewport keeps the whole GUI on one proven VTK/Qt integration path and
avoids the brittle low-level `QVTKRenderWindowInteractor` + `vtkContextView`
combination. Data is held in capped Python buffers and pushed to the chart lines
each update.
'''

import pyvista as pv
from pyvistaqt import QtInteractor


class LiveChart:
    '''A rolling line chart with one or more series sharing a time axis.'''

    def __init__(self, title, series, parent=None, window=30.0,
                 x_label='time [s]', y_label='', y_range=None):
        '''
        series: list of (name, hex_color) tuples, in draw order.
        window: width of the moving time window shown on the x-axis [s].
        y_range: optional (min, max) to pin the y-axis, else autoscale.
        '''
        self.series = series
        self.window = window
        self._t = [0.0]
        self._cols = [[0.0] for _ in series]

        self.plotter = QtInteractor(parent)
        self.plotter.set_background('#1e1e1e')

        self.chart = pv.Chart2D()  # pyvista's vtkChartXY wrapper
        self.chart.title = title
        self.chart.x_label = x_label
        self.chart.y_label = y_label
        self.chart.x_axis.range = [0.0, window]
        if y_range is not None:
            self.chart.y_axis.range = list(y_range)
        self._apply_dark_theme()

        self._lines = []
        for (name, color), col in zip(series, self._cols):
            line = self.chart.line(self._t, col, color=color, width=2.0, label=name)
            self._lines.append(line)
        self.chart.legend_visible = len(series) > 1

        self.plotter.add_chart(self.chart)

    def _apply_dark_theme(self):
        '''Dark chart background with light axes, labels and title.'''
        self.chart.background_color = '#1e1e1e'
        light = (0.86, 0.86, 0.86)
        self.chart.GetTitleProperties().SetColor(*light)
        for axis in (self.chart.x_axis, self.chart.y_axis):
            axis.pen.color = '#d6d6d6'        # axis line + ticks
            axis.grid_pen.color = '#3a3a3a'   # grid lines
            axis.GetLabelProperties().SetColor(*light)   # tick labels
            axis.GetTitleProperties().SetColor(*light)   # axis title

        # Legend: dark background with light text (default is the opposite).
        legend = self.chart.GetLegend()
        legend.GetBrush().SetColorF(0.118, 0.118, 0.118)  # dark fill
        legend.GetBrush().SetOpacityF(1.0)
        legend.GetPen().SetColorF(0.30, 0.30, 0.30)       # border
        legend.GetLabelProperties().SetColor(*light)      # text

    @property
    def widget(self):
        '''@brief The embeddable Qt widget hosting the chart.'''
        return self.plotter

    def append(self, t, values):
        '''Append one sample and scroll the fixed-width time window.'''
        t = float(t)
        self._t.append(t)
        for col, v in zip(self._cols, values):
            col.append(float(v))

        # Drop samples that fall outside the moving window (keep one extra in
        # front so the line stays connected at the left edge).
        cutoff = t - self.window
        while len(self._t) > 2 and self._t[1] < cutoff:
            self._t.pop(0)
            for col in self._cols:
                col.pop(0)

        for line, col in zip(self._lines, self._cols):
            line.update(self._t, col)

        hi = max(t, self.window)
        self.chart.x_axis.range = [hi - self.window, hi]
        self.plotter.render()

    def reset(self):
        '''@brief Clear all series and reset the time window.'''
        self._t = [0.0]
        self._cols = [[0.0] for _ in self.series]
        for line, col in zip(self._lines, self._cols):
            line.update(self._t, col)
        self.chart.x_axis.range = [0.0, self.window]
        self.plotter.render()
