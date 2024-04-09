import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from .common import *
import numpy as np
from matplotlib.widgets import Slider
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from .common import RigidBodyState


class RigidBodyTrajectoryPlot:
    def __init__(self, sample_times, state_trajectory, input_trajectory, workspace=None, ori_plot='quaternion', quat_reference=None):
        zdown = False
        
        spc = 0.05 # spacing between axes
        graph_w = 0.25
        graph_h = 0.4
        slider_thickness = 0.03
        fig = plt.figure('6 DoF Rigid Body Visualization', (16, 8))
        ax_3d = fig.add_axes([0.6, 0.1, .4, .8], projection='3d')
        ax_pos = fig.add_axes([spc, 0.5+spc, graph_w, graph_h])
        ax_ori = fig.add_axes([0.3+spc, 0.5+spc, graph_w, graph_h])
        ax_vlin = fig.add_axes([spc, slider_thickness+spc, graph_w, graph_h])
        ax_vang = fig.add_axes([0.3+spc, slider_thickness+spc, graph_w, graph_h])
        ax_slider = fig.add_axes([0.1, 0.01, 0.8, slider_thickness-0.01])

        for ax in [ax_pos, ax_ori, ax_vlin, ax_vang]:
            ax.set_prop_cycle(color=['red', 'green', 'blue', 'magenta', 'olive', 'dodgerblue'])

        # # Extract Data

        t = sample_times
        val = state_trajectory

        pos  = val[:,0:3]
        ori  = val[:,3:7]
        vlin = val[:,7:10]
        vang = val[:,10:13]

        # # Plot position in 3d

        ax_3d.plot3D(pos[:,0], pos[:,1], pos[:,2], color=[0.3, 0.3, 0.3])

        # Determine axes limits for 3d Plot
        ax_3d_limits = [np.min(pos,axis=0), np.max(pos,axis=0)]
        ax_3d_xyz_ranges = ax_3d_limits[1] - ax_3d_limits[0]
        ax_3d_xyz_middle = 0.5 * (ax_3d_limits[1] + ax_3d_limits[0])
        ax_3d_max_range = np.max(ax_3d_xyz_ranges)
        ax_3d_max_range = max(ax_3d_max_range, 1) # set minimum window size to 1

        if workspace is None:
            workspace = []
            for i in range(3):
                b = np.array([-1, 1])
                lim = ax_3d_xyz_middle[i] + b * ax_3d_max_range/2
                if i != 0 and zdown:
                    # flip y and z axes s.t. z points downwards
                    lim = np.flip(lim)
                workspace.append(lim)
            
            # z=0.0 is the water plane
            #workspace[2][1] = 0.0


        # set limits
        ax_3d.set_xlim(-workspace[0], workspace[0]);
        ax_3d.set_ylim(-workspace[1], workspace[1])
        ax_3d.set_zlim(-workspace[2], 0)
        ax_3d.set_xlabel('x'); ax_3d.set_ylabel('y'); ax_3d.set_zlabel('z')

        # # Plot graphs
        # lines = ax_pos.plot(t, pos)
        # ax_pos.legend(lines, ['x', 'y', 'z'], loc="upper right")
        # ax_pos.set_ylabel('position [m]')
        # ax_pos.set_ylim(-10,10)

        lines = ax_pos.plot(t, input_trajectory)
        ax_pos.legend(lines, ['X', 'Y', 'Z', 'K', 'M', 'N'], loc="upper right")
        ax_pos.set_ylabel('Wrench [N and Nm]')
        #ax_pos.set_ylim(-100,100)

        if ori_plot=='quaternion':
            lines = ax_ori.plot(t, ori[:,0], color='r', label='qx')
            lines = ax_ori.plot(t, ori[:,1], color='b', label='qy')
            lines = ax_ori.plot(t, ori[:,2], color='k', label='qz')
            lines = ax_ori.plot(t, ori[:,3], color='g', label='qw')

            if quat_reference is not None:
                lines = ax_ori.plot(t, quat_reference[:,0], color='r', ls='--')
                lines = ax_ori.plot(t, quat_reference[:,1], color='b', ls='--')
                lines = ax_ori.plot(t, quat_reference[:,2], color='k', ls='--')
                lines = ax_ori.plot(t, quat_reference[:,3], color='g', ls='--')
            ax_ori.legend(loc="upper right")
            ax_ori.set_ylabel('quaternions')
        elif ori_plot == 'euler':
            #ori_rpy = quat2rpy_array("xyz", ori, True)
            lines = ax_ori.plot(t, ori_rpy)
            ax_ori.legend(lines, ['r', 'p', 'y'], loc="upper right")
            ax_ori.set_ylabel('euler angles [deg.]')

        lines = ax_vlin.plot(t, vlin)
        ax_vlin.legend(lines, ['u', 'v', 'w'], loc="upper right")
        ax_vlin.set_ylabel('lin. vel. [m/s]')
        ax_vlin.set_ylim(-0.6, +0.6)

        lines = ax_vang.plot(t, vang)
        ax_vang.legend(lines, ['p', 'q', 'r'], loc="upper right")
        ax_vang.set_ylabel('ang. vel. [rad/s]')
        ax_vang.set_ylim(-0.3, +0.3)

        # # Interactive

        # add slider
        global t_slider
        t_slider = Slider(ax_slider, 'time', t[0], t[-1],
                          valinit=t[0],
                          valstep=(t[-1]-t[0])/1000.0 )

        # plot coordinate system at initial pose
        pose_0 = np.hstack((pos[0,:], ori[0,:]))
        #pose_marker = Cartesian3d(pose_0, ax_3d, length=(ax_3d_max_range)/5)
        pose_marker = Cartesian3d(pose_0, ax_3d, length=1.5)

        # add graph marker
        graph_marker = []
        marker_axes_list = []
        marker_axes_list.extend([ax_pos, ax_ori, ax_vlin, ax_vang])
        #marker_axes_list.extend(add_marker_axes)
        for ax in marker_axes_list:
            l = ax.axvline(0, color='k', linestyle='--')
            graph_marker.append(l)

        def update(val):

            timestamp = t_slider.val
            idx = np.argmin(np.abs(t - timestamp))

            pose_0 = np.hstack((pos[idx,:], ori[idx,:]))
            pose_marker.redraw(pose_0)

            for l in graph_marker:
                l.set_xdata([timestamp, timestamp])

            fig.canvas.draw_idle()


        t_slider.on_changed(update)

#############################
# RIGID BODY PLOTTING
#############################

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))

        return np.min(zs)

class Cartesian3d:
    def __init__(self, pose, ax, length=1, alpha=1):
        self.length = length
        self.ax = ax

        origin = pose[0:3]
        rot_ib = quat2rot(pose[3:7])

        base_x = origin + rot_ib.dot(np.asarray([1.5*self.length, 0, 0]))
        base_y = origin + rot_ib.dot(np.asarray([0, self.length, 0]))
        base_z = origin + rot_ib.dot(np.asarray([0, 0, self.length]))


        base_vecs=[base_x, base_y, base_z]
        colors=['r','g','b']

        self.lines = []

        for i in range(3):
            xs = [origin[0], base_vecs[i][0]]
            ys = [origin[1], base_vecs[i][1]]
            zs = [origin[2], base_vecs[i][2]]
            c = colors[i]

            l = Arrow3D(xs, ys, zs, mutation_scale=20, lw=2, arrowstyle="->", color=c, alpha=alpha)
            self.lines.append(l)
            ax.add_artist(l)


    def redraw(self, pose):
        origin = pose[0:3]
        rot_ib = quat2rot(pose[3:7])

        base_x = origin + rot_ib.dot(np.asarray([1.5*self.length, 0, 0]))
        base_y = origin + rot_ib.dot(np.asarray([0, self.length, 0]))
        base_z = origin + rot_ib.dot(np.asarray([0, 0, self.length]))

        base_vecs=[base_x, base_y, base_z]

        for i in range(3):
            xs = [origin[0], base_vecs[i][0]]
            ys = [origin[1], base_vecs[i][1]]
            zs = [origin[2], base_vecs[i][2]]

            self.lines[i]._verts3d = xs, ys, zs
