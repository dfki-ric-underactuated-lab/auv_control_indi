##
# @file
# @brief Post-processing: tracking-error metrics, summary tables and result plots.

import math
import pandas as pd
import numpy as np
import matplotlib

import matplotlib.pyplot as plt

from .common import quat2rpy_array, quaternion_conjugate, normalize_angle_radians, quat2rot, quaternion_product

# Display arrangement of the 8 thruster subplots (indices into the thruster
# vector). The human-readable names live on the propulsion model
# (PropulsionModel.thruster_names, loaded from YAML).
THRUSTER_DISPLAY_ORDER = [4, 0, 1, 5, 7, 3, 2, 6]


def calculate_conic_angle_error(ori_ref, ori_cur):
    '''@brief Off-axis angle [deg] between the current and reference look-at vectors.'''
    lookat_vectors     = np.empty((ori_cur.shape[0], 3))
    lookat_vectors_ref = np.empty((ori_cur.shape[0], 3))

    for i in range(ori_cur.shape[0]):
        lookat_vectors[i,:]     = quat2rot(ori_cur[i,:]).dot(np.array([0,0,1]))
        lookat_vectors_ref[i,:] = quat2rot(ori_ref[i,:]).dot(np.array([0,0,1]))

    lookat_vectors   = lookat_vectors / np.linalg.norm(lookat_vectors, axis=1, keepdims=True)
    lookat_vectors_ref = lookat_vectors_ref / np.linalg.norm(lookat_vectors_ref, axis=1, keepdims=True)
    dot_products     = np.sum(lookat_vectors * lookat_vectors_ref, axis=1)
    cone_error       = np.arccos(dot_products) * 180.0 / math.pi

    return cone_error

def create_orbit_summary_dataframe(thruster_selection, polar_velocity, ori_ref, ori_cur, polar_ref, polar_data, cone_error, vel_controller):
    '''@brief One-row DataFrame of RMS/max tracking errors and the QP weights for a run.'''
    polar_error = polar_ref - polar_data

    max_radius_error     = polar_error[np.argmax(np.abs(polar_error[:, 0])), 0]
    rmse_radius_error    = np.sqrt(np.mean(np.power(polar_error[:, 0], 2)))
    max_altitude_error   = polar_error[np.argmax(np.abs(polar_error[:, 2])), 2]
    rmse_altitude_error  = np.sqrt(np.mean(np.power(polar_error[:, 2], 2)))

    # zyz: Upright pose -> Yaw is roll, roll is heading, pitch is pitch.
    rpy_ref_array   = quat2rpy_array('xyz', ori_ref, False)
    rpy_cur_array   = quat2rpy_array('xyz', ori_cur, False)
    rpy_error_array = normalize_angle_radians(rpy_ref_array - rpy_cur_array) * 180.0 / math.pi

    max_roll_error   = rpy_error_array[np.argmax(np.abs(rpy_error_array[:, 0])), 0]
    rmse_roll_error  = np.sqrt(np.mean(np.power(rpy_error_array[:, 0], 2)))

    max_pitch_error  = rpy_error_array[np.argmax(np.abs(rpy_error_array[:, 1])), 1]
    rmse_pitch_error = np.sqrt(np.mean(np.power(rpy_error_array[:, 1], 2)))

    max_yaw_error    = rpy_error_array[np.argmax(np.abs(rpy_error_array[:, 2])), 2]
    rmse_yaw_error   = np.sqrt(np.mean(np.power(rpy_error_array[:, 2], 2)))

    rmse_cone_error  = np.sqrt(np.mean(np.power(cone_error, 2)))
    max_cone_error   = cone_error[np.argmax(np.abs(cone_error))]

    rmse_vel_t  = np.sqrt(np.mean(np.power(polar_velocity[:, 1], 2)))


    ori_curcon = quaternion_conjugate(ori_cur.T).T
    delta_quat = quaternion_product(ori_ref[:,0], ori_ref[:,1], ori_ref[:,2], ori_ref[:,3], ori_curcon[:,0], ori_curcon[:,1], ori_curcon[:,2], ori_curcon[:,3]).T
    rotz = np.arctan2(2*(delta_quat[:,3]*delta_quat[:,2] + delta_quat[:,0]*delta_quat[:,1]), 1 - 2*(delta_quat[:,1]**2 + delta_quat[:,2]**2))
    rotz_rms = np.sqrt(np.mean(np.power(rotz, 2)))


    Wx = vel_controller.control_allocation.W_x

    df = pd.DataFrame({
              'VFL':                thruster_selection[0],
              'VFR':                thruster_selection[1],
              'VTR':                thruster_selection[2],
              'VTL':                thruster_selection[3],
              'HFL':                thruster_selection[4],
              'HFR':                thruster_selection[5],
              'HTR':                thruster_selection[6],
              'HTL':                thruster_selection[7],
              'RMS Tan Velocity':   rmse_vel_t,
              'RMSE Cone Error':    rmse_cone_error,
              'Max Cone Error':     max_cone_error,
              'RMSE Radius Error':  rmse_radius_error,
              'Max Radius Error':   max_radius_error,
              'RMSE Altitude Error':rmse_altitude_error,
              'Max Altitude Error': max_altitude_error,
              'RMSE Roll Error':    rmse_roll_error,
              'Max Roll Error':     max_roll_error,
              'RMSE Pitch Error':   rmse_pitch_error,
              'Max Pitch Error':    max_pitch_error,
              'RMSE Yaw Error':     rmse_yaw_error,
              'Max Yaw Error':      max_yaw_error,
              'Wx[0]':              Wx[0,0],
              'Wx[1]':              Wx[1,1],
              'Wx[2]':              Wx[2,2],
              'Wx[3]':              Wx[3,3],
              'Wx[4]':              Wx[4,4],
              'Wx[5]':              Wx[5,5],
              'rotz':               rotz_rms * 180.0 / math.pi
    }, index=[0])

    return df

def plot_polar_error_paper(X):
    '''@brief Paper figure (LaTeX/pgf): radius, altitude, off-axis, z-rotation, velocity.'''
    matplotlib.use("pgf")
    matplotlib.rcParams.update({
        "pgf.texsystem": "pdflatex",
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })

    ori_cur = np.column_stack((X['qx'], X['qy'], X['qz'], X['qw']))
    ori_ref = np.column_stack((X['qx_ref'], X['qy_ref'], X['qz_ref'], X['qw_ref']))
    ori_curcon = quaternion_conjugate(ori_cur.T).T
    delta_quat = quaternion_product(ori_ref[:,0], ori_ref[:,1], ori_ref[:,2], ori_ref[:,3], ori_curcon[:,0], ori_curcon[:,1], ori_curcon[:,2], ori_curcon[:,3]).T
    rotz = np.arctan2(2*(delta_quat[:,3]*delta_quat[:,2] + delta_quat[:,0]*delta_quat[:,1]), 1 - 2*(delta_quat[:,1]**2 + delta_quat[:,2]**2))

    fs = 8  # font size
    plt.rc('font', size=fs)
    plt.rc('axes', titlesize=fs)
    plt.rc('axes', labelsize=fs)
    plt.rc('xtick', labelsize=fs)
    plt.rc('ytick', labelsize=fs)
    plt.rc('legend', fontsize=fs)
    plt.rc('figure', titlesize=16)

    metre_unit = r'{x:0.1f}\,m'
    ang_vel_unit = r'{x:0.1f}\,$\frac{{\mathrm{{rad}}}}{{\mathrm{{s}}}}$'
    vel_unit = r'{x:0.1f}\,$\mathrm{{m}}/\mathrm{{s}}$'
    force_unit = r'{x:0.1f}\,N'
    torque_unit = r'{x:0.1f}\,Nm'
    acc_unit = r'{x:0.1f}\,$\frac{{\mathrm{{m}}}}{{\mathrm{{s}}^2}}$'
    deg_unit = r'${x:0.1f}^{{\circ}}$'
    lw = 0.8

    fig, axs = plt.subplots(5,1, figsize=(3.5,4.5))
    axs[0].set_title(r'\textbf{{Radius}}')
    
    axs[0].plot(X.index, X['radius_ref'], label='Setpoint', color='black', linestyle='dashed', linewidth=lw)
    axs[0].plot(X.index, X['radius'], label='Actual', color='red', linewidth=lw)
    axs[0].yaxis.set_major_formatter(metre_unit)
    axs[0].set_ylim(1, 3)
    axs[0].set_yticks(np.arange(1, 3.1, 0.5))

    axs[1].set_title(r'\textbf{{Altitude}}')
    axs[1].plot(X.index, X['altitude_ref'], label='Setpoint', color='black', linestyle='dashed', linewidth=lw)
    axs[1].plot(X.index, X['altitude'], label='Actual', color='red', linewidth=lw)
    axs[1].yaxis.set_major_formatter(metre_unit)
    axs[1].set_ylim(2, 4)
    axs[1].set_yticks(np.arange(2, 4.1, 0.5))

    axs[2].set_title(r'\textbf{{Off-axis error}}')
    axs[2].plot(X.index, X['conic_error'], color='red', linewidth=lw)
    axs[2].set_ylim(0, 5)
    axs[2].set_yticks(np.arange(0, 16, 5))
    axs[2].yaxis.set_major_formatter(r'${x:g}^{{\circ}}$')
    axs[2].yaxis.set_tick_params(which='major')

    axs[3].set_title(r'\textbf{{Rotation around z-axis}}')
    axs[3].plot(X.index, rotz * 180.0 / math.pi, label='Actual', color='red', linewidth=lw)
    axs[3].set_ylim(-10, 135)
    axs[3].set_yticks(np.arange(0, 136, 45))
    axs[3].yaxis.set_major_formatter(r'${x:g}^{{\circ}}$')
    axs[3].yaxis.set_tick_params(which='major')
    
    axs[4].set_title(r'\textbf{{Tangential velocity}}')
    axs[4].plot(X.index, X['v_tangential'], label='Actual', color='red', linewidth=lw)
    axs[4].xaxis.set_major_formatter(r'{x:g}\,s')
    axs[4].xaxis.set_tick_params(which='major')
    axs[4].yaxis.set_major_formatter(vel_unit)
    axs[4].yaxis.set_tick_params(which='major')
    axs[4].axhline(0.0, linestyle='--', color='gray', linewidth=lw)
    axs[4].set_ylim(-0.2, 0.21)
    axs[4].set_yticks(np.array([-0.2, -0.1, 0.0, 0.1, 0.2]))

    for ax in axs:
        ax.xaxis.set_major_formatter(r'{x:g}\,s') # LaTex!
        ax.xaxis.set_tick_params(which='major')
        ax.yaxis.grid()
        ax.xaxis.grid()
        t0 = X.index[X['fault[0]'] == True].min()
        ax.axvline(t0, linestyle='--', color='black', linewidth=lw)

    fig.tight_layout()

    return fig, axs

def plot_thrusters_paper(propulsion_model, X, t_fault, rpm_limit):
    '''@brief Paper figure (LaTeX/pgf): per-thruster RPM with the fault window shaded.'''
    X = X.iloc[::10]
    matplotlib.use("pgf")
    matplotlib.rcParams.update({
        "pgf.texsystem": "pdflatex",
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })

    fs = 8  # font size
    plt.rc('font', size=fs)
    plt.rc('axes', titlesize=fs)
    plt.rc('axes', labelsize=fs)
    plt.rc('xtick', labelsize=fs)
    plt.rc('ytick', labelsize=fs)
    plt.rc('legend', fontsize=fs)
    plt.rc('figure', titlesize=fs)

    lw = 0.8

    thruster_names = propulsion_model.thruster_names
    display_order = THRUSTER_DISPLAY_ORDER

    fig, axs = plt.subplots(2,4, figsize=(7.2,3))

    for i_ax, ax in enumerate(axs[:,:].flat):
        i = display_order[i_ax]

        ax.set_title(thruster_names[i])
        ax.axhline(-rpm_limit, linestyle='--', color='red', linewidth=lw)
        ax.axhline(rpm_limit, linestyle='--', color='red', linewidth=lw)

        ax.fill_between(X.index, -rpm_limit, rpm_limit, where=X['fault[' + str(i) + ']'], facecolor=(1.0, 0.7, 0.7))
        ax.plot(X.index, X['rpm_sim_f[' + str(i) + ']'], label='Modeled Thruster (Filtered)', color='black', linestyle='dashed', linewidth=lw)
        ax.plot(X.index, X['n[' + str(i) + ']'], label='Actual Thruster', color='red', linewidth=lw)
        
        ax.yaxis.grid()
        ax.xaxis.grid()
        ax.xaxis.set_major_formatter(r'{x:g} s')
        ax.xaxis.set_tick_params(which='major')

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0), ncol=2)
    fig.tight_layout()
    fig.subplots_adjust(wspace=0.35, hspace=0.4)

    return fig, axs


def plot_thrusters_screen(propulsion_model, sample_times, rpm, model_rpm, fault_scheduler, plot_thrust=True):
    '''@brief On-screen per-thruster thrust/RPM plot with the fault window shaded.'''
    thruster_names = propulsion_model.thruster_names
    display_order = THRUSTER_DISPLAY_ORDER

    fig, axs = plt.subplots(2,4, figsize=(20,15))
    thrust_limit = propulsion_model.coefficient * propulsion_model.max_rpm ** 2
    model_thrust = propulsion_model.rpm_to_forces(model_rpm)
    thrust = propulsion_model.rpm_to_forces(rpm)

    if plot_thrust:
        fig.suptitle('Thrust [Newton]', fontsize=24)
    else:
        fig.suptitle('Thruster RPM', fontsize=24)

    for i_ax, ax in enumerate(axs[:,:].flat):
        i = display_order[i_ax]

        ax.set_title('Thruster ' + str(i) + ' (' + thruster_names[i] + ')')
        if plot_thrust:
            ax.plot(sample_times, model_thrust[:,i], label='Thruster Model [N]', color='black', linestyle='dashed')
            ax.plot(sample_times, thrust[:,i], label='Actual Thruster [N]', color='red')

            ax.axhline(-thrust_limit, linestyle='--', color='red', linewidth=1.5)
            ax.axhline(thrust_limit, linestyle='--', color='red', linewidth=1.5)

            ax.yaxis.set_major_formatter('{x:g} N')
            ax.yaxis.set_tick_params(which='major')
            ax.set_yticks(np.arange(-300, 301, 50))
        else:
            ax.plot(sample_times, model_rpm[:,i], label='Thruster Model', color='black', linestyle='dashed')
            ax.plot(sample_times, rpm[:,i], label='Actual Thruster', color='red')
            ax.axhline(-propulsion_model.max_rpm, linestyle='--', color='red', linewidth=1.5)
            ax.axhline(propulsion_model.max_rpm, linestyle='--', color='red', linewidth=1.5)

        if not fault_scheduler.thruster_selection[i]:
            ax.axvspan(0.0, fault_scheduler.t0, facecolor='lime', alpha=0.1)
            ax.axvspan(fault_scheduler.t0, sample_times[-1], facecolor='red', alpha=0.1)
        else:
            ax.axvspan(0.0, sample_times[-1], facecolor='lime', alpha=0.1)
        
        ax.yaxis.grid()
        ax.xaxis.grid()
        ax.xaxis.set_major_formatter(r'{x:g} s')
        ax.xaxis.set_tick_params(which='major')

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', prop={'size': 12}, ncol=2)
    fig.tight_layout()

    return fig, axs
