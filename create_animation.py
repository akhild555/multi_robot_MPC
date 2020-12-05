from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation
import importlib
import environment_manager
importlib.reload(environment_manager)
from environment_manager import Map
from copy import copy

def create_animation(x, x_des, t, obstacles, n_agents = 1, dt = 0.01, n_frames = 60, title=""):
    # Sample desired trajectory
    #n_samples = 1000
    #t_samples = np.linspace(0.0, tf, n_samples)
    #x_des = np.zeros((n_agents, n_samples, 6))

    #for i in range(t_samples.shape[0]):
        #x_des[:, i] = x_d(t_samples[i])

    from matplotlib import rc
    rc('animation', html='jshtml')

    tf = t[-1]
    n_frames = max(2, int(tf / (30 * dt)))

    # fig = plt.figure(figsize=(8,6))
    fig, ax = plt.subplots()
    # ax = plt.axes()
    # ax = fig.axes()
    # print(x_des) this
    # print(x_des) this
    if n_agents == 1:
        x = x.reshape((1, x.shape[0], x.shape[1]))

    # x_max = max(np.max(x_des[:, :, 0]), np.max(x[:, :, 0]))
    # x_min = min(np.min(x_des[:, :, 0]), np.min(x[:, :, 0]))
    # y_max = max(np.max(x_des[:, :, 1]), np.max(x[:, :, 1]))
    # y_min = min(np.min(x_des[:, :, 1]), np.min(x[:, :, 1]))

    # # Map 0: Straightway
    # y_max = 12
    # y_min = 0
    # z_max = 35
    # z_min = 0
    # ax.set_xlim(y_min, y_max)
    # ax.set_ylim(z_min, z_max)
    # ax.set_aspect('equal')

    # # Map 1: City Blocks
    # y_max = 12
    # y_min = 0
    # z_max = 35
    # z_min = 0
    # ax.set_xlim(y_min, y_max)
    # ax.set_ylim(z_min, z_max)
    # ax.set_aspect('equal')

    # # Map 2: Symmetric Maze
    # y_max = 7.5
    # y_min = 0
    # z_max = 35
    # z_min = 0
    # ax.set_xlim(y_min, y_max)
    # ax.set_ylim(z_min, z_max)
    # ax.set_aspect('equal')

    # # Map 3: Symmetric Maze
    y_min = 0
    y_max = 10
    z_min = 0
    z_max = 10  # 35
    ax.set_xlim(y_min, y_max)
    ax.set_ylim(z_min, z_max)
    ax.set_aspect('equal')

    #n_frames = round(x.shape[1]/10)
    # noinspection PyTypeChecker
    frame_idx = [round(x) for x in np.linspace(0, x_des.shape[1]-1, n_frames).tolist()]
    x_anim = np.zeros((n_agents, n_frames, 6))
    x_d_anim = np.zeros((2, n_frames, 6))
    t_anim = np.zeros((n_frames))
    for i in range(n_frames):
        x_anim[:, i, :] = x[:, frame_idx[i], :]
        x_d_anim[:, i, :] = x_des[:, frame_idx[i], :]
        t_anim[i] = t[frame_idx[i]]

    # a = length of quadrotor
    a = 0.25
    y = x_anim[:, :, 0]
    z = x_anim[:, :, 1]
    theta = x_anim[:, :, 2]

    # x_padding = 0.25 * (x_max - x_min)
    # y_padding = 0.25 * (y_max - y_min)
    y_padding = 0.25 * (y_max - y_min)
    z_padding = 0.25 * (z_max - z_min)

    def frame(i):
        ax.clear()
        for n in range(n_agents):
            if n == n_agents-1:
                key = 1
                color = 'r'
            else:
                key = 0
                color = 'g'

            # environment().map_0(ax) # Map 0
            # environment().map_1(ax) # Map 1
            # environment().map_2(ax) # Map 2
            # environment().map_3(ax) # Map 3
            map_plot(ax, obstacles) # Map Plot

            if not key and n>0:
                ax.plot(x_d_anim[key, i, 0], x_d_anim[key, i, 1], color + '*', label='_nolegend_')
                ax.plot(x_anim[n, 0:i + 1, 0], x_anim[n, 0:i + 1, 1], color + '--', label='_nolegend_')
            elif not key:
                ax.plot(x_d_anim[key, i, 0], x_d_anim[key, i, 1], color+'*', label='cop desired position')
                ax.plot(x_anim[n, 0:i + 1, 0], x_anim[n, 0:i + 1, 1], color+'--', label='cop actual trajectory')
            else:
                ax.plot(x_d_anim[key, i, 0], x_d_anim[key, i, 1], color+'*', label='robber desired position')
                ax.plot(x_anim[n, 0:i + 1, 0], x_anim[n, 0:i + 1, 1], color+'--', label='robber actual trajectory')

            # plot=ax.scatter(x_anim[i, 0], x_anim[i, 1], c='r', label='quadrotor position')
            ax.plot([y[n, i] + a*cos(theta[n, i]), y[n, i] - a*cos(theta[n, i])],
                           [z[n, i] + a*sin(theta[n, i]), z[n, i] - a*sin(theta[n, i])] , color, 'LineWidtheeta',3)

            # if(np.abs((x_max - x_min) - (y_max - y_min)) < 5):
            #     ax.set_xlim(x_min - x_padding, x_max + x_padding)
            #     ax.set_ylim(y_min - y_padding, y_max + y_padding)
            if(np.abs((y_max - y_min) - (z_max - z_min)) < 5):
                ax.set_xlim(y_min - y_padding, y_max + y_padding)
                ax.set_ylim(z_min - z_padding, z_max + z_padding)

        ax.set_xlabel('y (m)')
        ax.set_ylabel('z (m)')
        ax.set_aspect('equal')
        ax.legend(loc='upper left')
        ax.text(0.9, 0.02, "time: {:.2f}s".format(float(t_anim[i])), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes)
        ax.set_title(title)

        return ax

    return animation.FuncAnimation(fig, frame, interval=1000/30, frames=n_frames, blit=False, repeat=True), fig

# Plot Random Obstacles
def map_plot(ax, obstacles):
    # Plot Obstacles
    for obs in obstacles:
      ax.add_patch(copy(obs))