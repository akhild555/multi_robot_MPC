from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation

def x_d(t):
    return np.zeros(6)

def create_animation(x, tf, n_agents = 1, n_frames = 60):
    # Sample desired trajectory
    n_samples = 1000
    t_samples = np.linspace(0.0, tf, n_samples)
    x_des = np.zeros((n_agents, n_samples, 6))

    for i in range(t_samples.shape[0]):
        x_des[:, i] = x_d(t_samples[i])

    from matplotlib import rc
    rc('animation', html='jshtml')

    fig = plt.figure(figsize=(8,6))
    ax = plt.axes()

    if n_agents == 1:
        x = x.reshape((1, x.shape[0], x.shape[1]))

    x_max = max(np.max(x_des[:, :, 0]), np.max(x[:, :, 0]))
    x_min = min(np.min(x_des[:, :, 0]), np.min(x[:, :, 0]))
    y_max = max(np.max(x_des[:, :, 1]), np.max(x[:, :, 1]))
    y_min = min(np.min(x_des[:, :, 1]), np.min(x[:, :, 1]))

    # noinspection PyTypeChecker
    frame_idx = [round(x) for x in np.linspace(0, x.shape[1]-1, n_frames).tolist()]
    x_anim = np.zeros((n_agents, n_frames, 6))
    for i in range(n_frames):
        x_anim[:, i, :] = x[:, frame_idx[i], :]

    # a = length of quadrotor
    a = 0.25
    y = x_anim[:, :, 0]
    z = x_anim[:, :, 1]
    theta = x_anim[:, :, 2]

    x_padding = 0.25 * (x_max - x_min)
    y_padding = 0.25 * (y_max - y_min)

    def frame(i):
        ax.clear()
        for n in range(n_agents):
            if n == n_agents-1:
                color = 'r'
            else:
                color = 'g'
            ax.plot(x_des[n, 0, 0], x_des[n, 0, 1], 'b*', label='desired position')
            ax.plot(x_anim[n, 0:i+1, 0], x_anim[n, 0:i+1, 1], '--', label='actual trajectory')
            if n == n_agents - 1:
                ax.plot(x_anim[n, 0:i + 1, 0], x_anim[n, 0:i + 1, 1], color + '--', label='actual trajectory')
            # plot=ax.scatter(x_anim[i, 0], x_anim[i, 1], c='r', label='quadrotor position')
            ax.plot([y[n, i] + a*cos(theta[n, i]), y[n, i] - a*cos(theta[n, i])],
                           [z[n, i] + a*sin(theta[n, i]), z[n, i] - a*sin(theta[n, i])] , color, 'LineWidtheeta',3)

            if(np.abs((x_max - x_min) - (y_max - y_min)) < 5):
                ax.set_xlim(x_min - x_padding, x_max + x_padding)
                ax.set_ylim(y_min - y_padding, y_max + y_padding)

        ax.set_xlabel('y (m)')
        ax.set_ylabel('z (m)')
        ax.set_aspect('equal')
        ax.legend(loc='upper left')

        return ax

    return animation.FuncAnimation(fig, frame, interval=100, frames=n_frames, blit=False, repeat=True), fig