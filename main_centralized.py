import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib
import matplotlib; matplotlib.use("TkAgg")
from quad_sim import simulate_quadrotor
# Need to reload the module to use the latest code
import quadrotor
import quadrotor_robber
importlib.reload(quadrotor)
importlib.reload(quadrotor_robber)
from quadrotor import Quadrotor
from quadrotor_robber import QuadrotorRobber

import environment_manager
importlib.reload(environment_manager)
from environment_manager import Map

"""
Load in the animation function
"""
import create_animation
importlib.reload(create_animation)
from create_animation import create_animation
from quad_sim_centralized import *
from quadrotor_centralized import *
# Weights of LQR cost
R = np.eye(2);
Q = np.diag([1, 1, 0, 1, 1, 1])
Qf = Q

# Map
ax = plt.axes()
plt.close()
environment = Map  # Initialize Map Class
obstacles = environment().map_3(ax)

# End time of the simulation
tf = 25


# number of cop quadrotors
num_cops = 3

# Construct robber quadrotor controller
quad_robber = QuadrotorRobber(Q, R, Qf)


# initial states of quadrotors
# x0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x0_0 = np.array([0.5, 0.5, 0, 0, 0, 0])
x1_0 = np.array([0.75, -0.75, 0, 0, 0, 0])
x2_0 = np.array([-1, 1, 0, 0, 0, 0])
# x3_0 = np.array([-2.5, 1, 0, 1, 1, 0])
x0_cops = [x0_0, x1_0, x2_0]
x0_robber = np.array([-3.5, 1, 0, 0, 0, 0])

# Centralized MPC
quadrotor_central = QuadrotorCentralized(Q, R, Qf)
x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t = simulate_quadrotor_centralized(x0_cops, x0_robber, quadrotor_central, quad_robber, tf, num_cops, obstacles)

# fig1, ax1 = plt.subplots(1,1)
# ax1.scatter(u[:, 0], u[:, 1])
# fig1.show()
# print(u)


# anim, fig2 = create_animation(np.stack((x[0], x[1], x[2], x[3]), axis=0), tf, n_agents=4)


x_out = np.stack((x_cops[0], x_cops[1], x_cops[2], x_robber), axis=0)
x_d_out = np.stack((x_cop_d, x_rob_d), axis=0)
anim, fig2 = create_animation(x_out, x_d_out, tf, num_cops + 1)



anim
plt.show()
# put breakpoint here
print()
