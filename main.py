import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib
import matplotlib; matplotlib.use("TkAgg")
import time

# Import Simulator
from quad_sim import simulate_quadrotor
from quad_sim_centralized import simulate_quadrotor_centralized

# Need to reload the module to use the latest code
import quadrotor
import quadrotor_robber
import quadrotor_centralized

importlib.reload(quadrotor)
importlib.reload(quadrotor_robber)
importlib.reload(quadrotor_centralized)
from quadrotor import Quadrotor
from quadrotor_robber import QuadrotorRobber
from quadrotor_centralized import QuadrotorCentralized

"""
Load in the animation function
"""
import create_animation

importlib.reload(create_animation)
from create_animation import create_animation

# Weights of LQR cost
R = np.eye(2);
Q = np.diag([1, 1, 0, 1, 1, 1])
Qf = Q

# End time of the simulation
tf = 25


# number of cop quadrotors
num_cops = 3

# Distributed MPC
# Construct cop quadrotor controllers
quadrotor0 = Quadrotor(Q, R, Qf)
quadrotor1 = Quadrotor(Q, R, Qf)
quadrotor2 = Quadrotor(Q, R, Qf)

# Construct robber quadrotor controller
quad_robber = QuadrotorRobber(Q, R, Qf)
# quadrotor3 = Quadrotor(Q, R, Qf);
# quad_ctrls = [quadrotor0, quadrotor1, quadrotor2, quadrotor3]
quad_cops = [quadrotor0, quadrotor1, quadrotor2]
# quad_ctrls = [quadrotor0, quadrotor_robber]

# initial states of quadrotors
# x0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x0_0 = np.array([0.5, 0.5, 0, 0, 0, 0])
x1_0 = np.array([0.75, 0.75, 0, 0, 0, 0])
x2_0 = np.array([-1, 1, 0, 0, 0, 0])
# x3_0 = np.array([-2.5, 1, 0, 1, 1, 0])

x0_cops = [x0_0, x1_0, x2_0]
x0_robber = np.array([-3.5, 1, 0, 0, 0, 0])

# simulate quadrotors
# x, u, t = simulate_quadrotor(x0, tf, quad_ctrls, num_quad)
# x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t = simulate_quadrotor(x0_cops, x0_robber, quad_cops, quad_robber, tf, num_cops)

# Centralized MPC
quadrotor_central = QuadrotorCentralized(Q, R, Qf)
x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t = simulate_quadrotor_centralized(x0_cops, x0_robber, quadrotor_central, quad_robber, tf, num_cops)

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
