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

"""
Load in the animation function
"""
import create_animation

importlib.reload(create_animation)
from create_animation import create_animation

# Weights of LQR cost
R = np.eye(2);
Q = np.diag([10, 10, 1, 1, 1, 1]);
Qf = Q;

# End time of the simulation
tf = 10;


# number of quadrotors
num_quad = 4

# Construct our quadrotor controllers
quadrotor0 = Quadrotor(Q, R, Qf);
quadrotor1 = Quadrotor(Q, R, Qf);
quadrotor_robber = QuadrotorRobber(Q, R, Qf);
# quadrotor2 = Quadrotor(Q, R, Qf);
# quadrotor3 = Quadrotor(Q, R, Qf);
# quad_ctrls = [quadrotor0, quadrotor1, quadrotor2, quadrotor3]
quad_ctrls = [quadrotor0, quadrotor1]
# quad_ctrls = [quadrotor0, quadrotor_robber]

# initial states of quadrotors
# x0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x0_0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x1_0 = np.array([0.75, 0.75, 0, 1, 1, 0])
x2_0 = np.array([-1, 1, 0, 1, 1, 0])
x3_0 = np.array([-2.5, 1, 0, 1, 1, 0])

# x0 = [x0_0, x1_0, x2_0, x3_0]
x0 = [x0_0, x1_0]

# simulate quadrotors
# x, u, t = simulate_quadrotor(x0, tf, quad_ctrls, num_quad)
x, u, t = simulate_quadrotor(x0, tf, quad_ctrls, 2)
import matplotlib.pyplot as plt

# fig1, ax1 = plt.subplots(1,1)
# ax1.scatter(u[:, 0], u[:, 1])
# fig1.show()
# print(u)
#
# x2 = x + 0.1
#
# x3 = x2 + 0.1

# anim, fig2 = create_animation(np.stack((x[0], x[1], x[2], x[3]), axis=0), tf, n_agents=4)
anim, fig2 = create_animation(np.stack((x[0], x[1]), axis=0), tf, n_agents=2)



anim
plt.show()
# put breakpoint here
print()
