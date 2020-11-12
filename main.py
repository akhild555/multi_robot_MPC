import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib
import matplotlib; matplotlib.use("TkAgg")

from quad_sim import simulate_quadrotor

# Need to reload the module to use the latest code
import quadrotor

importlib.reload(quadrotor)
from quadrotor import Quadrotor

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

# Construct our quadrotor controller
quadrotor = Quadrotor(Q, R, Qf);
x0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x, u, t = simulate_quadrotor(x0, tf, quadrotor)
import matplotlib.pyplot as plt

fig1, ax1 = plt.subplots(1,1)
ax1.scatter(u[:, 0], u[:, 1])
fig1.show()
print(u)

x2 = x + 0.1

x3 = x2 + 0.1

anim, fig2 = create_animation(np.stack((x, x2, x3), axis=0), tf, n_agents=3)


fig2.show()

# put breakpoint here
print()