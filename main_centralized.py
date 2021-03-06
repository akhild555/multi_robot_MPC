import numpy as np
import matplotlib.pyplot as plt
import importlib
import matplotlib; matplotlib.use("TkAgg")

# Import Simulator
from quad_sim_centralized import simulate_quadrotor_centralized
import initial_states

# Load Quadrotor Objects
import quadrotor_centralized
import quadrotor_robber

#Reload Modules to Use Latest Code
importlib.reload(quadrotor_centralized)
importlib.reload(quadrotor_robber)
from quadrotor_centralized import QuadrotorCentralized
from quadrotor_robber import QuadrotorRobber

# Load Environment Manager
import environment_manager
importlib.reload(environment_manager)
from environment_manager import Map

# Load Animation Function
import create_animation
importlib.reload(create_animation)
from create_animation import create_animation

# Bounds of Environment
y_min = 0
y_max = 7.5
z_min = 0
z_max = 7.5

# Map
ax = plt.axes()
plt.close()
environment = Map  # Initialize Map Class
# obstacles = environment().map_3(ax)
obstacles = environment().map_4(ax, y_min, y_max, z_min, z_max) # Random

# Weights of LQR cost
R = np.eye(2)
Q = np.diag([1, 1, 0, 1, 1, 1])
Qf = Q

# End time of the simulation
tf = 30

# Number of Cop Quadrotors
num_cops = 3

# Construct Robber Quadrotor Controller
quadrotor_central = QuadrotorCentralized(Q, R, Qf)

# Construct Robber Quadrotor Controller
quad_robber = QuadrotorRobber(Q, R, Qf)

# Initial States of Quadrotors
# Fixed Initializations
# x0_cops = initial_states.cops_fixed(num_cops) # Fixed Cop Initializations
# x0_robber = initial_states.robber_fixed() # Fixed Robber Initialization
# Random Initializations
x0_cops = initial_states.cops_random(obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Cop Initializations
x0_robber = initial_states.robber_random(x0_cops, obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Robber Initialization
x0_robber_des = initial_states.init_robber_des(obstacles, y_min, y_max, z_min, z_max)

# Simulate Quadrotors
x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t, t_e = simulate_quadrotor_centralized(x0_cops, x0_robber, quadrotor_central,
                                                                                              quad_robber, tf, num_cops, obstacles, x0_robber_des)

# Create Animation
x_out = np.stack(([c for c in x_cops] + [x_robber]), axis=0)
x_d_out = np.stack((x_cop_d, x_rob_d), axis=0)
anim, fig2 = create_animation(x_out, x_d_out, t, obstacles, num_cops + 1)

anim
plt.show()
