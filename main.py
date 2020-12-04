import numpy as np
import matplotlib.pyplot as plt
import importlib
import matplotlib; matplotlib.use("TkAgg")
import matplotlib.animation as animation

# Import Simulator
from quad_sim import simulate_quadrotor
from quad_sim_centralized import simulate_quadrotor_centralized
import initial_states

# Load Quadrotor Objects
import quadrotor
import quadrotor_centralized
import quadrotor_robber

#Reload Modules to Use Latest Code
importlib.reload(quadrotor)
importlib.reload(quadrotor_centralized)
importlib.reload(quadrotor_robber)
from quadrotor import Quadrotor
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

from copy import copy

# Bounds of Environment
y_min = 0
y_max = 7.5
z_min = 0
z_max = 7.5

# Map
ax = plt.axes()
plt.close()
environment = Map()  # Initialize Map Class
obstacles = environment.map_3(ax) # Fixed
# obstacles = environment().map_4(ax, y_min, y_max, z_min, z_max) # Random

# Weights of LQR cost
R = np.eye(2);
Q = np.diag([1, 1, 0, 1, 1, 1])
Qf = Q

# End time of the simulation
tf = 1

# Number of Cop Quadrotors
num_cops = 3

# Construct Cop Quadrotor Controllers
# quadrotor0 = Quadrotor(Q, R, Qf)
# quadrotor1 = Quadrotor(Q, R, Qf)
# quadrotor2 = Quadrotor(Q, R, Qf)
# quad_cops = [quadrotor0, quadrotor1, quadrotor2]

# Distributed MPC Quadrotor Objects
quad_cops = []
for i in range(num_cops):
  quad_cops.append(Quadrotor(Q, R, Qf))

# Centralized Quadrotor Objects
quadrotor_central = QuadrotorCentralized(Q, R, Qf)

# Construct Robber Quadrotor Controller
quad_robber = QuadrotorRobber(Q, R, Qf)

# Initial States of Quadrotors
# Fixed Initializations
# x0_cops = initial_states.cops_fixed() # Fixed Cop Initializations
# x0_robber = initial_states.robber_fixed() # Fixed Robber Initialization
# Random Initializations
x0_cops = initial_states.cops_random(obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Cop Initializations
x0_robber = initial_states.robber_random(x0_cops, obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Robber Initialization
x0_robber_des = initial_states.init_robber_des(obstacles, y_min, y_max, z_min, z_max)

# Simulate Quadrotors
# Distributed MPC
x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t = simulate_quadrotor(x0_cops, x0_robber, quad_cops, quad_robber,
                                                                             tf, num_cops, obstacles, x0_robber_des)
# Create Animation
x_out = np.stack(([c for c in x_cops] + [x_robber]), axis=0)
# x_out = np.stack((x_cops[0], x_cops[1], x_cops[2], x_robber), axis=0)
x_d_out = np.stack((x_cop_d, x_rob_d), axis=0)
anim, fig2 = create_animation(x_out, x_d_out, t, obstacles, num_cops + 1, title="Distributed")
#anim.save('distributed.mp4')
anim
plt.show()
#del anim
#del fig2
#del ax2

# plt.show()
# plt.close()

# Simulate Quadrotors
# Centralized MPC
x_cops_c, x_cop_d_c, u_cops_c, x_robber_c, x_rob_d_c, u_robber_c, t_c = simulate_quadrotor_centralized(x0_cops, x0_robber, quadrotor_central,
                                                                                         quad_robber, tf, num_cops, obstacles, x0_robber_des)

# Create Animation
x_out_c = np.stack(([c for c in x_cops] + [x_robber]), axis=0)
# x_out_c = np.stack((x_cops_c[0], x_cops_c[1], x_cops_c[2], x_robber_c), axis=0)
x_d_out_c = np.stack((x_cop_d_c, x_rob_d_c), axis=0)
# plt.close()
anim_c, fig3 = create_animation(x_out_c, x_d_out_c, t_c, obstacles, num_cops + 1, title = "Centralized")
#anim_c.save('central.mp4')
anim_c
plt.show()
# anim_c
# plt.show()



# anim
# plt.show()
print()