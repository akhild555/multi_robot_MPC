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

# Reload Modules to Use Latest Code
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

# Bounds of Environment
y_min = 0
y_max = 40
z_min = 0
z_max = 20

# Map
ax = plt.axes()
plt.close()
environment = Map()  # Initialize Map Class
# obstacles = environment.map_3() # Fixed
# obstacles = environment.map_4(ax, y_min, y_max, z_min, z_max) # Random
# obstacles = environment.map_5() # Fixed Dense Maze
obstacles = environment.map_6() # Fixed Sparse, Small Obstacle Maze
# obstacles = environment.map_7() # Fixed Sparse, Large Obstacle Maze
# obstacles = environment.map_8() # City Blocks, Gaps
# obstacles = environment.map_9() # City Blocks, Solid Walls

# Weights of LQR cost
R = np.eye(2)
Q = np.diag([1, 1, 0, 1, 1, 1])
Qf = Q

# End time of the simulation
tf = 300

# Number of Cop Quadrotors
num_cops = 5

# Construct Cop Quadrotor Controllers
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
x0_cops = initial_states.cops_fixed(num_cops) # Fixed Cop Initializations
x0_robber = initial_states.robber_fixed() # Fixed Robber Initialization
x0_robber_des = initial_states.init_fixed_robber_des() # Fixed Robber Desired Initialization
# Random Initializations
# x0_cops = initial_states.cops_random(obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Cop Initializations
# x0_robber = initial_states.robber_random(x0_cops, obstacles, num_cops, y_min, y_max, z_min, z_max) # Random Robber Initialization
# x0_robber_des = initial_states.init_robber_des(obstacles, y_min, y_max, z_min, z_max)

# Simulate Quadrotors
# Distributed MPC
x_cops, x_cop_d, u_cops, x_robber, x_rob_d, u_robber, t, t_e = simulate_quadrotor(x0_cops, x0_robber, quad_cops, quad_robber,
                                                                             tf, num_cops, obstacles, x0_robber_des)
# Create Animation
x_out = np.stack(([c for c in x_cops] + [x_robber]), axis=0)
x_d_out = np.stack((x_cop_d, x_rob_d), axis=0)
anim, fig2 = create_animation(x_out, x_d_out, t, obstacles, num_cops + 1, title="Distributed")
# writervideo = animation.FFMpegWriter(fps=5)
# anim.save('distributed.mp4',writer=writervideo)
plt.show()

# Simulate Quadrotors
# Centralized MPC
x_cops_c, x_cop_d_c, u_cops_c, x_robber_c, x_rob_d_c, u_robber_c, t_c, t_e_c = simulate_quadrotor_centralized(x0_cops, x0_robber, quadrotor_central,
                                                                                         quad_robber, tf, num_cops, obstacles, x0_robber_des)

# Create Animation
x_out_c = np.stack(([c for c in x_cops_c] + [x_robber_c]), axis=0)
x_d_out_c = np.stack((x_cop_d_c, x_rob_d_c), axis=0)

print("\nDistributed: t_f = {:.2f}, t_elapsed = {:.2f}".format(t[-1], t_e))
print("Centralized: t_f = {:.2f}, t_elapsed = {:.2f}".format(t_c[-1], t_e_c))


anim_c, fig3 = create_animation(x_out_c, x_d_out_c, t_c, obstacles, num_cops + 1, title = "Centralized")
# anim_c.save('central.mp4',writer=writervideo)
plt.show()

print()