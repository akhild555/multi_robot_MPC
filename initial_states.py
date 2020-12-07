import numpy as np
from numpy.linalg import norm

def cops_fixed(num_cops):

  # # Map 5
  # x0_0 = np.array([0, 3.5, 0, 0, 0, 0]) # Cop 0
  # x1_0 = np.array([0, 7.5, 0, 0, 0, 0]) # Cop 1
  # x2_0 = np.array([3.5, 1.5, 0, 0, 0, 0]) # Cop 2
  # x3_0 = np.array([3.5, 5.5, 0, 0, 0, 0]) # Cop 3
  # x4_0 = np.array([3.5, 9.5, 0, 0, 0, 0]) # Cop 4
  # Map 6, 7
  x0_0 = np.array([0, 5.5, 0, 0, 0, 0]) # Cop 0
  x1_0 = np.array([0, 13.5, 0, 0, 0, 0]) # Cop 1
  x2_0 = np.array([9.5, 1.5, 0, 0, 0, 0]) # Cop 2
  x3_0 = np.array([9.5, 9.5, 0, 0, 0, 0]) # Cop 3
  x4_0 = np.array([9.5, 17.5, 0, 0, 0, 0]) # Cop 4
  # Map 8, 9
  # x0_0 = np.array([0, 3.5, 0, 0, 0, 0]) # Cop 0
  # x1_0 = np.array([0, 7.5, 0, 0, 0, 0]) # Cop 1
  # x2_0 = np.array([0, 11.5, 0, 0, 0, 0]) # Cop 2
  # x3_0 = np.array([6.5, 3.5, 0, 0, 0, 0]) # Cop 3
  # x4_0 = np.array([6.5, 11.5, 0, 0, 0, 0]) # Cop 4
  x_all_cops = [x0_0, x1_0, x2_0, x3_0, x4_0] # Combine Cops

  x0_cops =[] # Initialize Output
  # Gather Initial States
  for i in range(num_cops):
    x0_cops.append(x_all_cops[i])

  return x0_cops

def robber_fixed():

  # Map 5
  # x0_robber = np.array([7.5, 5.5, 0.0, 0.0, 0.0, 0.0])
  # Map 6, 7
  x0_robber = np.array([17.5, 13.5, 0.0, 0.0, 0.0, 0.0])
  # Map 8, 9
  # x0_robber = np.array([10.5, 7.5, 0.0, 0.0, 0.0, 0.0])

  return x0_robber

# Random Initial States of Cops
def cops_random(obstacles, num_cops, y_min, y_max, z_min, z_max):

  # Initialize Distances
  safe_dist_cop = 0.75 # Safe Distance Between Cops
  dist_bw_cops = [] # Store Distance Between Cops
  cop_locs = [] # Stoce Cop Locations
  # Initialize Cop Locations
  for i in range(num_cops):
    dist_bw_cops.append(0)
  # While Loop Counters
  outside_while = 0
  inner_while = 0
  # Check Cops are Not Colliding With Each Other
  while any(x < safe_dist_cop for x in dist_bw_cops):
    # Generate Random Initial Cop Positions
    for i in range(num_cops):
      y0 = np.random.uniform(y_min, y_max)
      z0 = np.random.uniform(z_min, z_max)
      cop_loc = np.array([y0, z0])
      cop_locs.append(cop_loc)
    # Check Cops are Not Colliding with Obstacles
    for i in range(num_cops):
      dist_bw_obs_cop = [] # Store Distance Between Obstacles and Cop
      obs_safe_dist = [] # Store Safe Distance Between Obstacles
      for j in range(len(obstacles)):
        # Calculate Safe Distance From Obstacle
        obs_safe_dist.append(obstacles[j].radius + 0.25)
        # Distance Between Cop and Obstacles
        dist_bw_obs_cop.append(norm(cop_locs[i] - np.array(obstacles[j].center)))
      # Check Cop is Not Colliding with Obstacles
      while any(x < y for x, y in zip(dist_bw_obs_cop, obs_safe_dist)):
        # Generate Random Initial Cop Positions
        y0 = np.random.uniform(y_min, y_max)
        z0 = np.random.uniform(z_min, z_max)
        cop_loc = np.array([y0, z0])
        cop_locs[i] = cop_loc
        inner_while += 1
        print("inner_while",inner_while)
    # Initialize Distance Between Cops
    dist_bw_cops = []
    # Calculate Distance Between Cops
    for i in range(num_cops):
      for j in range(i + 1, num_cops):
        if j < num_cops:
          dist_bw_cops.append(norm(cop_locs[i] - cop_locs[j]))

    outside_while += 1
    print("outside_while",outside_while)
  # Initialize Cop Initial States
  x0_cops = []
  # Store Cop Initial States
  for i in range(num_cops):
    x0_cop = np.array([cop_locs[i][0], cop_locs[i][1], 0, 0, 0, 0])
    x0_cops.append(x0_cop)

  print("Initialized Cops")

  return x0_cops

# Random Initial State of Robber
def robber_random(x0_cops, obstacles, num_cops, y_min, y_max, z_min, z_max):
  # While Loop Counters
  outside_while = 0
  inner_while = 0
  # Initialize Distances
  safe_dist_cop = 0.75  # Safe Distance Between Cops
  dist_bw_cops_rob = [] # Store Distance Between Cops and Robber
  for i in range(num_cops):
    dist_bw_cops_rob.append(0)
  # Check Obstacle & Cop Collisions
  while any(x < safe_dist_cop for x in dist_bw_cops_rob):
    # Generate Random Robber Position
    x0_rob_y0 = np.random.uniform(y_min, y_max)
    x0_rob_z0 = np.random.uniform(z_min, z_max)
    robber_loc0 = np.array([x0_rob_y0, x0_rob_z0])
    # Check Not Colliding With Obstacles
    dist_bw_obs_rob = [] # Store Distance Between Obstacles and Cop
    obs_safe_dist = [] # Store Safe Distance Between Obstacles
    for i in range(len(obstacles)):
      # Calculate Safe Distance From Obstacle
      obs_safe_dist.append(obstacles[i].radius + 0.25)
      # Distance Between Cop and Obstacles
      dist_bw_obs_rob.append(norm(robber_loc0 - np.array(obstacles[i].center)))
    # Check Robber is Not Colliding with Obstacles
    while any(x < y for x, y in zip(dist_bw_obs_rob, obs_safe_dist)):
      # Generate Random Initial Cop Positions
      x0_rob_y0 = np.random.uniform(y_min, y_max)
      x0_rob_z0 = np.random.uniform(z_min, z_max)
      robber_loc0 = np.array([x0_rob_y0, x0_rob_z0])
      inner_while += 1
      print("inner_while",inner_while)

    dist_bw_cops_rob = []
    for i in range(num_cops):
      dist_bw_cops_rob.append(norm(robber_loc0 - x0_cops[i][0:2]))
    outside_while += 1
    print("outside_while",outside_while)

  # Store Initial Robber State
  x0_robber = np.array([x0_rob_y0, x0_rob_z0, 0, 0, 0, 0])

  print("Initialized Robber")

  return x0_robber

def init_robber_des(obstacles, y_min, y_max, z_min, z_max):

  margin = 0.5
  x0_robber_des = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
  rob_des_found = False

  while not rob_des_found:
    x0_robber_des[0] = np.random.uniform(y_min, y_max)
    x0_robber_des[1] = np.random.uniform(z_min, z_max)

    for obs in obstacles:
      obs_center = np.array(obs.center)
      obs_radius = obs.radius
      dist = np.linalg.norm(x0_robber_des[0:2] - obs_center)
      if dist < obs_radius + margin:
        rob_des_found = False
        break
      else:
        rob_des_found = True

  return x0_robber_des

def init_fixed_robber_des():

  # Map 5
  # x0_robber_des = np.array([11.5, 5.5, 0.0, 0.0, 0.0, 0.0])
  # Map 6, 7
  x0_robber_des = np.array([21.5, 13.5, 0.0, 0.0, 0.0, 0.0])
  # Map 8, 9
  # x0_robber_des = np.array([15.0, 7.5, 0.0, 0.0, 0.0, 0.0])

  return x0_robber_des