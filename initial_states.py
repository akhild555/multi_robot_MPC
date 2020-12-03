import numpy as np

def cops_fixed():

  x0_0 = np.array([3, 0.5, 0, 0, 0, 0]) # Cop 0
  x1_0 = np.array([3, 0.75, 0, 0, 0, 0]) # Cop 1
  x2_0 = np.array([0, 1, 0, 0, 0, 0]) # Cop 2
  # x3_0 = np.array([-2.5, 1, 0, 1, 1, 0]) # Cop 3
  x0_cops = [x0_0, x1_0, x2_0]

  return x0_cops

def robber_fixed():

  x0_robber = np.array([-3.5, 1, 0, 0, 0, 0])

  return x0_robber

# Random Initial States of Cops
def cops_random(obstacles, y_min, y_max, z_min, z_max):

  # Initialize Distances
  d_01 = 0  # Distance Between Cop 0 & 1
  d_02 = 0  # Distance Between Cop 0 & 2
  d_12 = 0  # Distance Between Cop 1 & 2
  while d_01 < 0.75 or d_02 < 0.75 or d_12 < 0.75:
    # Cop 0 Initial Position
    x0_y0 = np.random.uniform(y_min, y_max)
    x0_z0 = np.random.uniform(z_min, z_max)
    cop0_loc0 = np.array([x0_y0, x0_z0])
    # Cop 1 Initial Position
    x1_y0 = np.random.uniform(y_min, y_max)
    x1_z0 = np.random.uniform(z_min, z_max)
    cop1_loc0 = np.array([x1_y0, x1_z0])
    # Cop 2 Initial Position
    x2_y0 = np.random.uniform(y_min, y_max)
    x2_z0 = np.random.uniform(z_min, z_max)
    cop2_loc0 = np.array([x2_y0, x2_z0])

    # Distance to Obstacles
    for obs in obstacles:
      obs_center = np.array(obs.center)
      obs_radius = obs.radius
      obs_safe_pos = obs_radius + 0.5
      dist0 = 0
      dist1 = 0
      dist2 = 0
      while dist0 < obs_safe_pos:
        dist0 = np.linalg.norm(cop0_loc0 - obs_center)
        if dist0 < obs_safe_pos:
          # Cop 0 Initial Position
          x0_y0 = np.random.uniform(y_min, y_max)
          x0_z0 = np.random.uniform(z_min, z_max)
          cop0_loc0 = np.array([x0_y0, x0_z0])
      while dist1 < obs_safe_pos:
        dist1 = np.linalg.norm(cop1_loc0 - obs_center)
        if dist1 < obs_safe_pos:
          # Cop 1 Initial Position
          x1_y0 = np.random.uniform(y_min, y_max)
          x1_z0 = np.random.uniform(z_min, z_max)
          cop1_loc0 = np.array([x1_y0, x1_z0])
      while dist2 < obs_safe_pos:
        dist2 = np.linalg.norm(cop2_loc0 - obs_center)
        if dist2 < obs_safe_pos:
          # Cop 2 Initial Position
          x2_y0 = np.random.uniform(y_min, y_max)
          x2_z0 = np.random.uniform(z_min, z_max)
          cop2_loc0 = np.array([x2_y0, x2_z0])

    # Initial States
    x0_0 = np.array([x0_y0, x0_z0, 0, 0, 0, 0])
    x1_0 = np.array([x1_y0, x1_z0, 0, 0, 0, 0])
    x2_0 = np.array([x2_y0, x2_z0, 0, 0, 0, 0])
    # Distances
    d_01 = np.linalg.norm(x0_0 - x1_0)
    d_02 = np.linalg.norm(x0_0 - x2_0)
    d_12 = np.linalg.norm(x1_0 - x2_0)

  x0_cops = [x0_0, x1_0, x2_0]

  return x0_cops

# Random Initial State of Robber
def robber_random(x0_cops, obstacles, y_min, y_max, z_min, z_max):
  # Extract Cops
  x0_0 = x0_cops[0]
  x1_0 = x0_cops[1]
  x2_0 = x0_cops[2]
  # Initialize Distances
  d_r0 = 0  # Distance Between Robber & Cop 0
  d_r1 = 0  # Distance Between Robber & Cop 1
  d_r2 = 0  # Distance Between Robber & Cop 2
  while d_r0 < 0.75 or d_r1 < 0.75 or d_r2 < 0.75:
    x0_rob_y0 = np.random.uniform(y_min, y_max)
    x0_rob_z0 = np.random.uniform(z_min, z_max)
    robber_loc0 = np.array([x0_rob_y0, x0_rob_z0])
    # Distance to Obstacles
    for obs in obstacles:
      obs_center = np.array(obs.center)
      obs_radius = obs.radius
      obs_safe_pos = obs_radius + 0.5
      dist_robber = 0

      while dist_robber < obs_safe_pos:
        dist_robber = np.linalg.norm(robber_loc0 - obs_center)
        if dist_robber < obs_safe_pos:
          # Cop 0 Initial Position
          x0_rob_y0 = np.random.uniform(y_min, y_max)
          x0_rob_z0 = np.random.uniform(z_min, z_max)
          robber_loc0 = np.array([x0_rob_y0, x0_rob_z0])

    x0_robber = np.array([x0_rob_y0, x0_rob_z0, 0, 0, 0, 0])
    # Distances
    d_r0 = np.linalg.norm(x0_robber - x0_0)
    d_r1 = np.linalg.norm(x0_robber - x1_0)
    d_r2 = np.linalg.norm(x0_robber - x2_0)

  return x0_robber
