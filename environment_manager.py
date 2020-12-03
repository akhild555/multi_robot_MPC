import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle, Ellipse
import matplotlib.pyplot as plt

class Map(object):
  def __init__(self):
    # print("in init")
    self.red, self.blue, self.yellow, self.green = '#ff0000', '#0000ff', '#ffff00', '#00ff00'

  # Example
  def map_0(self,ax):
    # Bottom Left Corner, Width, Height
    square = Rectangle((0.7, 0.1), 0.25, 0.25, facecolor=self.red)
    # Center, Radius
    circle = Circle((0.8, 0.8), 0.15, facecolor=self.blue)
    # Vertices of Polygon
    triangle = Polygon(((0.05,0.1), (0.396,0.1), (0.223, 0.38)), fc=self.yellow)
    rhombus = Polygon(((0.5,0.2), (0.7,0.525), (0.5,0.85), (0.3,0.525)),  fc=self.green)
    # Center, Width, Height
    ellipse = Ellipse((.75, .75), 1, 0.75)
    obstacles = [square, circle, triangle, rhombus]
    for shape in obstacles:
      ax.add_patch(shape)

  # Map 1: City Blocks
  def map_1(self,ax):
    square_init_y = 0.0
    square_init_z = 0.0
    square_width = 2
    square_height = 2
    z_spacing = 3
    col_2 = 5 # Distance From 1st Col of Obs
    col_3 = 10 # Distance From 2st Col of Obs
    # Initialize Starting Obstacles
    square_obs = [Rectangle((square_init_y, square_init_z), square_width, square_height, facecolor=self.red),
                  Rectangle((square_init_y + col_2, square_init_z), square_width, square_height, facecolor=self.blue),
                  Rectangle((square_init_y + col_3, square_init_z), square_width, square_height, facecolor=self.red)]
    square_corner_y = square_init_y
    square_corner_z = square_init_z
    # Generate List of All Obstacles
    for i in range(10):
      # Create Left Block
      square_obs.append(Rectangle((square_corner_y, square_corner_z + z_spacing),
                                  square_width, square_height, facecolor=self.red))
      # Create Center Block
      square_obs.append(Rectangle((square_corner_y + col_2, square_corner_z + z_spacing),
                                  square_width, square_height, facecolor=self.blue))
      # Create Right Block
      square_obs.append(Rectangle((square_corner_y + col_3, square_corner_z + z_spacing),
                                  square_width, square_height, facecolor=self.red))
      square_corner_z = square_corner_z + z_spacing

    # Plot Obstacles
    for obs in square_obs:
      ax.add_patch(obs)

  # Map 2: Symmetric Maze
  def map_2(self,ax):
    circle_init_y = 1.5
    circle_init_z = 1.5
    z_spacing_centered = 3 # For Centered Obstacles
    z_spacing_offset = 1.5 # For Offset Obstacles
    col_2 = 2 # Distance From 1st Col of Obs
    col_3 = 4 # Distance From 1st Col of Obs
    y_spacing_l = 0.75
    y_spacing_r = 1
    circle_radius = 0.25
    # Initialize Starting Obstacles
    circle_obs = [Circle((circle_init_y, circle_init_z), circle_radius, facecolor=self.blue),
                  Circle((circle_init_y + col_2, circle_init_z), circle_radius, facecolor=self.blue),
                  Circle((circle_init_y + col_3, circle_init_z), circle_radius, facecolor=self.blue)]
    circle_center_y = circle_init_y
    circle_center_z = circle_init_z
    # Generate List of All Obstacles
    for i in range(5):
      # Create Centered Obstacles
      # 1st Col
      circle_obs.append(Circle((circle_center_y, circle_center_z + z_spacing_centered),
                               circle_radius, facecolor=self.blue))
      # 2nd Col
      circle_obs.append(Circle((circle_center_y + col_2, circle_center_z + z_spacing_centered),
                               circle_radius, facecolor=self.blue))
      # 3rd Col
      circle_obs.append(Circle((circle_center_y + col_3, circle_center_z + z_spacing_centered),
                               circle_radius, facecolor=self.blue))
      # Create Offset Obstacles
      # Left of 1st Col
      circle_obs.append(Circle((circle_center_y - y_spacing_l, circle_center_z + z_spacing_offset),
                               circle_radius, facecolor=self.red))
      # Right of 1st Col
      circle_obs.append(Circle((circle_center_y + y_spacing_r, circle_center_z + z_spacing_offset),
                               circle_radius, facecolor=self.red))
      # Right of 2nd Col
      circle_obs.append(Circle((circle_center_y + col_2 + y_spacing_r, circle_center_z + z_spacing_offset),
                               circle_radius, facecolor=self.red))
      # Right of 3rd Col
      circle_obs.append(Circle((circle_center_y + col_3 + y_spacing_r, circle_center_z + z_spacing_offset),
                               circle_radius, facecolor=self.red))
      circle_center_z = circle_center_z + z_spacing_centered

    # Plot Obstacles
    for obs in circle_obs:
      ax.add_patch(obs)

  # Map 3: Small Symmetric Maze
  def map_3(self, ax):
    circle_init_y = 1.5
    circle_init_z = 1.5
    z_spacing_centered = 3 # For Centered Obstacles
    z_spacing_offset = 1.5 # For Offset Obstacles
    col_2 = 2 # Distance From 1st Col of Obs
    col_3 = 4 # Distance From 1st Col of Obs
    y_spacing_l = 0.75
    y_spacing_r = 1
    circle_radius = 0.25
    # Initialize Starting Obstacles
    circle_obs = [Circle((circle_init_y, circle_init_z), circle_radius, facecolor=self.blue),
                  Circle((circle_init_y + col_2, circle_init_z + 1), 0.4, facecolor=self.blue)]
    circle_center_y = circle_init_y
    circle_center_z = circle_init_z
    # Generate List of All Obstacles
    for i in range(1):
      # Create Centered Obstacles
      # 1st Col
      # circle_obs.append(Circle((circle_center_y, circle_center_z + z_spacing_centered),
      #                          circle_radius, facecolor=self.blue))
      # 2nd Col
      # circle_obs.append(Circle((circle_center_y + col_2, circle_center_z + z_spacing_centered),
      #                          circle_radius, facecolor=self.blue))
      # 3rd Col
      # circle_obs.append(Circle((circle_center_y + col_3, circle_center_z + z_spacing_centered),
      #                          circle_radius, facecolor=self.blue))
      # Create Offset Obstacles
      # Left of 1st Col
      circle_obs.append(Circle((circle_center_y - y_spacing_l - 1, circle_center_z + z_spacing_offset),
                               0.5, facecolor=self.red))
      # Right of 1st Col
      circle_obs.append(Circle((circle_center_y + y_spacing_r, circle_center_z - z_spacing_offset),
                               circle_radius, facecolor=self.red))
      # Right of 2nd Col
      # circle_obs.append(Circle((circle_center_y + col_2 + y_spacing_r, circle_center_z + z_spacing_offset),
      #                          circle_radius, facecolor=self.red))
      # Right of 3rd Col
      # circle_obs.append(Circle((circle_center_y + col_3 + y_spacing_r, circle_center_z + z_spacing_offset),
      #                          circle_radius, facecolor=self.red))
      circle_center_z = circle_center_z + z_spacing_centered

    # Plot Obstacles
    for obs in circle_obs:
      ax.add_patch(obs)

    return circle_obs

  # # Map 4: Random Small Symmetric Maze
  # def map_4(self, ax, y_min, y_max, z_min, z_max):
  #   num_obs = 4
  #   # Initialize Distances
  #   d_01 = 0  # Distance Between Obstacle 0 & Obstacle 1
  #   d_02 = 0  # Distance Between Obstacle 0 & Obstacle 2
  #   d_03 = 0  # Distance Between Obstacle 0 & Obstacle 3
  #   d_12 = 0  # Distance Between Obstacle 1 & Obstacle 2
  #   d_13 = 0  # Distance Between Obstacle 1 & Obstacle 3
  #   d_23 = 0  # Distance Between Obstacle 2 & Obstacle 3
  #   while d_01 < 0.75 or d_02 < 0.75 or d_12 < 0.75 or d_03 < 0.75 or d_13 < 0.75 or d_23 < 0.75:
  #     circle_centers_y = np.random.uniform(y_min, y_max, num_obs)
  #     circle_centers_z = np.random.uniform(z_min, z_max, num_obs)
  #     circle_radii = np.random.uniform(0, 5, num_obs)
  #     circle_obs = []
  #     # Create Obstacles
  #     for i in range(num_obs):
  #       circle_obs.append(Circle((circle_centers_y[i], circle_centers_z[i]), circle_radii[i], facecolor=self.blue))
  #     # Distance to Obstacles
  #     for i in range(num_obs):
  #       obs_center = np.array(obs.center)
  #       obs_radius = obs.radius
  #       obs_safe_pos = obs_radius + 0.5
  #       dist_robber = 0
  #
  #       while dist_robber < obs_safe_pos:
  #         dist_robber = np.linalg.norm(robber_loc0 - obs_center)
  #         if dist_robber < obs_safe_pos:
  #           # Cop 0 Initial Position
  #           x0_rob_y0 = np.random.uniform(y_min, y_max)
  #           x0_rob_z0 = np.random.uniform(z_min, z_max)
  #           robber_loc0 = np.array([x0_rob_y0, x0_rob_z0])
  #
  #     x0_robber = np.array([x0_rob_y0, x0_rob_z0, 0, 0, 0, 0])
  #     # Distances
  #     d_r0 = np.linalg.norm(x0_robber - x0_0)
  #     d_r1 = np.linalg.norm(x0_robber - x1_0)
  #     d_r2 = np.linalg.norm(x0_robber - x2_0)
  #
  #   circle_centers_y = np.random.uniform(y_min, y_max, num_obs)
  #   circle_centers_z = np.random.uniform(z_min, z_max, num_obs)
  #   circle_radii = np.random.uniform(0, 5, num_obs)
  #
  #
  #
  #
  #   # # Plot Obstacles
  #   # for obs in circle_obs:
  #   #   ax.add_patch(obs)
  #
  #   return circle_obs

  def map_4(self, ax, y_min, y_max, z_min, z_max):
    num_obs = 4
    circle_centers_y = np.random.uniform(y_min, y_max, num_obs)
    circle_centers_z = np.random.uniform(z_min, z_max, num_obs)
    circle_radii = np.random.uniform(0, 2, num_obs)

    circle_obs = []
    # Create Obstacles
    for i in range(num_obs):
      circle_obs.append(Circle((circle_centers_y[i], circle_centers_z[i]), circle_radii[i], facecolor=self.blue))


    # # Plot Obstacles
    # for obs in circle_obs:
    #   ax.add_patch(obs)

    return circle_obs

  def map_plot(self, ax, obstacles):
    # Plot Obstacles
    for obs in obstacles:
      ax.add_patch(obs)
