import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables

class QuadrotorCentralized(object):
  def __init__(self, Q, R, Qf):
    self.g = 9.81
    self.m = 1
    self.a = 0.25
    self.I = 0.0625
    self.Q = Q
    self.R = R
    self.Qf = Qf

    # Input limits
    self.umin = 0
    self.umax = 5.5

    self.n_x = 6
    self.n_u = 2

    # MPC cost weights
    self.w_x = 1
    self.w_u = 1

    # Collision cost weights (max cost 1)
    self.w_y = 1.33
    self.w_z = 5

    self.D_y = 0.75
    self.D_z = 0.2

    self.obstacle_margin = 0.5

    self.obstacle_margin = 0.5

  def x_d(self):
    # Nomial state
    return np.array([0, 0, 0, 0, 0, 0])

  def u_d(self, x_d):
    # Nomial input
    return np.array([self.m*self.g/(2*cos(x_d[2])), self.m*self.g/(2*cos(x_d[2]))])

  def continuous_time_full_dynamics(self, x, u):
    # Dynamics for the quadrotor
    g = self.g
    m = self.m
    a = self.a
    I = self.I

    theta = x[2]
    ydot = x[3]
    zdot = x[4]
    thetadot = x[5]
    u0 = u[0]
    u1 = u[1]

    xdot = np.array([ydot,
                     zdot,
                     thetadot,
                     -sin(theta) * (u0 + u1) / m,
                     -g + cos(theta) * (u0 + u1) / m,
                     a * (u0 - u1) / I])
    return xdot

  def continuous_time_linearized_dynamics(self, x_des):
    # Dynamics linearized at the fixed point
    # This function returns A and B matrix
    u_des = self.u_d(x_des)
    A = np.zeros((6,6))
    A[:3, -3:] = np.identity(3)
    A[3, 2] = (-cos(x_des[2])/self.m) * (u_des[0] + u_des[1])
    A[4, 2] = (-sin(x_des[2])/self.m) * (u_des[0] + u_des[1])

    B = np.zeros((6,2))
    B[3,0] = -sin(x_des[2])/self.m
    B[3,1] = -sin(x_des[2])/self.m
    B[4,0] = cos(x_des[2])/self.m
    B[4,1] = cos(x_des[2])/self.m
    B[5,0] = self.a/self.I
    B[5,1] = -self.a/self.I

    return A, B

  def discrete_time_linearized_dynamics(self, x_des, T):
    # Discrete time version of the linearized dynamics at the fixed point
    # This function returns A and B matrix of the discrete time dynamics
    A_c, B_c = self.continuous_time_linearized_dynamics(x_des)
    A_d = np.identity(6) + A_c * T
    B_d = B_c * T

    return A_d, B_d

  def add_initial_state_constraint(self, prog, x, x_des, x_current):
    # TODO: impose initial state constraint.
    # Use AddBoundingBoxConstraint
    prog.AddBoundingBoxConstraint(x_current - x_des, x_current - x_des, x[0])

    # pass

  def add_input_saturation_constraint(self, prog, x, x_des, u, N):
    # TODO: impose input limit constraint.
    # Use AddBoundingBoxConstraint
    # The limits are available through self.umin and self.umax

    for i in range(N-1):
      prog.AddBoundingBoxConstraint(self.umin-self.u_d(x_des), self.umax-self.u_d(x_des), u[i])
    # pass

  def add_linear_velocity_constraint(self, prog, x, x_des, N):
    # -2.5 m/s <= x_dot,y_dot <= 2.5 m/s
    # N or N-1?
    v_min = -2.5
    v_max = 2.5
    for i in range(N):
      prog.AddBoundingBoxConstraint(v_min - x_des[3:5], v_max - x_des[3:5], x[i][3:5])
    # pass

  def add_angular_velocity_constraint(self, prog, x, x_des, N):
    # -0.075 rad/s <= w <= 0.075 rad/s
    # N or N-1?
    w_min = -0.7
    w_max = 0.7
    for i in range(N):
      prog.AddBoundingBoxConstraint(w_min - x_des[5], w_max - x_des[5], x[i][5])
    # pass

  def add_acceleration_constraint(self, prog, x, x_des, N):
    #   # -0.25 m/s^2 <= a <= 0.25 m/s^2
    a_min = -0.25
    a_max = 0.25
    dt = 1e-2
    for i in range(1, N):
      accel = (x[i][3:5] - x[i - 1][3:5])# 1 sec Interval
      prog.AddLinearConstraint(accel[0] <= a_max) # y_ddot UB
      prog.AddLinearConstraint(accel[0] >= a_min) # y_ddot LB
      prog.AddLinearConstraint(accel[1] <= a_max) # z_ddot UB
      prog.AddLinearConstraint(accel[1] >= a_min) # z_ddot LB
    # pass

  def add_angular_acceleration_constraint(self, prog, x, x_des, N):
      #   # -0.005 rad/s^2 <= alpha <= 0.005 rad/s^2
    alpha_min = -0.4
    alpha_max = 0.4
    dt = 1e-2
    for i in range(1, N):
      accel = (x[i][5] - x[i - 1][5]) #1 sec Interval
      prog.AddLinearConstraint(accel <= alpha_max)  # theta_ddot UB
      prog.AddLinearConstraint(accel >= alpha_min)  # theta_ddot LB

  def add_obstacle_constraint(self, prog, x, x_des, N, obstacles):
    for i in range(N):
      y = x[i][0] + x_des[0]
      z = x[i][1] + x_des[1]
      quad_loc = np.array([y, z])
      for obstacle in obstacles:
        obs_center = np.array(obstacle.center)
        obs_radius = obstacle.radius
        dist = np.linalg.norm(quad_loc - obs_center)
        prog.AddConstraint(dist >= obs_radius + self.obstacle_margin)

  def add_dynamics_constraint(self, prog, x, x_des, u, N, T):
    # TODO: impose dynamics constraint.
    # Use AddLinearEqualityConstraint(expr, value)
    A, B = self.discrete_time_linearized_dynamics(x_des, T)

    for i in range(N-1):
      val = A @ (x[i]) + B @ u[i]
      prog.AddLinearEqualityConstraint((x[i+1])-val, np.zeros(len(x[i])))
    # pass

  # HW6 Cost
  def add_cost(self, prog, x_all, u_all, N):
    # TODO: add cost.
    expr = 0
    # Loop Through Each Cop
    for i in range(len(x_all)):
      # For Each Cop, Add Cost For Knot Point
      for j in range(N - 1):
        val1 = x_all[i][j].T @ self.Q @ x_all[i][j]
        val2 = u_all[i][j].T @ self.R @ u_all[i][j]
        expr += val1 + val2
      # Add End Point for Each Cop
      expr += (x_all[i][N - 1]).T @ self.Qf @ x_all[i][N - 1]
    prog.AddQuadraticCost(expr)

  def add_collision_cost(self, prog, x_all, x_current, x_des, N):
    for n in range(1, N):
      for i in range(len(x_all)):
        for j in range(len(x_all)):
          if not i==j:
            dist = (x_current[i][0][:2] - x_current[j][0][:2]) ** 2

            if dist[0] < self.D_y ** 2 and dist[1] < self.D_z ** 2:
              expr_y = (x_all[i][n][0] - x_current[j][n][0] + x_des[0]) ** 2
              expr_z = (x_all[i][n][1] - x_current[j][n][1] + x_des[1]) ** 2

              prog.AddQuadraticCost(self.w_y * (self.D_y ** 2 - expr_y) + self.w_z * (self.D_z ** 2 - expr_z))

  # def compute_mpc_feedback(self, x_current, x_js, x_des):
  def compute_mpc_feedback(self, x_current, x_des, obstacles, num_cops):
    '''
    This function computes the MPC controller input u
    '''

    # Parameters for the QP
    N = 10
    T = 0.1

    # Initialize mathematical program and decalre decision variables
    prog = MathematicalProgram()
    # Initialize State Decision Variables for Cops
    x_all = []
    for i in range(num_cops):
      x_cop = np.zeros((N, 6), dtype="object")
      x_all.append(x_cop)
    # Create State Decision Variables for Cops
    for i in range(num_cops):
      for j in range(N):
        x_all[i][j] = prog.NewContinuousVariables(6, "x_cop" + str(i) + "_" + str(j))

    # Initialize Input Decision Variables for Cops
    u_all = []
    for i in range(num_cops):
      u_cop = np.zeros((N-1, 2), dtype="object")
      u_all.append(u_cop)
    # Create Input Decision Variables for Cops
    for i in range(num_cops):
      for j in range(N-1):
        u_all[i][j] = prog.NewContinuousVariables(2, "u_cop" + str(i) + "_" + str(j))

    # Add constraints
    for i in range(len(x_all)):
      self.add_initial_state_constraint(prog, x_all[i], x_des[i][0], x_current[i][0])
      self.add_input_saturation_constraint(prog, x_all[i], x_des[i][0], u_all[i], N)
      # self.add_linear_velocity_constraint(prog, x_all[i], x_des[i][0], N)
      # self.add_angular_velocity_constraint(prog, x_all[i], x_des[i][0], N)
      # self.add_acceleration_constraint(prog, x_all[i], x_des[i][0], N)
      # self.add_angular_acceleration_constraint(prog, x_all[i], x_des[i][0], N)
      self.add_obstacle_constraint(prog, x_all[i], x_des[i][0], N, obstacles)
      self.add_dynamics_constraint(prog, x_all[i], x_des[i][0], u_all[i], N, T)
    # Add Cost
    self.add_cost(prog, x_all, u_all, N)
    self.add_collision_cost(prog, x_all, x_current, x_des, N)
    # Solve the QP
    solver = SnoptSolver()
    result = solver.Solve(prog)

    u_mpc = np.zeros((num_cops,2))
    # TODO: retrieve the controller input from the solution of the optimization problem
    # and use it to compute the MPC input u
    # You should make use of result.GetSolution(decision_var) where decision_var
    # is the variable you want
    for i in range(len(x_all)):
      u_mpc[i] = result.GetSolution(u_all[i][0]) + self.u_d(x_des[i][0])


    return u_mpc