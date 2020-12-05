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

class Quadrotor(object):
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

    self.w_x = 1
    self.w_u = 1

    self.w_y = 1
    self.w_z = 1

    self.D_y = 0.75
    self.D_z = 0.2

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


  # HW6 Cost
  def add_cost(self, prog, x, u, N):
    # TODO: add cost.
    expr = 0
    for i in range(N - 1):
      val1 = x[i].T @ self.Q @ x[i]
      val2 = u[i].T @ self.R @ u[i]
      expr += self.w_x*val1 + self.w_u*val2
    expr += self.w_x * (x[N - 1]).T @ self.Qf @ x[N - 1]
    prog.AddQuadraticCost(expr)

  def add_collision_cost(self, prog, x, x_current, x_js, x_des, N):
    for i in range(1, N):
      for x_j in x_js:
        dist = (x_current[:2] - x_j[:2]) ** 2

        #curr_tar_dist = sum(x_current[:2] - x_des[:2] ** 2)
        #j_tar_dist = sum(x_j[:2] - x_des[:2] ** 2)

        if dist[0] < self.D_y ** 2 and dist[1] < self.D_z ** 2:
          expr_y = (x[i][0] - x_j[0] + x_des[0]) ** 2
          expr_z = (x[i][1] - x_j[1] + x_des[1]) ** 2

          prog.AddQuadraticCost(self.w_y * (self.D_y ** 2 - expr_y) + self.w_z * (self.D_z ** 2 - expr_z))

  def compute_mpc_feedback(self, x_current, x_des, x_js, obstacles):
    '''
    This function computes the MPC controller input u
    '''

    # Parameters for the QP
    N = 10
    T = 0.1

    # Initialize mathematical program and decalre decision variables
    prog = MathematicalProgram()
    x = np.zeros((N, 6), dtype="object")
    for i in range(N):
      x[i] = prog.NewContinuousVariables(6, "x_" + str(i))
    u = np.zeros((N-1, 2), dtype="object")
    for i in range(N-1):
      u[i] = prog.NewContinuousVariables(2, "u_" + str(i))

    # print("x_all", x)
    # print("x_des", x_des)
    # print("x_current", x_current)
    # Add constraints and cost
    self.add_initial_state_constraint(prog, x, x_des, x_current)
    self.add_input_saturation_constraint(prog, x, x_des, u, N)
    # self.add_linear_velocity_constraint(prog, x, x_des, N)
    # self.add_angular_velocity_constraint(prog, x, x_des, N)
    # self.add_acceleration_constraint(prog, x, x_des, N)
    # self.add_angular_acceleration_constraint(prog, x, x_des, N)
    self.add_dynamics_constraint(prog, x, x_des, u, N, T)
    self.add_obstacle_constraint(prog, x, x_des, N, obstacles)
    self.add_cost(prog, x, u, N)
    self.add_collision_cost(prog, x, x_current, x_js, x_des, N)

    # Solve the QP
    solver = SnoptSolver()
    result = solver.Solve(prog)

    u_mpc = np.zeros(2)
    # TODO: retrieve the controller input from the solution of the optimization problem
    # and use it to compute the MPC input u
    # You should make use of result.GetSolution(decision_var) where decision_var
    # is the variable you want

    u_mpc = result.GetSolution(u[0]) + self.u_d(x_des)

    return u_mpc