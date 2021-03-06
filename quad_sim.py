import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d
from quadrotor import Quadrotor
import matplotlib.pyplot as plt
import time

def closest_cop(cop_positions, robber_position):
    # initialize
    dist = np.inf

    # calculate closes cop to robber
    for i in cop_positions:
        dist_new = (((i[0] - robber_position[0]) ** 2) +  ((i[1] - robber_position[1]) ** 2) ** (1/2))
        if dist_new < dist:
            dist = dist_new
            close_cop_pos = i

    close_cop = np.array([close_cop_pos[0], close_cop_pos[1], 0,0,0,0])

    return close_cop

def furthest_point_from_cop(close_cop, env_extents):
    # initialize
    dist = -1
    furthest_point = np.array([0, 0])

    # calculate furthest point from cop in environment
    for i in env_extents:
        dist_new = (((i[0] - close_cop[0]) ** 2) + ((i[1] - close_cop[1]) ** 2) ** (1 / 2))
        if dist_new > dist:  # and dist_new < 10:
            dist = dist_new
            furthest_point = i
    furthest_point = np.array([furthest_point[0], furthest_point[1], 0, 0, 0, 0])

    return furthest_point


def robber_desired_pos(x_rob_des, furthest_point, obstacles):
    margin = 0.5

    # calculate furthest point from

    # interpolation function between current goal and furthest point
    x = [x_rob_des[0], furthest_point[0]]
    y = [x_rob_des[1], furthest_point[1]]
    f = interp1d(x,y)

    # calculate next robber desired position using interpolation
    xnew = x_rob_des[0] + ((furthest_point[0] - x_rob_des[0]) / 2000).item()
    ynew = f(xnew).item()
    new_x_rob_des = np.array([xnew, ynew])

    # check that next robber desired position doesnt interfere with obstacles
    for obs in obstacles:
        obs_center = np.array(obs.center)
        obs_radius = obs.radius
        dist = np.linalg.norm(new_x_rob_des - obs_center)
        if dist < obs_radius + margin:
            new_x_rob_des = np.array([xnew + .1, ynew])

    return new_x_rob_des

def robber_sim(x_robber, quad_robber, x_des, x_js, dt, obstacles):

    current_x_robber = x_robber[-1]

    current_u_cmd_robber = quad_robber.compute_mpc_feedback(current_x_robber, x_des, x_js, obstacles)
    current_u_robber_real = np.clip(current_u_cmd_robber, quad_robber.umin, quad_robber.umax)

    # Autonomous ODE for constant inputs to work with solve_ivp
    def f_robber(t, x):
        return quad_robber.continuous_time_full_dynamics(current_x_robber + x_des, current_u_robber_real)

    # Integrate one step
    sol_rob = solve_ivp(f_robber, (0, dt), current_x_robber, first_step=dt)

    return sol_rob.y[:, -1], current_u_cmd_robber

def cop_sim(x_cop, quad_cop, x_des, xjs, dt, obstacles):

    current_x_cop = x_cop[-1].copy()

    current_u_cmd_cop = quad_cop.compute_mpc_feedback(current_x_cop, x_des, xjs, obstacles)

    current_u_cop_real = np.clip(current_u_cmd_cop, quad_cop.umin, quad_cop.umax)

    # Autonomous ODE for constant inputs to work with solve_ivp
    def f_cop(t, x):
        return quad_cop.continuous_time_full_dynamics(current_x_cop + x_des, current_u_cop_real)

    # Integrate one step
    sol_cop = solve_ivp(f_cop, (0, dt), current_x_cop, first_step=dt)

    return sol_cop.y[:, -1], current_u_cmd_cop

def simulate_quadrotor(x0_cops, x0_robber, quad_cops, quad_robber, tf, num_cops, obstacles, x0_rob_des):
    # Simulates a stabilized maneuver on the 2D quadrotor
    # system, with an initial value of x0
    t0 = 0.0
    n_points = 1000
    dt = 1e-2

    # rectangle corners of environment
    env_extents = np.array([[0, 0],
                            [40, 0],
                            [0, 20],
                            [40, 20]])

    # robber setup
    x_robber = [x0_robber]
    u_robber = [np.zeros((2,))]
    x_rob_des = x0_rob_des
    x_rob_des_list = []

    # robber desired location (furthest point in map from intialization)
    # distances = np.linalg.norm(np.ones((4,2)) * x_rob_des[0:2] - env_extents, axis = 1)
    # furthest_point = env_extents[np.argmax(distances)]


    # cops setup
    x_cops = []
    u_cops = []
    x_cop_des = []
    x_cops_current = np.zeros((num_cops, 2))
    for i in range(num_cops):
        x_cops_current[i] = x0_cops[i][0:2]
        x_cops.append([x0_cops[i]])
        u_cops.append([np.zeros((2,))])

    t = [t0]
    eps_y = 0.5
    eps_z = 0.05
    eps_check = True

    start = time.time()

    while eps_check and t[-1] < tf:

        # get position of closest cop
        close_cop = closest_cop(x_cops_current, x_robber[-1])

        # get furthest point from closest cop
        furthest_point = furthest_point_from_cop(close_cop, env_extents)

        # static robber desired position
        # x_rob_des_list.append(x0_rob_des)

        # moving robber desired position
        x_rob_des = robber_desired_pos(x_rob_des, furthest_point, obstacles)
        x_rob_des_list.append(np.array([x_rob_des[0], x_rob_des[1], 0, 0, 0, 0]))

        # re-initialize current positions of all cops
        x_cops_current = np.zeros((num_cops, 2))

        # current_time = t[-1]
        x_cop_des.append(np.array([x_robber[-1][0], x_robber[-1][1], 0, 0, 0, 0]))

        xj_curr = []
        for i in range(num_cops):
            xj_curr.append(x_cops[i][-1])

        # Compute MPC for robber
        sol_rob, u_cmd_rob = robber_sim(x_robber, quad_robber, x_rob_des_list[-1], xj_curr, dt, obstacles)
        x_robber.append(sol_rob)
        u_robber.append(u_cmd_rob)

        # compute MPC for cops
        for i in range(num_cops):
            xjs = []
            for j in range(num_cops):
                if not i==j:
                    xjs.append(xj_curr[j])

            sol_cop, u_cmd_cop = cop_sim(x_cops[i], quad_cops[i], x_cop_des[-1], xjs, dt, obstacles)
            x_cops_current[i] = x_cops[i][-1][0:2]
            x_cops[i].append(sol_cop)
            u_cops[i].append(u_cmd_cop)

        t.append(t[-1] + dt)

        # determine if cops captured robber
        for i in range(num_cops):
            y_dist = x_cops[i][-1][0] - x_cop_des[-1][0]
            z_dist = x_cops[i][-1][1] - x_cop_des[-1][1]
            # print("Distances for cop {}: y = {}, z = {}".format(i, y_dist, z_dist))
            if eps_check:
                eps_check = not (abs(y_dist) <= eps_y and abs(z_dist) <= eps_z)

        print("time: {:.2f}".format(t[-1]))
        # print("desired: {}\n".format(x_cop_des[-1]))

    end = time.time()
    t_elapsed = end - start
    x_cops = np.array(x_cops)
    x_cop_des = np.array(x_cop_des)
    u_cops = np.array(u_cops)
    x_rob_des_list = np.array(x_rob_des_list)
    t = np.array(t)
    return x_cops, x_cop_des, u_cops, x_robber, x_rob_des_list, u_robber, t, t_elapsed

def plot_x_and_u(x, u, t, name):
    plt.figure()
    ax = plt.axes()
    plt.plot(0, 0, 'o', label='target position')
    plt.plot(x[0, 0], x[0, 1], 'o', label='initial position')
    plt.plot(x[:, 0], x[:, 1], label='actual trajectory')
    plt.xlabel("y (m)")
    plt.ylabel("z (m)")
    plt.legend()
    ax.set_aspect('equal', 'datalim')
    ax.legend(loc='upper right')
    plt.title("Quadrotor trajectory (" + name + ")")

    plt.figure()
    plt.plot(t[1:], u[1:])
    plt.xlabel("time (s)")
    plt.ylabel("u (N)")
    plt.legend(["u1", "u2"])
    plt.title(name + " commanded inputs")


if __name__ == '__main__':
    R = np.eye(2);
    Q = np.diag([10, 10, 1, 1, 1, 1]);
    Qf = Q;

    quadrotor = Quadrotor(Q, R, Qf);

    # Initial state
    d_rand = 1
    x0 = np.array([0.5, 0.5, 0, 1, 1, 0])

    tf = 10;

    x, u, t = simulate_quadrotor(x0, tf, quadrotor)
    plot_x_and_u(x, u, t, "MPC")

    plt.show()