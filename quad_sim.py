import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from quadrotor import Quadrotor
import matplotlib.pyplot as plt

def robber_sim(x_robber, quad_robber, x_des, dt):

    current_x_robber = x_robber[-1]

    current_u_cmd_robber = quad_robber.compute_mpc_feedback(current_x_robber, x_des)
    current_u_robber_real = np.clip(current_u_cmd_robber, quad_robber.umin, quad_robber.umax)

    # Autonomous ODE for constant inputs to work with solve_ivp
    def f_robber(t, x):
        return quad_robber.continuous_time_full_dynamics(current_x_robber, current_u_robber_real)

    # Integrate one step
    sol_rob = solve_ivp(f_robber, (0, dt), current_x_robber, first_step=dt)

    return sol_rob.y[:, -1], current_u_cmd_robber

def cop_sim(x_cop, quad_cop, x_des, xjs, dt):

    current_x_cop = x_cop[-1]

    current_u_cmd_cop = quad_cop.compute_mpc_feedback(current_x_cop, x_des, xjs)

    current_u_cop_real = np.clip(current_u_cmd_cop, quad_cop.umin, quad_cop.umax)

    # Autonomous ODE for constant inputs to work with solve_ivp
    def f_cop(t, x):
        return quad_cop.continuous_time_full_dynamics(current_x_cop, current_u_cop_real)

    # Integrate one step
    sol_cop = solve_ivp(f_cop, (0, dt), current_x_cop, first_step=dt)

    return sol_cop.y[:, -1], current_u_cmd_cop



def simulate_quadrotor(x0_cops, x0_robber, quad_cops, quad_robber, tf, num_cops = 2):
    # Simulates a stabilized maneuver on the 2D quadrotor
    # system, with an initial value of x0
    t0 = 0.0
    n_points = 1000

    dt = 1e-2

    # x_rob_des = np.array([3.5, 1, 0, 1, 1, 0])
    x_rob_des = np.array([0, 0, 0, 0, 0, 0])
    # robber setup
    x_robber = [x0_robber]
    u_robber = [np.zeros((2,))]

    # cops setup
    x_cops = []
    u_cops = []
    for i in range(num_cops):
        x_cops.append([x0_cops[i]])
        u_cops.append([np.zeros((2,))])

    t = [t0]

    eps = 1e-3
    eps_check = True
    while eps_check and t[-1] < tf:

        # current_time = t[-1]
        x_cop_des = x_robber[-1]


        # Compute MPC for robber
        sol_rob, u_cmd_rob = robber_sim(x_robber, quad_robber, x_rob_des, dt)
        x_robber.append(sol_rob)
        u_robber.append(u_cmd_rob)

        xj_curr = []
        for i in range(num_cops):
            xj_curr.append(x_cops[i][-1])

        # compute MPC for cops
        for i in range(num_cops):
            xjs = []
            for j in range(num_cops):
                if not i==j:
                    xjs.append(xj_curr[j])

            sol_cop, u_cmd_cop = cop_sim(x_cops[i], quad_cops[i], x_cop_des, xjs, dt)
            x_cops[i].append(sol_cop)
            u_cops[i].append(u_cmd_cop)

        t.append(t[-1] + dt)

        # calculate norms for all quads
        norms = []
        for i in range(num_cops):
            norm = np.linalg.norm(np.array(x_cops[i][-1][0:2]))
            norms.append(norm)
        eps_check = any(i > eps for i in norms)

        print(t[-1])

    x_cops = np.array(x_cops)
    u_cops = np.array(u_cops)
    t = np.array(t)
    return x_cops, u_cops, x_robber, u_robber, t

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
