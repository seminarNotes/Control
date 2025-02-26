import numpy as np
import matplotlib.pyplot as plt
from casadi import *

class GoalTrajectoryGenerator:
    def __init__(self, t_start=0, t_count=301, t_delta=0.1):
        self.t_start = t_start
        self.t_count = t_count
        self.t_delta = t_delta
        self.trajectory = self.generate_goal_trajectory()

    def Delta(self, t):
        return 0.79 - (np.sin((2 * np.pi * (t + 2)) / 3 + 1) + np.cos((4 * np.pi * (t + 2)) / 3 + 1)) / np.sqrt(4 * t + 12)
    
    def generate_goal_trajectory(self):
        control_input_goal = []
        for idx in range(self.t_count):
            t = self.t_start + idx * self.t_delta
            v_goal = 0.0 - self.Delta(t)
            w_goal = 1.0 - self.Delta(t)
            u_goal = np.array([v_goal, w_goal])
            control_input_goal.append(u_goal)
        return np.array(control_input_goal)

class MobileRobot:
    def __init__(self, initial_state=np.array([0.0, 0.0, 0.0])):
        self.state = initial_state  
    
    def jacobian_S(self, q):
        theta = q[2]
        return np.array([
            [np.cos(theta), 0],
            [np.sin(theta), 0],
            [0, 1]
        ])
    
    def mobile_robot_dynamics(self, u, dt = 0.1):
        dq = dt * self.jacobian_S(self.state) @ u
        self.state += dq 
        self.state[2] = self.normalize_angle(self.state[2])  
    
    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def simulate_motion(self, goal_trajectory):
        x_traj = []
        y_traj = []
        theta_traj = []
        
        for u in goal_trajectory:
            self.mobile_robot_dynamics(u)
            x_traj.append(self.state[0])
            y_traj.append(self.state[1])
            theta_traj.append(self.state[2])
        
        return x_traj, y_traj, theta_traj

class MPCController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
    
    @staticmethod
    def normalize_angle(angle):
        return fmod(angle + np.pi, 2 * np.pi) - np.pi
    
    def solve_mpc(self, q_real, q_goal):
        opti = Opti()
        
        U = opti.variable(2, self.horizon)  
        X = opti.variable(3, self.horizon + 1)  
        
        Q = np.diag([10.0, 10.0, 3.0]) 
        R = np.diag([0.01, 0.02])  
        P = np.diag([10.0, 10.0, 3.0])  
        S = np.diag([10.0, 15.0])  
        
        cost = 0
        X[:, 0] = q_real  
        
        for k in range(self.horizon):
            e = X[:, k] - q_goal
            e[2] = self.normalize_angle(e[2])
            u = U[:, k]
            cost += vertcat(e.T @ Q @ e + u.T @ R @ u)
            
            if k > 0:
                delta_u = U[:, k] - U[:, k - 1]
                cost += vertcat(delta_u.T @ S @ delta_u)
            
            X_next = X[:, k] + self.dt * vertcat(
                U[0, k] * np.cos(X[2, k]),
                U[0, k] * np.sin(X[2, k]),
                U[1, k]
            )
            X_next[2] = self.normalize_angle(X_next[2])
            opti.subject_to(X[:, k + 1] == X_next)
        
        cost += vertcat((X[:, -1] - q_goal).T @ P @ (X[:, -1] - q_goal))
        opti.minimize(cost)
        
        opti.subject_to(opti.bounded(-2.0, U[0, :], 2.0))
        opti.subject_to(opti.bounded(-np.pi, U[1, :], np.pi))
        
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-8}
        opti.solver('ipopt', opts)
        
        sol = opti.solve()
        return sol.value(U[:, 0])
    
def animate_robot_motion(x_goal, y_goal, x_real, y_real):
    plt.figure()
    plt.plot(x_goal, y_goal, linestyle="--", color="blue", label="Planned Trajectory")
    robot_marker, = plt.plot([], [], color='red', marker='^', markersize=10)
    robot_path, = plt.plot([], [], linestyle='-.', color='orange', linewidth=1.5)
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    # plt.legend()
    plt.xlim([-4, 4])
    plt.ylim([-10, 2])
    plt.grid(True)
    
    path_x, path_y = [], []
    
    for i in range(len(x_real)):
        robot_marker.set_data([x_real[i]], [y_real[i]])
        path_x.append(x_real[i])
        path_y.append(y_real[i])
        robot_path.set_data(path_x, path_y)
        plt.pause(0.01)
    
    plt.show()

if __name__ == '__main__' :

    goal_trajectory_generator = GoalTrajectoryGenerator()
    robot_goal = MobileRobot()
    robot_real = MobileRobot(initial_state=np.array([2.0, -2.0, 0.0]))  
    mpc_controller = MPCController()

    goal_traj = goal_trajectory_generator.trajectory
    x_goal, y_goal, theta_goal = robot_goal.simulate_motion(goal_traj)
    x_real, y_real, theta_real = [], [], [] 

    for i in range(len(goal_traj)):
        mpc_control = mpc_controller.solve_mpc(robot_real.state, np.array([x_goal[i], y_goal[i], theta_goal[i]]))
        robot_real.mobile_robot_dynamics(mpc_control)
        x_real.append(robot_real.state[0])
        y_real.append(robot_real.state[1])



    animate_robot_motion(x_goal, y_goal, x_real, y_real)