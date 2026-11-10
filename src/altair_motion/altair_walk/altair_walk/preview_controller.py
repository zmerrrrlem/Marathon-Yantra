# pip install numpy scipy control matplotlib pytransform3d
import numpy as np
import scipy.linalg as linalg
import control
import matplotlib.pyplot as plt
from pytransform3d.trajectories import *

class PreviewControl:
    def __init__(self, com_height, com_swing, x_offset, foot_distance, n_preview_step=120):
        self.dt = 0.03
        self.g = 9.8
        self.zc = com_height
        self.com_swing = com_swing
        self.x_offset = x_offset
        self.foot_distance = foot_distance
        self.n_preview_steps = n_preview_step
        self.swing_height = 0.035

        self.cmd_x = 0.03
        self.cmd_y = 0.0
        self.cmd_a = np.radians(0)

        self.t_step = 0.6
        self.dsp_ratio = 0.15
        self.t_dsp = self.dsp_ratio * self.t_step
        self.t_ssp = (1.0 - self.dsp_ratio) * self.t_step
        self.t = 0.0
        self.t_bezier = 0.0
        self.dt_bezier = 1 / (self.t_ssp / self.dt)
        # DSP: 0, SSP: 1
        self.walking_phase = 0
        self.walking_ready = False

        self.counter = 1
        self.hip_offset = self.foot_distance + self.com_swing

        # Prediction State
        self.x = np.zeros((3, 1))
        self.y = np.zeros((3, 1))

        # ZMP State
        self.px_ref = []
        self.py_ref = []

        # Initialize footstep
        self.footstep = [[0.0, -self.foot_distance, 0.0],
                         [0.0, self.foot_distance, 0.0],
                         [0.0, -self.foot_distance, 0.0]]
        self.support_foot = -1

        self.com_pose = np.array([0.0, 0.0, 0.0], dtype=float)
        self.left_foot_pose = np.array([0.0, self.foot_distance, 0.0], dtype=float)
        self.right_foot_pose = np.array([0.0, -self.foot_distance, 0.0], dtype=float)
        self.curr_left_foot_pose = np.array([0.0, self.foot_distance, 0.0], dtype=float)
        self.curr_right_foot_pose = np.array([0.0, -self.foot_distance, 0.0], dtype=float)
        self.right = []
        self.left = []

        # Foot yaw state (turning)
        self.curr_left_foot_yaw = 0.0
        self.curr_right_foot_yaw = 0.0
        self.left_foot_pose_yaw = 0.0
        self.right_foot_pose_yaw = 0.0
        self.p_start_yaw = 0.0
        self.p_end_yaw = 0.0
        self.left_yaw_buf = []
        self.right_yaw_buf = []
        self.left_yaw = 0.0
        self.right_yaw = 0.0

        # Bezier Control Point
        self.p_start = np.array([0.0, 0.0, 0.0], dtype=float)
        self.p_cnt = np.array([0.0, 0.0, 0.0], dtype=float)
        self.p_end = np.array([0.0, 0.0, 0.0], dtype=float)

        self.sx = 0.0
        self.sy = 0.0
        self.sa = 0.0

        # A state space of discrete cart table model with jerk input
        # Kajita's Book P:144 Eq:4.72
        self.A = np.array([[1.0, self.dt, 0.5 * self.dt ** 2], 
                           [0.0, 1.0, self.dt], 
                           [0.0, 0.0, 1.0]], dtype=float)
        self.B = np.array([[self.dt**3 / 6.0], 
                           [self.dt**2 / 2.0], 
                           [self.dt]], dtype=float)
        self.C = np.array([[1.0, 0.0, -self.zc / self.g]], dtype=float)

        # Augmented system for preview control, Kajita et al.
        # Used for integrator on the measured output y: zmp error.
        # Kajita's Book P:145 Eq:4.77
        self.A1 = np.block([[np.eye(1), self.C @ self.A], [np.zeros((3, 1)), self.A]])
        self.B1 = np.vstack((self.C @ self.B, self.B))
        self.C1 = np.array([1., 0., 0., 0.])

        self.Q = np.diag([1.0, 1e-6, 1e-6, 1e-6])
        self.R = np.array([[1e-4]])

        # Kajita's Book P:144 Eq:4.75
        P, _, K = control.dare(self.A1, self.B1, self.Q, self.R)
        
        # Kajita's Book P:146 Eq:4.80
        self.Ks = K[0, 0]
        self.Kx = K[0, 1:4].reshape(1, 3)  # Reshape to row vector

        # Compute Gain
        ## Compute Gd
        self.G = np.zeros(self.n_preview_steps)
        self.G[0] = -self.Ks
        A1_closed_loop = self.A1 - self.B1 @ K
        I1 = np.array([[1.], [0.], [0.], [0.]])
        X1 = -A1_closed_loop.T @ P @ I1
        Rinv_term = linalg.inv(self.R + self.B1.T @ P @ self.B1)
        for i in range(1, self.n_preview_steps):
            self.G[i] = (Rinv_term @ (self.B1.T @ X1)).item()
            X1 = A1_closed_loop.T @ X1

    def update_foot_trajectory(self):
        # Update Start and End Foot Trajectory
        if self.t == 0:
            if self.support_foot == 1: # left support, right swings
                self.p_start = self.curr_right_foot_pose.copy()
                self.p_end = np.array([self.footstep[1][0], self.footstep[1][1], 0.0])
                self.p_start_yaw = self.curr_right_foot_yaw
                self.p_end_yaw = self.footstep[1][2]
                self.p_cnt = np.array([
                    0.5*(self.p_start[0] + self.p_end[0]),
                    0.5*(self.p_start[1] + self.p_end[1]),
                    self.swing_height
                ])
                self.p_cnt2 = np.array([
                    self.p_end[0],
                    self.p_end[1],
                    self.swing_height
                ])
            else:
                self.p_start = self.curr_left_foot_pose.copy()
                self.p_end = np.array([self.footstep[1][0], self.footstep[1][1], 0])
                self.p_start_yaw = self.curr_left_foot_yaw
                self.p_end_yaw = self.footstep[1][2]
                self.p_cnt = np.array([
                    0.5*(self.p_start[0] + self.p_end[0]),
                    0.5*(self.p_start[1] + self.p_end[1]),
                    self.swing_height
                ])
                self.p_cnt2 = np.array([
                    self.p_end[0],
                    self.p_end[1],
                    self.swing_height
                ])

        # Update Reset Phase of the Bezier Walking
        if self.t < (self.t_dsp / 2.0) or self.t >= (self.t_dsp / 2.0 + self.t_ssp):
            self.walking_phase = 0
            self.t_bezier = 0
            if self.support_foot == 1:
                self.curr_right_foot_yaw = self.p_end_yaw
            else:
                self.curr_left_foot_yaw = self.p_end_yaw
            self.left_foot_pose_yaw = self.curr_left_foot_yaw
            self.right_foot_pose_yaw = self.curr_right_foot_yaw
        else:
            self.walking_phase = 1
            blend = float(np.clip(self.t_bezier, 0.0, 1.0))
            swing_yaw = self.p_start_yaw + (self.p_end_yaw - self.p_start_yaw) * blend
            if self.support_foot == 1: # KIRI
                self.curr_left_foot_pose[0] = self.footstep[0][0]
                self.curr_left_foot_pose[1] = self.footstep[0][1]
                self.curr_left_foot_pose[2] = 0.0

                self.left_foot_pose_yaw = self.curr_left_foot_yaw
                self.right_foot_pose_yaw = swing_yaw
                self.right_foot_pose = self.update_foot_path()
            else:
                self.curr_right_foot_pose[0] = self.footstep[0][0]
                self.curr_right_foot_pose[1] = self.footstep[0][1]
                self.curr_right_foot_pose[2] = 0.0

                self.right_foot_pose_yaw = self.curr_right_foot_yaw
                self.left_foot_pose_yaw = swing_yaw
                self.left_foot_pose = self.update_foot_path()

        self.left.append([self.left_foot_pose[0], self.left_foot_pose[1], self.left_foot_pose[2]])
        self.right.append([self.right_foot_pose[0], self.right_foot_pose[1], self.right_foot_pose[2]])
        self.left_yaw_buf.append(self.left_foot_pose_yaw)
        self.right_yaw_buf.append(self.right_foot_pose_yaw)

        self.t_bezier += self.dt_bezier
        if len(self.left) == self.n_preview_steps:
            self.left.pop(0)
            self.right.pop(0)
            self.left_yaw_buf.pop(0)
            self.right_yaw_buf.pop(0)

        self.right_foot = np.array(self.right[0])
        self.left_foot = np.array(self.left[0])
        self.left_yaw = self.left_yaw_buf[0]
        self.right_yaw = self.right_yaw_buf[0]

    def update_foot_path(self):
        t = np.clip(self.t_bezier, 0.0, 1.0)
        P0, P1, P2, P3 = self.p_start, self.p_cnt, self.p_cnt2, self.p_end
        return ((1 - t)**3) * P0 + 3*(1 - t)**2*t * P1 + 3*(1 - t)*t**2 * P2 + t**3 * P3

    def swap_support_foot(self):
        # Left: 1, Right: -1
        if self.support_foot == 1:
            self.support_foot = -1
        else:
            self.support_foot = 1

    def update_footstep(self):
        if self.counter % int(self.t_step / self.dt) == 0:
            self.footstep.pop(0)

            if self.support_foot == 1:
                self.sx = self.cmd_x
                self.sy = -2 * self.foot_distance + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy, self.sa])
            else:
                self.sx = self.cmd_x
                self.sy = 2 * self.foot_distance + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy, self.sa])

            self.swap_support_foot()

        self.counter += 1

    def update_pose(self):
        # Updating Foot Trajectory
        self.update_foot_trajectory()

        # Remove the oldest zmp reference if the reference is already at the preview count
        if len(self.px_ref) == self.n_preview_steps:
            self.px_ref.pop(0)
            self.py_ref.pop(0)

        # Add the newest zmp reference from footstep
        self.px_ref.append(self.footstep[0][0])
        self.py_ref.append(self.footstep[0][1])

    def update_preview_control(self):
        # Update COM Trajectory
        # Calculate ZMP error (p_ref - p_actual)
        zmp_x = (self.C @ self.x).item()
        zmp_y = (self.C @ self.y).item()
        
        xe = self.px_ref[0] - zmp_x
        ye = self.py_ref[0] - zmp_y

        G_x, G_y = 0, 0

        for j in range(0, min(self.n_preview_steps, len(self.px_ref))):
            G_x += self.G[j] * self.px_ref[j]
            G_y += self.G[j] * self.py_ref[j]
        
        self.walking_ready = True

        # Kajita's Book P:146 Eq:4.80
        Kx_term_x = (self.Kx @ self.x).item()
        Kx_term_y = (self.Kx @ self.y).item()
        
        ux = -self.Ks * xe - Kx_term_x - G_x
        uy = -self.Ks * ye - Kx_term_y - G_y

        # Kajita's Book P:145 Eq:4.77
        # State prediction or model propagation
        self.x = self.A @ self.x + self.B * ux
        self.y = self.A @ self.y + self.B * uy

        self.com_pose[0] = self.x[0, 0] + self.x_offset
        self.com_pose[1] = self.y[0, 0]
        self.com_pose[2] = self.zc

    def update_walking_pattern(self):
        self.update_pose()
        if len(self.px_ref) == self.n_preview_steps:
            self.update_preview_control()

        self.t += self.dt
        if self.t > self.t_step:
            self.t = 0
        # Update ZMP
        self.update_footstep()

    def debug(self):
        t_sim = 8
        t = 0
        com_x = []
        com_y = []

        px = []
        py = []

        com_trajectory = []
        r_trajectory = []
        l_trajectory = []

        while t < t_sim:
            self.update_walking_pattern()
            t += self.dt
            com_x.append(self.x[0, 0])
            com_y.append(self.y[0, 0])

            px.append(self.px_ref[0])
            py.append(self.py_ref[0])

            com_trajectory.append([self.x[0, 0], self.y[0, 0], self.zc, 0, 0, 0, 0])
            r_trajectory.append([self.right_foot[0], self.right_foot[1], self.right_foot[2], 0, 0, 0, 0])
            l_trajectory.append([self.left_foot[0], self.left_foot[1], self.left_foot[2], 0, 0, 0, 0])

        plt.figure(0)
        plt.plot(px, label='ZMP X')
        plt.plot(com_x, label='COM X')
        plt.legend()
        plt.title('X-direction')

        plt.figure(1)
        plt.plot(py, label='ZMP Y')
        plt.plot(com_y, label='COM Y')
        plt.legend()
        plt.title('Y-direction')

        plt.figure(2)
        plt.plot(com_x, com_y, label='COM Trajectory')
        plt.plot(px, py, label='ZMP Reference')
        plt.legend()
        plt.title('COM and ZMP Trajectories')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.axis('equal')

        fig = plt.figure(3)
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim([0, 0.4])
        ax.set_ylim([-0.075, 0.075])
        ax.set_zlim([0, 0.5])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')

        com_trajectory = np.array(com_trajectory)
        l_trajectory = np.array(l_trajectory)
        r_trajectory = np.array(r_trajectory)

        plot_trajectory(ax=ax, P=com_trajectory, s=0.02, show_direction=False)
        plot_trajectory(ax=ax, P=r_trajectory, s=0.02, show_direction=False)
        plot_trajectory(ax=ax, P=l_trajectory, s=0.02, show_direction=False)

        print(f"COM at Index 10: ", com_trajectory[10])
        print(f"l_trajectory at Index 10: ", l_trajectory[10])
        print(f"r_trajectory at Index 10: ", r_trajectory[10])

        plt.show()


def main():
    pc = PreviewControl(com_height=370/1000, com_swing=100.5/1000, x_offset=5/1000, foot_distance=58.5 / 1000)
    pc.debug()

if __name__ == "__main__":
    main()