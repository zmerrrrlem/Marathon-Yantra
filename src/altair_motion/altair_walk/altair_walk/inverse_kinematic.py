import numpy as np
from numpy import sin, cos, clip

class InverseKinematic:
    def __init__(self):
        self.trunk_to_left_hip_yaw_  = np.array([0.02134,  0.055, -0.090436])
        self.trunk_to_right_hip_yaw_ = np.array([0.02134, -0.055, -0.090436])
        self.hip_yaw_to_pitch_ = np.array([-0.003, 0.0, -0.0335])
        self.hip_pitch_to_knee_ = 0.1168
        self.knee_to_ankle_ = 0.1168
        self.ankle_to_sole_ = -0.0355
        self.trunk_tilt_deg = 10.0

        self.LEFT_LEG_AXIS_SIGNS  = np.array([-1, -1, -1, -1,  1,  1])
        self.RIGHT_LEG_AXIS_SIGNS = np.array([-1, -1,  1,  1, -1,  1])

    def Rx(self, t): return np.array([[1,0,0],[0,cos(t),-sin(t)],[0,sin(t),cos(t)]])
    def Ry(self, t): return np.array([[cos(t),0,sin(t)],[0,1,0],[-sin(t),0,cos(t)]])
    def Rz(self, t): return np.array([[cos(t),-sin(t),0],[sin(t),cos(t),0],[0,0,1]])

    def transform_from(self, position, rpy):
        roll, pitch, yaw = rpy
        R = self.Rz(yaw) @ self.Ry(pitch) @ self.Rx(roll)
        T = np.eye(4)
        T[:3,:3], T[:3,3] = R, position
        return T

    def inverse_kinematic(self, COM, LEG, isLeft=True, foot_yaw=0.0, trunk_yaw=0.0):
        # COM: [x, y, z, roll, pitch, yaw] in world (m, radians)
        # LEG: [x, y, z] sole center in world (m)
        # isLeft: True for left leg, False for right leg
        # foot_yaw: desired sole yaw in world (radians)
        # trunk_yaw: body yaw in world (radians); 0 assumes an unrotated trunk
        pos_com = np.array(COM[:3])
        rpy_com = np.array([0, np.deg2rad(self.trunk_tilt_deg), trunk_yaw])
        T_world_trunk = self.transform_from(pos_com, rpy_com)

        pos_foot = np.array(LEG[:3])
        T_world_foot = self.transform_from(pos_foot, [0, 0, foot_yaw])

        if isLeft:
            T_trunk_hipyaw = self.transform_from(self.trunk_to_left_hip_yaw_, [0,0,0])
        else:
            T_trunk_hipyaw = self.transform_from(self.trunk_to_right_hip_yaw_, [0,0,0])

        T_world_hipyaw = T_world_trunk @ T_trunk_hipyaw

        T_hipyaw_foot = np.linalg.inv(T_world_hipyaw) @ T_world_foot

        p_foot = T_hipyaw_foot[:3,3]
        R_foot = T_hipyaw_foot[:3,:3]

        foot_roll, foot_pitch, foot_yaw = 0.0, 0.0, np.arctan2(R_foot[1,0], R_foot[0,0])
        hip_yaw = foot_yaw

        Rz_yaw = self.Rz(hip_yaw)
        hip_yaw_to_pitch = Rz_yaw @ self.hip_yaw_to_pitch_
        p_goal = p_foot - hip_yaw_to_pitch

        p_goal_ankle = p_goal - np.array([0, 0, self.ankle_to_sole_])

        leg_len = np.linalg.norm(p_goal_ankle)
        if leg_len > (self.hip_pitch_to_knee_ + self.knee_to_ankle_):
            print("Target unreachable.")
            return None

        hip_roll = np.arctan2(p_goal_ankle[2], p_goal_ankle[1]) + np.pi/2
        ankle_roll = -hip_roll + foot_roll

        hip_pitch = np.arctan2(p_goal_ankle[2], p_goal_ankle[0]) + np.pi/2
        ankle_pitch = -hip_pitch - foot_pitch

        cos_beta = (self.hip_pitch_to_knee_**2 + self.knee_to_ankle_**2 - leg_len**2) / \
                   (2*self.hip_pitch_to_knee_*self.knee_to_ankle_)
        cos_beta = clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        knee = (np.pi - beta) * -1.0

        hip_pitch -= knee * 0.5
        ankle_pitch -= knee * 0.5

        q = np.array([hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll])
        if isLeft:
            q *= self.LEFT_LEG_AXIS_SIGNS
        else:
            q *= self.RIGHT_LEG_AXIS_SIGNS

        return q

    def solve(self, COM, LEFT, RIGHT, left_yaw=0.0, right_yaw=0.0, trunk_yaw=0.0):
        left_leg = self.inverse_kinematic(COM, LEFT, True, left_yaw, trunk_yaw)
        right_leg = self.inverse_kinematic(COM, RIGHT, False, right_yaw, trunk_yaw)

        if left_leg is None or right_leg is None:
            return None

        result = np.hstack((left_leg, right_leg))
        if result.shape != (12,):
            return None
        return result

if __name__ == "__main__":
    ik = InverseKinematic()

    COM = [0.0, 0.0, 0.37, 0.0, 0.0, 0.0]

    left_foot = [0.0, 0.073, 0.0]
    right_foot = [0.0, -0.073, 0.0]

    q_left = ik.inverse_kinematic(COM, left_foot, isLeft=True)
    q_right = ik.inverse_kinematic(COM, right_foot, isLeft=False)


    print("[hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]")
    print("Left leg angles [rad]:", np.round(q_left, 4))
    print("[hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]")
    print("Right leg angles [rad]:", np.round(q_right, 4))
    print("=====")
    print(ik.solve(COM, left_foot, right_foot))
