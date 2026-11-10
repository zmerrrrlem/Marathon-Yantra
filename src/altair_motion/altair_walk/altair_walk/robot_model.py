# robot_model.py — kept as a stub; meshcat / pinocchio visualization removed.
# altair_walk no longer imports this module. The altair_description package
# (required by the original Pinocchio URDF loader) is not in this workspace.
import numpy as np


class RobotModel:
    """Placeholder retained for API compatibility. No visualization backend."""

    def __init__(self):
        self.model = None
        self.data = None
        self.q = None
        self.left_foot_id = None
        self.right_foot_id = None
        self.torso_id = None

    def open_visualizer(self):
        pass

    def update_visualizer(self, q):
        pass


def print_joints(model):
    if model is None:
        print("RobotModel not initialized (pinocchio/altair_description unavailable).")
        return
    print(f"{'ID':>3} | {'q_index':>7} | {'nq':>3} | Joint Name")
    print("-" * 40)
    for j_id, j_name in enumerate(model.names):
        joint = model.joints[j_id]
        print(f"{j_id:3d} | {joint.idx_q:7d} | {joint.nq:3d} | {j_name}")


def print_frames(model):
    if model is None:
        print("RobotModel not initialized (pinocchio/altair_description unavailable).")
        return
    for i, frame in enumerate(model.frames):
        print(i, frame.name, frame.parent, frame.type)


def main():
    robot_model = RobotModel()
    print(robot_model.q)
    print_joints(robot_model.model)


if __name__ == "__main__":
    main()
