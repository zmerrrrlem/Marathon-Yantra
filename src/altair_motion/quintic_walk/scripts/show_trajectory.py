#!/usr/bin/env python3
"""Plot WalkEngine global-frame trajectories produced by walk_engine_trajectory_test."""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

try:
    # Registers 3D projection when available
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    HAS_3D = True
except Exception:
    HAS_3D = False


def load_trajectory(csv_path: Path) -> Dict[str, List[float]]:
    data: Dict[str, List[float]] = {
        "time": [],
        "left_foot_x": [],
        "left_foot_y": [],
        "left_foot_z": [],
        "right_foot_x": [],
        "right_foot_y": [],
        "right_foot_z": [],
        "com_x": [],
        "com_y": [],
        "com_z": [],
    }

    with csv_path.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            for key in data:
                data[key].append(float(row[key]))

    return data


def finite_diff(series: List[float], dt: float) -> np.ndarray:
    arr = np.asarray(series)
    if len(arr) < 3:
        return np.zeros_like(arr)
    vel = np.gradient(arr, dt)
    return vel


def plot(data: Dict[str, List[float]], save_path: Optional[Path]) -> None:
    fig_xy = plt.figure(figsize=(6, 6))
    ax_xy = fig_xy.add_subplot(1, 1, 1)
    ax_xy.plot(
        data["left_foot_x"], data["left_foot_y"], label="Left foot", color="tab:blue"
    )
    ax_xy.plot(
        data["right_foot_x"],
        data["right_foot_y"],
        label="Right foot",
        color="tab:green",
    )
    ax_xy.plot(data["com_x"], data["com_y"], label="COM", color="tab:orange")
    ax_xy.set_title("XY projection")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True)
    ax_xy.legend()
    fig_xy.tight_layout()

    # z plane
    fig_z = plt.figure(figsize=(8, 4))
    ax_height = fig_z.add_subplot(1, 1, 1)
    ax_height.plot(
        data["time"], data["left_foot_z"], label="Left foot z", color="tab:blue"
    )
    ax_height.plot(
        data["time"], data["right_foot_z"], label="Right foot z", color="tab:green"
    )
    ax_height.plot(data["time"], data["com_z"], label="COM z", color="tab:orange")
    ax_height.set_title("Height vs time")
    ax_height.set_xlabel("Time [s]")
    ax_height.set_ylabel("Height [m]")
    ax_height.grid(True)
    ax_height.legend()
    fig_z.tight_layout()

    if HAS_3D:
        fig_3d = plt.figure(figsize=(7, 6))
        ax_3d = fig_3d.add_subplot(1, 1, 1, projection="3d")
        ax_3d.plot3D(
            data["left_foot_x"],
            data["left_foot_y"],
            data["left_foot_z"],
            label="Left foot",
            color="tab:blue",
            alpha=0.85,
        )
        ax_3d.plot3D(
            data["right_foot_x"],
            data["right_foot_y"],
            data["right_foot_z"],
            label="Right foot",
            color="tab:green",
            alpha=0.85,
        )
        ax_3d.plot3D(
            data["com_x"],
            data["com_y"],
            data["com_z"],
            label="COM",
            color="tab:orange",
            alpha=0.95,
        )
        ax_3d.set_title("3D trajectory (time-aligned)")
        ax_3d.set_xlabel("X [m]")
        ax_3d.set_ylabel("Y [m]")
        ax_3d.set_zlabel("Z [m]")
        ax_3d.legend()

        all_x = data["left_foot_x"] + data["right_foot_x"] + data["com_x"]
        all_y = data["left_foot_y"] + data["right_foot_y"] + data["com_y"]
        all_z = data["left_foot_z"] + data["right_foot_z"] + data["com_z"]
        x_mid = (max(all_x) + min(all_x)) * 0.5
        y_mid = (max(all_y) + min(all_y)) * 0.5
        z_mid = (max(all_z) + min(all_z)) * 0.5
        radius = (
            max(
                (max(all_x) - min(all_x)),
                (max(all_y) - min(all_y)),
                (max(all_z) - min(all_z)),
            )
            * 0.5
        )
        ax_3d.set_xlim([x_mid - radius, x_mid + radius])
        ax_3d.set_ylim([y_mid - radius, y_mid + radius])
        ax_3d.set_zlim([z_mid - radius, z_mid + radius])
        fig_3d.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=200)
        print(f"Saved figure to {save_path}")

    plt.show()


def animate(data: Dict[str, List[float]], fps: int, save_path: Optional[Path]) -> None:
    time = np.asarray(data["time"])
    dt = float(time[1] - time[0]) if len(time) > 1 else 0.01

    com = np.vstack((data["com_x"], data["com_y"], data["com_z"])).T
    com_vel = np.vstack(
        (
            finite_diff(data["com_x"], dt),
            finite_diff(data["com_y"], dt),
            finite_diff(data["com_z"], dt),
        )
    ).T
    com_acc = np.vstack(
        (
            finite_diff(com_vel[:, 0], dt),
            finite_diff(com_vel[:, 1], dt),
            finite_diff(com_vel[:, 2], dt),
        )
    ).T

    fig = plt.figure(figsize=(18, 5))
    ax_xy = fig.add_subplot(1, 3, 1)
    ax_zy = fig.add_subplot(1, 3, 2)
    ax_3d = fig.add_subplot(1, 3, 3, projection="3d") if HAS_3D else None

    ax_xy.plot(data["left_foot_x"], data["left_foot_y"], color="tab:blue", alpha=0.2)
    ax_xy.plot(data["right_foot_x"], data["right_foot_y"], color="tab:green", alpha=0.2)
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True)

    ax_zy.plot(time, data["com_z"], color="tab:orange", alpha=0.3)
    ax_zy.set_xlabel("Time [s]")
    ax_zy.set_ylabel("Z [m]")
    ax_zy.grid(True)

    if ax_3d:
        ax_3d.plot3D(
            data["left_foot_x"],
            data["left_foot_y"],
            data["left_foot_z"],
            color="tab:blue",
            alpha=0.2,
        )
        ax_3d.plot3D(
            data["right_foot_x"],
            data["right_foot_y"],
            data["right_foot_z"],
            color="tab:green",
            alpha=0.2,
        )
        ax_3d.set_xlabel("X [m]")
        ax_3d.set_ylabel("Y [m]")
        ax_3d.set_zlabel("Z [m]")

        all_x = data["left_foot_x"] + data["right_foot_x"] + data["com_x"]
        all_y = data["left_foot_y"] + data["right_foot_y"] + data["com_y"]
        all_z = data["left_foot_z"] + data["right_foot_z"] + data["com_z"]
        x_mid = (max(all_x) + min(all_x)) * 0.5
        y_mid = (max(all_y) + min(all_y)) * 0.5
        z_mid = (max(all_z) + min(all_z)) * 0.5
        radius = (
            max(
                (max(all_x) - min(all_x)),
                (max(all_y) - min(all_y)),
                (max(all_z) - min(all_z)),
            )
            * 0.5
        )
        ax_3d.set_xlim([x_mid - radius, x_mid + radius])
        ax_3d.set_ylim([y_mid - radius, y_mid + radius])
        ax_3d.set_zlim([z_mid - radius, z_mid + radius])
    else:
        fig.delaxes(fig.axes[-1])
        fig.set_size_inches(12, 5)

    com_dot_xy = ax_xy.plot([], [], "o", color="tab:orange")[0]
    com_vec_xy = ax_xy.quiver(
        [], [], [], [], color="tab:red", angles="xy", scale_units="xy", scale=1.0
    )

    com_dot_zy = ax_zy.plot([], [], "o", color="tab:orange")[0]
    vel_line = ax_zy.plot([], [], label="|v|", color="tab:red")[0]
    acc_line = ax_zy.plot([], [], label="|a|", color="tab:purple")[0]
    ax_zy.legend()

    com_dot_3d = ax_3d.plot([], [], [], "o", color="tab:orange")[0] if ax_3d else None

    speed = np.linalg.norm(com_vel, axis=1)
    accel = np.linalg.norm(com_acc, axis=1)

    def init():
        com_dot_xy.set_data([], [])
        com_vec_xy.set_offsets(np.empty((0, 2)))
        com_vec_xy.set_UVC(np.array([]), np.array([]))
        com_dot_zy.set_data([], [])
        vel_line.set_data([], [])
        acc_line.set_data([], [])
        if com_dot_3d:
            com_dot_3d.set_data_3d([], [], [])
        return tuple(
            artist
            for artist in (
                com_dot_xy,
                com_vec_xy,
                com_dot_zy,
                vel_line,
                acc_line,
                com_dot_3d,
            )
            if artist
        )

    def update(frame: int):
        com_dot_xy.set_data(com[frame, 0], com[frame, 1])
        if com_vec_xy:
            com_vec_xy.set_offsets([[com[frame, 0], com[frame, 1]]])
            com_vec_xy.set_UVC(com_vel[frame, 0], com_vel[frame, 1])

        com_dot_zy.set_data(time[frame], com[frame, 2])
        vel_line.set_data(time[: frame + 1], speed[: frame + 1])
        acc_line.set_data(time[: frame + 1], accel[: frame + 1])

        if com_dot_3d:
            com_dot_3d.set_data_3d(com[frame, 0], com[frame, 1], com[frame, 2])

        return tuple(
            artist
            for artist in (
                com_dot_xy,
                com_vec_xy,
                com_dot_zy,
                vel_line,
                acc_line,
                com_dot_3d,
            )
            if artist
        )

    interval_ms = 1000.0 / max(fps, 1)
    frames = len(time)
    anim = animation.FuncAnimation(
        fig, update, init_func=init, frames=frames, interval=interval_ms, blit=False
    )

    fig.tight_layout()

    if save_path:
        anim_path = save_path.with_suffix(".mp4")
        try:
            writer = animation.writers["ffmpeg"]
            anim.save(anim_path, writer=writer(fps=fps))
            print(f"Saved animation to {anim_path}")
        except Exception:
            print(
                "Animation save skipped (ffmpeg writer not available). Showing interactively."
            )

    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot gait trajectory CSV emitted by walk_engine_trajectory_test"
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default="walk_engine_trajectory.csv",
        type=Path,
        help="Path to trajectory CSV (default: walk_engine_trajectory.csv)",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional path to save the rendered figure (skips saving when omitted)",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Show a short animation with position/velocity/acceleration overlays",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Animation framerate (default: 30)",
    )
    args = parser.parse_args()

    data = load_trajectory(args.csv)
    if args.animate:
        animate(data, args.fps, args.save)
    else:
        plot(data, args.save)


if __name__ == "__main__":
    main()
