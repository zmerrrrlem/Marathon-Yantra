import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the robot's dimensions (from the provided data)
k = {
    'HIP_OFFSET_X': 0.02134,  # X-offset of the hip from the base
    'LEG_SIDE_OFFSET': 0.055,  # Y-offset of the hip from the base
    'HIP_OFFSET_Z': 0.090436,  # Z-offset of the hip from the base
    'THIGH_LENGTH': 0.1168,  # Length of the thigh (hip to knee)
    'CALF_LENGTH': 0.1168,  # Length of the calf (knee to ankle)
    'ANKLE_LENGTH': 0.0355,  # Length of the ankle (ankle to foot)
}

# Define the kicking parameters (from the provided data)
kicking_param = {
    'Torso_X': -0.03,
    'Torso_Y': 0.005,
    'Torso_Z': 0.350,
    'Torso_Pitch': 20 * np.pi / 180,  # Convert degrees to radians
    'Shift_Roll': 16 * np.pi / 180,
    'Lift_Pitch': 0 * np.pi / 180,
    'Lift_Roll': 25 * np.pi / 180,
    'Swing_Pitch': 0 * np.pi / 180,
    'Swing_Roll': 25 * np.pi / 180,
    'Retract_Pitch': 25 * np.pi / 180,
    'Retract_Roll': 25 * np.pi / 180,
    'R_Shift_X': -0.008,
    'R_Shift_Y': 0.11,
    'R_Shift_Z': 0.356,
    'R_Lift_X': -0.1,
    'R_Lift_Y': -0.045,
    'R_Lift_Z': 0.12,
    'R_Swing_X': 0.15,
    'R_Swing_Y': -0.047,
    'R_Swing_Z': 0.125,
    'R_Retract_X': 0.045,
    'R_Retract_Y': -0.052,
    'R_Retract_Z': 0.135,
    'L_Shift_X': -0.010,
    'L_Shift_Y': -0.093,
    'L_Shift_Z': 0.355,
    'L_Lift_X': -0.139,
    'L_Lift_Y': 0.053,
    'L_Lift_Z': 0.145,
    'L_Swing_X': 0.185,
    'L_Swing_Y': 0.055,
    'L_Swing_Z': 0.135,
    'L_Retract_X': 0.040,
    'L_Retract_Y': 0.065,
    'L_Retract_Z': 0.120,
    'SHIFT_TIME': 2.0,
    'LIFT_TIME': 0.3,
    'SWING_TIME': 0.21,
    'RETRACT_TIME': 0.25,
    'LANDING_TIME': 0.29,
    'FINISHED_TIME': 0.25
}

# Define the transformations
BASE = np.eye(4)  # Identity matrix for the base frame

# Right leg transformations
R_HIP = np.array([
    [1, 0, 0, k['HIP_OFFSET_X']],
    [0, 1, 0, -k['LEG_SIDE_OFFSET']],
    [0, 0, 1, -k['HIP_OFFSET_Z']],
    [0, 0, 0, 1]
])

R_KNEE = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['THIGH_LENGTH']],
    [0, 0, 0, 1]
])

R_ANKLE = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['CALF_LENGTH']],
    [0, 0, 0, 1]
])

R_FOOT = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['ANKLE_LENGTH']],
    [0, 0, 0, 1]
])

# Left leg transformations
L_HIP = np.array([
    [1, 0, 0, k['HIP_OFFSET_X']],
    [0, 1, 0, k['LEG_SIDE_OFFSET']],
    [0, 0, 1, -k['HIP_OFFSET_Z']],
    [0, 0, 0, 1]
])

L_KNEE = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['THIGH_LENGTH']],
    [0, 0, 0, 1]
])

L_ANKLE = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['CALF_LENGTH']],
    [0, 0, 0, 1]
])

L_FOOT = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, -k['ANKLE_LENGTH']],
    [0, 0, 0, 1]
])

# Function to apply transformations
def apply_transform(transform, point):
    return transform @ np.append(point, 1)  # Append 1 for homogeneous coordinates

# Function to compute Bezier curves
def bezierCurve2(phase, start, end):
    return (1 - phase) * start + phase * end

def bezierCurve3(phase, start, inter, end):
    return (1 - phase)**2 * start + 2 * (1 - phase) * phase * inter + phase**2 * end

# Initialize the points dictionary
points = {
    'BASE': np.array([0, 0, 0]),
    'R_HIP': np.array([0, 0, 0]),
    'R_KNEE': np.array([0, 0, 0]),
    'R_ANKLE': np.array([0, 0, 0]),
    'R_FOOT': np.array([0, 0, 0]),
    'L_HIP': np.array([0, 0, 0]),
    'L_KNEE': np.array([0, 0, 0]),
    'L_ANKLE': np.array([0, 0, 0]),
    'L_FOOT': np.array([0, 0, 0])
}

# Function to update the plot for each frame of the animation
def update(frame):
    phase = frame / 100  # Phase ranges from 0 to 1

    # Simulate the kicking motion based on the phase
    if phase < 0.2:
        # Shift phase
        Torso = bezierCurve2(phase, np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]),
                             np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]))
        R_Leg = bezierCurve2(phase, np.array([0, -0.07, 0]), np.array([kicking_param['R_Shift_X'], kicking_param['R_Shift_Y'], kicking_param['R_Shift_Z']]))
        L_Leg = bezierCurve2(phase, np.array([0, 0.07, 0]), np.array([kicking_param['L_Shift_X'], kicking_param['L_Shift_Y'], kicking_param['L_Shift_Z']]))
    elif phase < 0.4:
        # Lift phase
        Torso = bezierCurve2(phase, np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]),
                             np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]))
        R_Leg = bezierCurve2(phase, np.array([kicking_param['R_Shift_X'], kicking_param['R_Shift_Y'], kicking_param['R_Shift_Z']]),
                             np.array([kicking_param['R_Lift_X'], kicking_param['R_Lift_Y'], kicking_param['R_Lift_Z']]))
        L_Leg = bezierCurve2(phase, np.array([kicking_param['L_Shift_X'], kicking_param['L_Shift_Y'], kicking_param['L_Shift_Z']]),
                             np.array([kicking_param['L_Lift_X'], kicking_param['L_Lift_Y'], kicking_param['L_Lift_Z']]))
    elif phase < 0.6:
        # Swing phase
        Torso = bezierCurve2(phase, np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]),
                             np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]))
        R_Leg = bezierCurve3(phase, np.array([kicking_param['R_Lift_X'], kicking_param['R_Lift_Y'], kicking_param['R_Lift_Z']]),
                             np.array([0, -0.07, 0.2]), np.array([kicking_param['R_Swing_X'], kicking_param['R_Swing_Y'], kicking_param['R_Swing_Z']]))
        L_Leg = bezierCurve3(phase, np.array([kicking_param['L_Lift_X'], kicking_param['L_Lift_Y'], kicking_param['L_Lift_Z']]),
                             np.array([0, 0.07, 0.2]), np.array([kicking_param['L_Swing_X'], kicking_param['L_Swing_Y'], kicking_param['L_Swing_Z']]))
    elif phase < 0.8:
        # Retract phase
        Torso = bezierCurve2(phase, np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]),
                             np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']]))
        R_Leg = bezierCurve2(phase, np.array([kicking_param['R_Swing_X'], kicking_param['R_Swing_Y'], kicking_param['R_Swing_Z']]),
                             np.array([kicking_param['R_Retract_X'], kicking_param['R_Retract_Y'], kicking_param['R_Retract_Z']]))
        L_Leg = bezierCurve2(phase, np.array([kicking_param['L_Swing_X'], kicking_param['L_Swing_Y'], kicking_param['L_Swing_Z']]),
                             np.array([kicking_param['L_Retract_X'], kicking_param['L_Retract_Y'], kicking_param['L_Retract_Z']]))
    else:
        # Finish phase
        Torso = np.array([kicking_param['Torso_X'], kicking_param['Torso_Y'], kicking_param['Torso_Z']])
        R_Leg = np.array([0, -0.07, 0])
        L_Leg = np.array([0, 0.07, 0])

    # Update the points for visualization
    points['BASE'] = Torso
    points['R_HIP'] = Torso + np.array([k['HIP_OFFSET_X'], -k['LEG_SIDE_OFFSET'], -k['HIP_OFFSET_Z']])
    points['R_KNEE'] = points['R_HIP'] + np.array([0, 0, -k['THIGH_LENGTH']])
    points['R_ANKLE'] = points['R_KNEE'] + np.array([0, 0, -k['CALF_LENGTH']])
    points['R_FOOT'] = points['R_ANKLE'] + np.array([0, 0, -k['ANKLE_LENGTH']])

    points['L_HIP'] = Torso + np.array([k['HIP_OFFSET_X'], k['LEG_SIDE_OFFSET'], -k['HIP_OFFSET_Z']])
    points['L_KNEE'] = points['L_HIP'] + np.array([0, 0, -k['THIGH_LENGTH']])
    points['L_ANKLE'] = points['L_KNEE'] + np.array([0, 0, -k['CALF_LENGTH']])
    points['L_FOOT'] = points['L_ANKLE'] + np.array([0, 0, -k['ANKLE_LENGTH']])

    # Clear the previous plot
    ax.clear()

    # Plot the kinematic chain
    ax.plot([points['BASE'][0], points['R_HIP'][0]],
            [points['BASE'][1], points['R_HIP'][1]],
            [points['BASE'][2], points['R_HIP'][2]], 'r-', label='Right Leg')
    ax.plot([points['R_HIP'][0], points['R_KNEE'][0]],
            [points['R_HIP'][1], points['R_KNEE'][1]],
            [points['R_HIP'][2], points['R_KNEE'][2]], 'r-')
    ax.plot([points['R_KNEE'][0], points['R_ANKLE'][0]],
            [points['R_KNEE'][1], points['R_ANKLE'][1]],
            [points['R_KNEE'][2], points['R_ANKLE'][2]], 'r-')
    ax.plot([points['R_ANKLE'][0], points['R_FOOT'][0]],
            [points['R_ANKLE'][1], points['R_FOOT'][1]],
            [points['R_ANKLE'][2], points['R_FOOT'][2]], 'r-')

    ax.plot([points['BASE'][0], points['L_HIP'][0]],
            [points['BASE'][1], points['L_HIP'][1]],
            [points['BASE'][2], points['L_HIP'][2]], 'b-', label='Left Leg')
    ax.plot([points['L_HIP'][0], points['L_KNEE'][0]],
            [points['L_HIP'][1], points['L_KNEE'][1]],
            [points['L_HIP'][2], points['L_KNEE'][2]], 'b-')
    ax.plot([points['L_KNEE'][0], points['L_ANKLE'][0]],
            [points['L_KNEE'][1], points['L_ANKLE'][1]],
            [points['L_KNEE'][2], points['L_ANKLE'][2]], 'b-')
    ax.plot([points['L_ANKLE'][0], points['L_FOOT'][0]],
            [points['L_ANKLE'][1], points['L_FOOT'][1]],
            [points['L_ANKLE'][2], points['L_FOOT'][2]], 'b-')

    # Plot points
    for key, point in points.items():
        ax.scatter(point[0], point[1], point[2], label=key)

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])

# Create the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create the animation
ani = FuncAnimation(fig, update, frames=100, interval=50, repeat=True)

# Show the plot
plt.show()