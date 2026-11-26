import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the robot's dimensions (from the provided data)
k = {
    'HIP_OFFSET_X': 0.02134,  # X-offset of the hip from the base
    'LEG_SIDE_OFFSET': 0.055,  # Y-offset of the hip from the base (half the distance between the two hips)
    'HIP_OFFSET_Z': 0.090436,  # Z-offset of the hip from the base
    'THIGH_LENGTH': 0.1168,  # Length of the thigh (hip to knee)
    'CALF_LENGTH': 0.1168,  # Length of the calf (knee to ankle)
    'ANKLE_LENGTH': 0.0355,  # Length of the ankle (ankle to foot)
    'LEG_LENGTH': 0.2691,  # Total leg length (thigh + calf + ankle)
    'BODY_LENGTH': 0.231  # Length of the body (not used in this visualization)
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

# Define points for visualization
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

# Apply transformations
points['R_HIP'] = apply_transform(BASE @ R_HIP, points['R_HIP'])[:3]
points['R_KNEE'] = apply_transform(BASE @ R_HIP @ R_KNEE, points['R_KNEE'])[:3]
points['R_ANKLE'] = apply_transform(BASE @ R_HIP @ R_KNEE @ R_ANKLE, points['R_ANKLE'])[:3]
points['R_FOOT'] = apply_transform(BASE @ R_HIP @ R_KNEE @ R_ANKLE @ R_FOOT, points['R_FOOT'])[:3]

points['L_HIP'] = apply_transform(BASE @ L_HIP, points['L_HIP'])[:3]
points['L_KNEE'] = apply_transform(BASE @ L_HIP @ L_KNEE, points['L_KNEE'])[:3]
points['L_ANKLE'] = apply_transform(BASE @ L_HIP @ L_KNEE @ L_ANKLE, points['L_ANKLE'])[:3]
points['L_FOOT'] = apply_transform(BASE @ L_HIP @ L_KNEE @ L_ANKLE @ L_FOOT, points['L_FOOT'])[:3]

# Plot the kinematic chain
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot right leg
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

# Plot left leg
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
ax.set_box_aspect([1, 1, 1])  # Equal scaling for X, Y, Z axes

# Show the plot
plt.show()