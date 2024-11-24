import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

encoder_data = pd.read_csv('EncoderData.csv')

encoder_x = encoder_data['x'].values   # Encoder x
encoder_y = encoder_data['y'].values   # Encoder y

# LIDAR starting point
lidar_start_x = 1.2890949777049199
lidar_start_y = -0.4733531484089912

# Encoder starting point 
encoder_start_x = 0
encoder_start_y = 0

# Calculate the translation vector
translation_x = lidar_start_x - encoder_start_x
translation_y = lidar_start_y - encoder_start_y

# Rotation angle in degrees
theta_deg = -70
theta_rad = np.deg2rad(theta_deg)  

# Rotation matrix
cos_theta = np.cos(theta_rad)
sin_theta = np.sin(theta_rad)

# Apply 70 degree rotation
rotated_encoder_x = encoder_x * cos_theta - encoder_y * sin_theta  # x' = x*cos(θ) - y*sin(θ)
rotated_encoder_y = encoder_x * sin_theta + encoder_y * cos_theta  # y' = x*sin(θ) + y*cos(θ)

# Shift the rotated encoder data to align with LIDAR starting point
shifted_encoder_x = rotated_encoder_x + translation_x
shifted_encoder_y = rotated_encoder_y + translation_y


transformed_ground_truth = np.array([
   [1.283, -0.475],  # P1
    [1.355, -1.3],   # P2
    [0, -1.22],       # P3
    [0, -0.575],      # P4
    [0.595, -0.41]   # P5›
])

lidar_data = pd.read_csv('lidar_positions.csv')

lidar_x = lidar_data['X'].values   # LIDAR x
lidar_y = lidar_data['Y'].values   # LIDAR y

plt.figure(figsize=(8, 8))

plt.scatter(shifted_encoder_x, shifted_encoder_y, label='Encoder', color='red', s=30, marker='^')

plt.scatter(lidar_x, lidar_y, label='LIDAR', color='blue', s=10, marker='o')

plt.scatter(transformed_ground_truth[:, 0], transformed_ground_truth[:, 1], label='Ground Truth', color='green', s=30, marker='s')

plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Comparison of Encoder, LIDAR, and Ground-Truth Trajectories')

plt.legend()

plt.grid(True)
plt.axis('equal')
plt.show()
