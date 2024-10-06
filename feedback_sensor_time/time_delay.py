#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, PointCloud2
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from time import time

t1 = time()

# Define a Fourier series model with four harmonics (sine terms)
def fourier_series(t, A1, B1, C1, A2, B2, C2, A3, B3, C3, A4, B4, C4, D):
    return (A1 * np.sin(B1 * t + C1) + 
            A2 * np.sin(B2 * t + C2) + 
            A3 * np.sin(B3 * t + C3) + 
            A4 * np.sin(B4 * t + C4) + D)

def get_closest_timestamp(camera_timestamps, target_timestamp):
    """Find the closest timestamp from the camera timestamps for a given target timestamp."""
    closest_time = min(camera_timestamps, key=lambda x: abs(x - target_timestamp))
    return closest_time

def process_rosbag(bag_file):
    global t1
    # Open the ROS bag
    bag = rosbag.Bag(bag_file)

    # Lists to store timestamps and time differences
    camera_timestamps = []
    lidar_timestamps = []
    time_differences = []

    # Iterate over the bag file and extract timestamps
    for topic, msg, t in bag.read_messages(topics=['/camera/image_color/compressed', '/livox/lidar']):
        if topic == '/camera/image_color/compressed':
            # Get the timestamp from the camera image
            camera_timestamps.append(msg.header.stamp.to_sec())
        elif topic == '/livox/lidar':
            # Get the timestamp from the LiDAR point cloud
            lidar_timestamps.append(msg.header.stamp.to_sec())

    bag.close()
    t1 = time()
    # Calculate the closest camera timestamp for each LiDAR timestamp and the time difference
    for lidar_time in lidar_timestamps:
        closest_camera_time = get_closest_timestamp(camera_timestamps, lidar_time)
        time_difference = closest_camera_time - lidar_time
        time_differences.append(time_difference)
        print(f"LiDAR Time: {lidar_time}, Closest Camera Time: {closest_camera_time}, Time Difference: {time_difference:.6f} sec")

    # Convert to numpy arrays for fitting
    lidar_timestamps_np = np.array(lidar_timestamps)
    time_differences_np = np.array(time_differences)

    # Initial guess for Fourier series parameters with higher frequency guesses
    initial_guess = [0.001, 5, 0, 
                     0.001, 10, 0, 
                     0.001, 15, 0, 
                     0.001, 20, 0, 
                     np.mean(time_differences_np)]

    # Bounded optimization to allow higher frequencies
    bounds = (
        [0, 
         5, -np.pi, 0, 
         5, -np.pi, 0, 
         5, -np.pi, 0,
         5, -np.pi, -np.inf],  # Lower bounds
        [np.inf, 
         100, np.pi, np.inf, 
         100, np.pi, np.inf, 
         100, np.pi, np.inf, 
         100, np.pi, np.inf]  # Upper bounds
    )

    # Fit the data to the Fourier series model
    params, params_covariance = curve_fit(
        fourier_series, 
        lidar_timestamps_np, 
        time_differences_np, 
        p0=initial_guess,
        bounds=bounds,
        maxfev=30000  # Increase the number of iterations for convergence
    )

    # Generate points for the fitting curve
    fit_timestamps = np.linspace(lidar_timestamps_np.min(), lidar_timestamps_np.max(), 1000)
    fit_differences = fourier_series(fit_timestamps, *params)

    # Calculate residuals
    residuals = time_differences_np - fourier_series(lidar_timestamps_np, *params)

    print(time()-t1)

    # Subplot for both the fitted curve and residuals
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Plotting the scatter plot and the Fourier series fitting curve
    ax1.plot(lidar_timestamps, time_differences, label='Time Difference', color='blue')
    ax1.plot(fit_timestamps, fit_differences, label='Fitting Curve (Fourier Series)', color='red')
    ax1.set_xlabel('LiDAR Timestamps (seconds)')
    ax1.set_ylabel('Time Difference (seconds)')
    ax1.set_title('Scatter Plot with Fourier Series Fitting (4 Harmonics): Time Difference Between LiDAR and Camera')
    ax1.legend()
    ax1.grid(True)

    # Plotting the residuals
    ax2.plot(lidar_timestamps_np, residuals, label='Residuals', color='green')
    ax2.set_xlabel('LiDAR Timestamps (seconds)')
    ax2.set_ylabel('Residuals (seconds)')
    ax2.set_title('Residuals of the Fourier Series Fit')
    ax2.axhline(y=0, color='black', linestyle='--')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

    # Print the Fourier series parameters
    print(f"Fitted Parameters: A1={params[0]:.6f}, B1={params[1]:.6f}, C1={params[2]:.6f}, "
          f"A2={params[3]:.6f}, B2={params[4]:.6f}, C2={params[5]:.6f}, "
          f"A3={params[6]:.6f}, B3={params[7]:.6f}, C3={params[8]:.6f}, "
          f"A4={params[9]:.6f}, B4={params[10]:.6f}, C4={params[11]:.6f}, D={params[12]:.6f}")

    # Check if the residuals are less than 0.001
    if np.all(np.abs(residuals) < 0.001):
        print("Residuals are within the threshold (< 0.001).")
    else:
        print(f"Residuals exceed the threshold. Max residual: {np.max(np.abs(residuals)):.6f}")

if __name__ == '__main__':
    rospy.init_node('rosbag_timestamp_comparator', anonymous=True)

    # Using the specified bag file path and topics
    bag_file_path = '/mnt/SSD/test.bag'
    
    process_rosbag(bag_file_path)
