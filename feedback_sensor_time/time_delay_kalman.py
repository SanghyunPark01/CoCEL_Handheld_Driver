#!/usr/bin/env python

import rospy
import rosbag
from sensor_msgs.msg import CompressedImage, PointCloud2
import numpy as np
import matplotlib.pyplot as plt
from time import time 

# Kalman Filter class
class KalmanFilter:
    def __init__(self, dt, process_var, measurement_var, initial_state):
        self.dt = dt  # time step

        # State transition matrix (2x2)
        self.A = np.array([[1, dt],
                           [0, 1]])

        # Observation matrix (1x2)
        self.H = np.array([[1, 0]])

        # Initial state
        self.x = initial_state

        # Covariance matrix
        self.P = np.eye(2)

        # Process noise covariance (2x2)
        self.Q = process_var * np.array([[dt**4 / 4, dt**3 / 2],
                                         [dt**3 / 2, dt**2]])

        # Measurement noise covariance (1x1)
        self.R = np.array([[measurement_var]])

    def predict(self):
        # Prediction step
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state estimate with measurement
        y = z - self.H @ self.x  # innovation
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        return self.x
    
    def predict_next_step(self):
        return self.A @ self.x


def process_rosbag_with_kalman_filter(bag_file, dt=0.1):
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
    
    # Calculate the closest camera timestamp for each LiDAR timestamp and the time difference
    for lidar_time in lidar_timestamps:
        closest_camera_time = min(camera_timestamps, key=lambda x: abs(x - lidar_time))
        time_difference = closest_camera_time - lidar_time
        time_differences.append(time_difference)
        print(f"LiDAR Time: {lidar_time}, Closest Camera Time: {closest_camera_time}, Time Difference: {time_difference:.6f} sec")

    # Initialize Kalman filter
    process_var = 1e-3  # Process noise variance
    measurement_var = 1e-2  # Measurement noise variance
    initial_state = np.array([time_differences[0], 0])  # initial time difference and rate of change

    kf = KalmanFilter(dt, process_var, measurement_var, initial_state)

    predictions = []
    residuals = []

    # Run the Kalman filter on the time difference data
    # predicted_next = 0
    for z in time_differences:
        # residuals.append(z - predicted_next)
        t1 = time()
        kf.predict()  # Prediction step

        predicted_time_diff = kf.get_state()[0]
        residuals.append(z - predicted_time_diff)  # Calculate and store the residual
        predictions.append(predicted_time_diff)  # Save the predicted time difference

        kf.update(z)  # Update step with the observation
        # predicted_time_diff = kf.get_state()[0]

        # predicted_next = kf.predict_next_step()[0]

        # residuals.append(z - predicted_time_diff)  # Calculate and store the residual
        print("Kalman filter processing time : {}".format(time()-t1))

    # Predict the next time difference based on the Kalman Filter
    next_time_diff = kf.get_state()[0] + kf.get_state()[1] * dt  # Predicted next time difference
    return time_differences, predictions, residuals, next_time_diff, lidar_timestamps


def plot_results(time_differences, predictions, residuals, next_time_diff, lidar_timestamps):
    # Subplot for observed, predicted, and residuals
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Plotting observed and predicted time differences
    ax1.plot(lidar_timestamps, time_differences, label='Observed Time Differences', color='blue')
    ax1.plot(lidar_timestamps, predictions, label='Kalman Filter Predictions', color='orange')
    ax1.axhline(y=next_time_diff, color='red', linestyle='--', label=f'Predicted Next Time Diff: {next_time_diff:.3f}')
    ax1.set_xlabel('LiDAR Timestamps (seconds)')
    ax1.set_ylabel('Time Difference (seconds)')
    ax1.set_title('Kalman Filter Time Difference Prediction')
    ax1.legend()
    ax1.grid(True)

    # Plotting residuals
    ax2.plot(lidar_timestamps, residuals, label='Residuals', color='green')
    ax2.axhline(y=0, color='black', linestyle='--')
    ax2.set_xlabel('LiDAR Timestamps (seconds)')
    ax2.set_ylabel('Residuals (seconds)')
    ax2.set_title('Kalman Filter Residuals')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    rospy.init_node('rosbag_kalman_filter', anonymous=True)

    # Bag file path
    bag_file_path = '/mnt/SSD/test.bag'
    
    # Process ROS bag and apply Kalman Filter
    time_differences, predictions, residuals, next_time_diff, lidar_timestamps = process_rosbag_with_kalman_filter(bag_file_path)

    # Plot the results including residuals
    plot_results(time_differences, predictions, residuals, next_time_diff, lidar_timestamps)

    print(f'Predicted next time difference: {next_time_diff:.6f} seconds')
