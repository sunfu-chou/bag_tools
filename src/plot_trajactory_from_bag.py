import matplotlib.pyplot as plt
import numpy as np
import rosbag


# Function to extract positions from a pose message
def extract_position(pose_msg):
    return np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])


# Initialize lists to hold the positions
drone_positions = []
wamv_positions = []

# Open the ROS bag file
with rosbag.Bag("/home/argrobotx/robotx-2022/bags/1115_1454/_2023-11-15-14-54-51.bag", "r") as bag:
    # Read messages from the drone topic
    for topic, msg, t in bag.read_messages(topics=["/pozyx_simulation/drone/pose_stamped"]):
        drone_positions.append(extract_position(msg))

    # Read messages from the wamv topic
    for topic, msg, t in bag.read_messages(topics=["/pozyx_simulation/wamv/pose_stamped"]):
        wamv_positions.append(extract_position(msg))

# Convert lists to numpy arrays for easier handling
drone_positions = np.array(drone_positions)
wamv_positions = np.array(wamv_positions)

# Plotting
plt.figure(figsize=[9, 8])
plt.plot(-drone_positions[:, 1], drone_positions[:, 0], label="Drone Trajectory")
plt.plot(-wamv_positions[:, 1], wamv_positions[:, 0], label="WAMV Trajectory")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectories of Drone and WAMV")
plt.legend()
plt.show()
