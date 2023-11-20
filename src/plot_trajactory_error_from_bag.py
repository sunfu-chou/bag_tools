import matplotlib.pyplot as plt
import numpy as np
import rosbag
import tf.transformations  # For quaternion to Euler conversion


def extract_position_orientation(pose_msg):
    position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
    orientation = np.array(
        [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
    )
    return position, orientation


def quaternion_to_euler(quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]  # Yaw angle


# Initialize lists to hold the positions and orientations
drone_positions = []
drone_orientations = []

drone_timestamps = []

with rosbag.Bag("/home/argrobotx/robotx-2022/bags/1115_1454/_2023-11-15-14-54-51.bag", "r") as bag:
    for topic, msg, t in bag.read_messages(topics=["/pozyx_simulation/drone/pose/ground_truth"]):
        position, orientation = extract_position_orientation(msg)
        drone_positions.append(position)
        drone_orientations.append(quaternion_to_euler(orientation))
        drone_timestamps.append(t.to_sec())

# Convert to numpy arrays
drone_positions = np.array(drone_positions)
drone_orientations = np.array(drone_orientations)
drone_timestamps = np.array(drone_timestamps)
drone_positions += np.array([5, 0, -3])
def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi).
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def quaternion_to_matrix(quaternion):
    return tf.transformations.quaternion_matrix(quaternion)

# Calculate errors
from scipy.signal import savgol_filter

position_errors = drone_positions
orientation_errors = normalize_angle(drone_orientations)

position_errors[:, 0] = savgol_filter(position_errors[:, 0], window_length=3, polyorder=2)
position_errors[:, 1] = savgol_filter(position_errors[:, 1], window_length=3, polyorder=2)
position_errors[:, 2] = savgol_filter(position_errors[:, 2], window_length=3, polyorder=2)
orientation_errors = savgol_filter(orientation_errors, window_length=11, polyorder=2)

# Plotting errors
plt.figure(figsize=(12, 8))

# 第一個子圖 - X位置誤差
plt.subplot(4, 1, 1)  # 4行1列的第1個
plt.plot(drone_timestamps - drone_timestamps[0], position_errors[:, 0], label="X Error")
plt.xlim(0, 150)
plt.axvline(x=16, color="red", linestyle="--")
plt.axvline(x=40, color="red", linestyle="--")
plt.axvline(x=65, color="red", linestyle="--")
plt.axvline(x=91, color="red", linestyle="--")
plt.axvline(x=117, color="red", linestyle="--")
plt.title("X Position Error")
plt.xlabel("Time (seconds)")
plt.ylabel("X Error (meters)")
plt.grid(True)

# 第二個子圖 - Y位置誤差
plt.subplot(4, 1, 2)  # 4行1列的第2個
plt.plot(drone_timestamps - drone_timestamps[0], position_errors[:, 1], label="Y Error")
plt.xlim(0, 150)
plt.axvline(x=16, color="red", linestyle="--")
plt.axvline(x=40, color="red", linestyle="--")
plt.axvline(x=65, color="red", linestyle="--")
plt.axvline(x=91, color="red", linestyle="--")
plt.axvline(x=117, color="red", linestyle="--")
plt.title("Y Position Error")
plt.xlabel("Time (seconds)")
plt.ylabel("Y Error (meters)")
plt.grid(True)

# 第三個子圖 - Z位置誤差
plt.subplot(4, 1, 3)  # 4行1列的第3個
plt.plot(drone_timestamps - drone_timestamps[0], position_errors[:, 2], label="Z Error")
plt.xlim(0, 150)
plt.axvline(x=16, color="red", linestyle="--")
plt.axvline(x=40, color="red", linestyle="--")
plt.axvline(x=65, color="red", linestyle="--")
plt.axvline(x=91, color="red", linestyle="--")
plt.axvline(x=117, color="red", linestyle="--")
plt.title("Z Position Error")
plt.xlabel("Time (seconds)")
plt.ylabel("Z Error (meters)")
plt.grid(True)

# 第四個子圖 - 偏航角誤差
plt.subplot(4, 1, 4)  # 4行1列的第4個
plt.plot(drone_timestamps - drone_timestamps[0], orientation_errors, label="Yaw Error")
plt.xlim(0, 150)
plt.axvline(x=16, color="red", linestyle="--")
plt.axvline(x=40, color="red", linestyle="--")
plt.axvline(x=65, color="red", linestyle="--")
plt.axvline(x=91, color="red", linestyle="--")
plt.axvline(x=117, color="red", linestyle="--")
plt.title("Yaw Angle Error")
plt.xlabel("Time (seconds)")
plt.ylabel("Yaw Error (radians)")
plt.grid(True)


plt.tight_layout()
plt.savefig("1115_1454.png")
plt.show()
