#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PoseStampedToPathConverter:
    def __init__(self):
        rospy.init_node('pose_stamped_to_path_converter', anonymous=True)
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        self.pose_stamped_sub = rospy.Subscriber('pose', PoseStamped, self.pose_stamped_callback)
        self.path = Path()

        # Read parameters from ROS parameter server
        self.timer_frequency = rospy.get_param('~timer_frequency', 10)  # Default to 1.0 Hz
        self.epsilon = rospy.get_param('~epsilon', 0.1)  # Default epsilon value

        # Create a timer that calls the publish_timer_callback function at the specified frequency
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.timer_frequency), self.publish_timer_callback)

    def pose_stamped_callback(self, pose_stamped_msg):
        # Check if the new pose is different enough before appending
        if self.is_significant_change(pose_stamped_msg):
            self.path.header = pose_stamped_msg.header
            self.path.poses.append(pose_stamped_msg)

    def is_significant_change(self, new_pose):
        # Compare the new pose to the last pose in the path
        if not self.path.poses:
            return True  # No previous pose, consider it a significant change
        
        last_pose = self.path.poses[-1]
        dx = new_pose.pose.position.x - last_pose.pose.position.x
        dy = new_pose.pose.position.y - last_pose.pose.position.y
        dz = new_pose.pose.position.z - last_pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        return distance >= self.epsilon

    def publish_timer_callback(self, event):
        # Publish the accumulated Path message
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    converter = PoseStampedToPathConverter()
    converter.run()
