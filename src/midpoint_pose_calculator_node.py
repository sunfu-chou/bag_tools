#!/usr/bin/env python3
import math

import fix_python3_path
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Point, PoseStamped, Quaternion


class PoseCalculatorNode:
    def __init__(self):
        rospy.init_node("pose_calculator_node")

        self.in1_sub = rospy.Subscriber("/pozyx_simulation/uwb1/pose/optim", PoseStamped, self.in1_callback)
        self.in2_sub = rospy.Subscriber("/pozyx_simulation/uwb0/pose/optim", PoseStamped, self.in2_callback)
        self.out_pub = rospy.Publisher("/pozyx_simulation/drone/pose/optim", PoseStamped, queue_size=10)
        self.in1_pose = None
        self.in2_pose = None

    def in1_callback(self, msg):
        self.in1_pose = msg
        if self.in2_pose:
            self.calculate_and_publish_pose()
            self.in1_pose = None
    
    def in2_callback(self, msg):
        self.in2_pose = msg
        if self.in1_pose:
            self.calculate_and_publish_pose()
            self.in2_pose = None

    def calculate_and_publish_pose(self):
        if hasattr(self, "in1_pose") and hasattr(self, "in2_pose"):
            out_pose = PoseStamped()
            out_pose.header.stamp = rospy.Time.now()
            out_pose.header.frame_id = self.in1_pose.header.frame_id

            # Calculate the position in the middle of in1 and in2
            out_pose.pose.position.x = (self.in1_pose.pose.position.x + self.in2_pose.pose.position.x) / 2.0
            out_pose.pose.position.y = (self.in1_pose.pose.position.y + self.in2_pose.pose.position.y) / 2.0
            out_pose.pose.position.z = (self.in1_pose.pose.position.z + self.in2_pose.pose.position.z) / 2.0

            # Calculate the orientation from in1 to in2
            delta_x = self.in2_pose.pose.position.x - self.in1_pose.pose.position.x
            delta_y = self.in2_pose.pose.position.y - self.in1_pose.pose.position.y

            yaw = math.atan2(delta_y, delta_x)
            quat = tft.quaternion_from_euler(0.0, 0.0, yaw)
            out_pose.pose.orientation = Quaternion(*quat)

            self.out_pub.publish(out_pose)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    pose_calculator = PoseCalculatorNode()
    pose_calculator.run()
