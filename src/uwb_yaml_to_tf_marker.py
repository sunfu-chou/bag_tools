#!/usr/bin/env python3
import sys

import fix_python3_path
import rospy
import tf2_ros
import yaml
from visualization_msgs.msg import Marker, MarkerArray


class AnchorLoader:
    @staticmethod
    def load_yaml(yaml_file):
        with open(yaml_file, "r") as file:
            try:
                data = yaml.safe_load(file)
                return data
            except yaml.YAMLError as e:
                print(f"Error loading YAML file: {e}")
                return None


# This class is used to read the uwb pose from yaml and braocast it as tf and publish it as marker
class UWBYamlToTFMarker:
    def __init__(self):
        rospy.init_node("uwb_yaml_to_tf_marker")
        config_path = rospy.get_param(
            "~config_path", "/home/argrobotx/robotx-2022/catkin_ws/src/pozyx_ros/config/wamv/wamv.yaml"
        )
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "wamv/base_link")
        self.robot_uwb_origin_frame = rospy.get_param("~robot_uwb_origin_frame", "wamv/uwb/origin")
        self.robot_uwb_frame_prefix = rospy.get_param("~robot_uwb_frame", "wamv/uwb").rstrip("/")
        self.uwb_origin_shift_x = rospy.get_param("~uwb_origin_shift_x", 0.0)
        self.uwb_origin_shift_y = rospy.get_param("~uwb_origin_shift_y", 0.0)
        self.uwb_origin_shift_z = rospy.get_param("~uwb_origin_shift_z", 0.0)
        self.marker_color = rospy.get_param("~marker_color", [0.0, 0.0, 1.0, 1.0])
        self.anchors = AnchorLoader.load_yaml(config_path)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.marker_publisher = rospy.Publisher("anchors", MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_tf()
            self.publish_marker()
            self.rate.sleep()

    def publish_tf(self):
        # Pre-define a origin frame to UWBs
        rospy.loginfo_throttle(
            1, f"Broadcasting transform from {self.robot_base_frame} to {self.robot_uwb_origin_frame}"
        )
        t = tf2_ros.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = self.robot_uwb_origin_frame
        t.transform.translation.x = self.uwb_origin_shift_x
        t.transform.translation.y = self.uwb_origin_shift_y
        t.transform.translation.z = self.uwb_origin_shift_z
        t.transform.rotation.w = 1
        self.tf_broadcaster.sendTransform(t)

        rospy.loginfo_throttle(
            1, f"Broadcasting transform from {self.robot_uwb_origin_frame} to {self.robot_uwb_frame_prefix}"
        )
        for i, anchor in enumerate(self.anchors):
            t = tf2_ros.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.robot_uwb_origin_frame
            t.child_frame_id = f"{self.robot_uwb_frame_prefix}/{i}"
            t.transform.translation.x = self.anchors[anchor]["x"] / 1000.0
            t.transform.translation.y = self.anchors[anchor]["y"] / 1000.0
            t.transform.translation.z = self.anchors[anchor]["z"] / 1000.0
            t.transform.rotation.w = 1
            self.tf_broadcaster.sendTransform(t)

    def publish_marker(self):
        self.marker_array.markers = []
        for anchor in self.anchors:
            marker = Marker()
            marker.header.frame_id = self.robot_uwb_origin_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "anchors"
            marker.id = int(anchor[-1])
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.anchors[anchor]["x"] / 1000.0
            marker.pose.position.y = self.anchors[anchor]["y"] / 1000.0
            marker.pose.position.z = self.anchors[anchor]["z"] / 1000.0
            marker.pose.orientation.w = 1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = self.marker_color[0]
            marker.color.g = self.marker_color[1]
            marker.color.b = self.marker_color[2]
            marker.color.a = self.marker_color[3]
            self.marker_array.markers.append(marker)
        self.marker_publisher.publish(self.marker_array)


if __name__ == "__main__":
    uwb_yaml_to_tf_marker = UWBYamlToTFMarker()
    uwb_yaml_to_tf_marker.run()
