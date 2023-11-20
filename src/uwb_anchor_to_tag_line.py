#!/usr/bin/env python3
import sys

import fix_python3_path
import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray


class UWBAchorToTagLine:
    def __init__(self):
        rospy.init_node("uwb_anchor_to_tag_line")
        self.tag_frame_prefix = rospy.get_param("~tag_frame_prefix", "drone/uwb/")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_publisher = rospy.Publisher("uwb_line", MarkerArray, queue_size=10)
        self.maker_array = MarkerArray()
        self.anchor_pose = [TransformStamped() for i in range(6)]
        self.tag_pose = TransformStamped()

        self.elapsed_second = 0
        self.red_line_index = 0  # Index of the line to be turned red
        self.tag_index = 0

        self.main_loop_rate = rospy.Rate(20)  # Main loop rate (20Hz)
        self.color_change_timer = rospy.Timer(
            rospy.Duration(0.4), self.change_color_callback
        )  # Color change timer (1Hz)

    def listen_tf(self):
        for i in range(6):
            try:
                self.anchor_pose[i] = self.tf_buffer.lookup_transform("map", f"wamv/uwb/{i}", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        try:
            rospy.loginfo_throttle(1, f"Listening to {self.tag_frame_prefix + str(self.tag_index)}")
            self.tag_pose = self.tf_buffer.lookup_transform(
                "map", self.tag_frame_prefix + str(self.tag_index), rospy.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def publish_marker(self):
        self.maker_array.markers = []
        for i in range(6):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "uwb"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02

            # Change the color of the line to red if the index matches
            if i == self.red_line_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5

            marker.lifetime = rospy.Duration(1.0)
            marker.points.append(
                Point(
                    self.anchor_pose[i].transform.translation.x,
                    self.anchor_pose[i].transform.translation.y,
                    self.anchor_pose[i].transform.translation.z,
                )
            )
            marker.points.append(
                Point(
                    self.tag_pose.transform.translation.x,
                    self.tag_pose.transform.translation.y,
                    self.tag_pose.transform.translation.z,
                )
            )
            self.maker_array.markers.append(marker)
        self.marker_publisher.publish(self.maker_array)

    def change_color_callback(self, event):
        # Change the red line index every 1 second
        self.elapsed_second += 1
        self.red_line_index = (self.elapsed_second + 1) % 6
        self.tag_index = (self.elapsed_second // 7) % 2

    def run(self):
        while not rospy.is_shutdown():
            self.listen_tf()
            self.publish_marker()
            self.main_loop_rate.sleep()


if __name__ == "__main__":
    uwb_anchor_to_tag_line = UWBAchorToTagLine()
    uwb_anchor_to_tag_line.run()
