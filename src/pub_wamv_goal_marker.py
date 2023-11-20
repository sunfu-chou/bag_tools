#! /usr/bin/env python3

import fix_python3_path
import rospy
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker


def main():
    def publish_goal(event):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "wamv/base_link"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = -5
        goal_pose.pose.position.y = 0
        goal_pose.pose.position.z = 3.0
        goal_pose.pose.orientation.w = 1.0
        pub_goal.publish(goal_pose)

    rospy.init_node("pub_wamv_goal_marker")
    pub = rospy.Publisher("/wamv/goal_marker", Marker, queue_size=1)
    pub_goal = rospy.Publisher("/wamv/goal", PoseStamped, queue_size=1)
    pub_goal_timer = rospy.Timer(rospy.Duration(0.05), publish_goal)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wamv_goal"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = [
            Point(x=-750, y=420, z=0.0),
            Point(x=-730, y=420, z=0.0),
            Point(x=-730, y=400, z=0.0),
            Point(x=-750, y=400, z=0.0),
            Point(x=-750, y=420, z=0.0),
        ]
        pub.publish(marker)

        rate.sleep()


if __name__ == "__main__":
    main()
