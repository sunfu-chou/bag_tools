#! /usr/bin/env python3

import fix_python3_path
import rospy
from gazebo_msgs.msg import ModelStates


class GetModelState:
    def __init__(self):
        rospy.init_node("model_state_to_tf")
        self.model_name = rospy.get_param("~model_name", "wamv")
        self.publish_rate = rospy.get_param("~publish_rate", 20)

        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.pose

        self.tf_msg = TransformStamped()

    def model_callback(self, msg):
        # Get the transform from the world to the robot
        model_index = msg.name.index(self.model_name)
        rospy.loginfo_throttle(1, f"Model name: {msg.name[model_index]}")

        # Create the transform from the world to the model
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.frame_id = "map"
        self.tf_msg.child_frame_id = self.model_name + self.model_tf_suffix
        self.tf_msg.transform.translation = msg.pose[model_index].position
        self.tf_msg.transform.rotation = msg.pose[model_index].orientation

    def broadcast_tf(self, event):
        rospy.loginfo_throttle(
            1, f"Broadcasting transform from {self.tf_msg.header.frame_id} to {self.tf_msg.child_frame_id}"
        )
        self.tf_broadcaster.sendTransform(self.tf_msg)


if __name__ == "__main__":
    ModelStateToTf()
    rospy.spin()
