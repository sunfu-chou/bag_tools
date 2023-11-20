#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

class ModelStateToTf {
public:
    ModelStateToTf() {
        ros::NodeHandle nh("~");
        nh.param("model_name", model_name, std::string("wamv"));
        nh.param("model_tf_suffix", model_tf_suffix, std::string("/base_link"));
        nh.param("broadcast_rate", broadcast_rate, 20);

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
        model_sub = nh.subscribe("/gazebo/model_states", 10, &ModelStateToTf::modelCallback, this);

        tf_msg.header.frame_id = "map";  // or the name of your world frame

        timer = nh.createTimer(ros::Duration(1.0 / broadcast_rate), &ModelStateToTf::broadcastTf, this);
    }

    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        try {
            auto model_index = std::find(msg->name.begin(), msg->name.end(), model_name) - msg->name.begin();
            if (model_index >= msg->name.size()) {
                ROS_WARN_STREAM_THROTTLE(1, "Model name " << model_name << " not found in model states message");
                return;
            }

            ROS_INFO_STREAM_THROTTLE(1, "Model name: " << msg->name[model_index]);

            tf_msg.header.stamp = ros::Time::now();
            tf_msg.child_frame_id = model_name + model_tf_suffix;
            tf_msg.transform.translation.x = msg->pose[model_index].position.x;
            tf_msg.transform.translation.y = msg->pose[model_index].position.y;
            tf_msg.transform.translation.z = msg->pose[model_index].position.z;
            tf_msg.transform.rotation = msg->pose[model_index].orientation;
        }
        catch (const std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1, "Error: " << e.what());
        }
    }

    void broadcastTf(const ros::TimerEvent&) {
        ROS_INFO_STREAM_THROTTLE(
            1, "Broadcasting transform from " << tf_msg.header.frame_id << " to " << tf_msg.child_frame_id
        );
        tf_broadcaster->sendTransform(tf_msg);
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    ros::Subscriber model_sub;
    ros::Timer timer;
    geometry_msgs::TransformStamped tf_msg;
    std::string model_name;
    std::string model_tf_suffix;
    int broadcast_rate;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "model_state_to_tf");
    ModelStateToTf mstf;
    ros::spin();
    return 0;
}
