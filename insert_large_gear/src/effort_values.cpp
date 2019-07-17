#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_effort_values");
    ros::NodeHandle nh;

    while (true) {
        auto joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        while (joint_states->name.size() == 2) { // sometimes only return 2 fingers
            joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
        }

        ROS_INFO_STREAM("  name: " << joint_states->name.at(6));
        ROS_INFO_STREAM("effort: " << joint_states->effort.at(6));
    }

    return 0;
}