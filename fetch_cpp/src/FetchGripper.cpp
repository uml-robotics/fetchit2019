#include "fetch_cpp/FetchGripper.h"

#include <control_msgs/GripperCommandGoal.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>

const double FetchGripper::wrist_roll_gripper_z_diff = 0.166; // rosrun tf tf_echo wrist_roll_link gripper_link
const double FetchGripper::wrist_roll_gripper_tip_z_diff = 0.03 + wrist_roll_gripper_z_diff;
const double FetchGripper::MAX = 0.11;

FetchGripper::FetchGripper() : client("gripper_controller/gripper_action") {

}

double FetchGripper::getPosition()
{

    auto joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    while (joint_states->name.size() != 2) { // sometimes only return 2 fingers
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    }
    return joint_states->position[0];
}

void FetchGripper::move_to_position(double position) {
    if ( ! client.isServerConnected()) {
        ROS_INFO("[FetchGripper] waitForServer...");
        client.waitForServer();
        ROS_INFO("... done");
    }

    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    client.sendGoal(goal);

    ROS_INFO("[FetchGripper] waitForResult...");
    bool result = client.waitForResult();
    if (result) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO_STREAM("[FetchGripper] " << state.toString() << ": " << state.getText());
    }
    else
        ROS_INFO("[FetchGripper] Action did not finish before the time out.");
}

void FetchGripper::open(double position) {
    ROS_INFO("[FetchGripper] open...");
    move_to_position(position);
}

void FetchGripper::close() {
    ROS_INFO("[FetchGripper] close...");
    move_to_position(0);
}
