#include "fetch_cpp/TorsoLiftClient.h"

#include <sensor_msgs/JointState.h>

TorsoLiftClient::TorsoLiftClient() : client("torso_controller/follow_joint_trajectory") {
    ROS_INFO("[torso] Waiting for server...");
    client.waitForServer();
}

double TorsoLiftClient::get_torso_height() {
    ros::NodeHandle nh;
    // get current positions
    auto joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    while (joint_states->name.size() == 2) { // sometimes only return 2 fingers
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    }
    return joint_states->position[2];
}

void TorsoLiftClient::lift_to(double height, double max_duration) {
    if (height > MAX_HEIGHT) {
        ROS_INFO_STREAM("[torso] " << height << " > " << MAX_HEIGHT << "; lifting to MAX_HEIGHT " << MAX_HEIGHT << ")");
        height = MAX_HEIGHT;
    }
    else if (height < 0) {
        ROS_INFO_STREAM("[torso] " << height << " < 0; lifting to 0)");
        height = 0;
    }
    else
        ROS_INFO_STREAM("[torso] lifting to: " << height);

    // calculate duration
    double current_height = get_torso_height();
    ROS_INFO_STREAM("[torso] current height: " << current_height);
    double duration = abs(current_height - height) / MAX_HEIGHT * max_duration;
    ROS_INFO_STREAM("[torso] duration: " << duration);

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(height);
    point.time_from_start = ros::Duration(duration);

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    goal.trajectory.points.push_back(point);

    client.sendGoal(goal);
    ROS_INFO("[torso] waitForResult...");
    bool result = client.waitForResult();
    if (result) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO_STREAM("[torso] " << state.toString() << ": " << state.getText());
    }
    else
        ROS_INFO("[torso] Action did not finish before the time out.");
}

void TorsoLiftClient::lift_to_highest() {
    lift_to(MAX_HEIGHT);
}
