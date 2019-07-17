#include "fetch_cpp/HeadClient.h"

#include <sensor_msgs/JointState.h>

HeadClient::HeadClient() : client("head_controller/follow_joint_trajectory") {
    ROS_INFO("[head] Waiting for server...");
    client.waitForServer();
}

void HeadClient::pan_head(int degrees, double whole_pan_duration) {
    ROS_INFO_STREAM("[pan_head] input: " << degrees << " degrees");
    // validate input & convert to radians
    if (degrees < -90)
        degrees = -90;
    else if (degrees > 90)
        degrees = 90;
    double radian = angles::from_degrees(degrees);

    // get current positions
    auto joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    while (joint_states->name.size() == 2) { // sometimes only return 2 fingers
        joint_states = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    }
    auto curr_pan_position = joint_states->position.at(4);
    auto curr_tilt_position = joint_states->position.at(5);

    // calculate time_from_start
    double duration = std::abs(radian - curr_pan_position) / M_PI * whole_pan_duration;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"head_pan_joint", "head_tilt_joint"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {radian, curr_tilt_position};
    point.time_from_start = ros::Duration(duration);
    goal.trajectory.points = {point};

    client.sendGoal(goal);
    ROS_INFO("[head] waitForResult...");
    bool result = client.waitForResult();
    if (result) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO_STREAM("[head] " << state.toString() << ": " << state.getText());
    }
    else
        ROS_INFO("[torso] Action did not finish before the time out.");
}