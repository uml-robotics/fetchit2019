#include "fetch_cpp/PointHeadClient.h"

#include <control_msgs/PointHeadGoal.h>
#include <ros/console.h>

PointHeadClient::PointHeadClient() : client("head_controller/point_head") {
    ROS_INFO("[point_head] waitForServer...");
    client.waitForServer();
}

void PointHeadClient::look_at(double x, double y, double z, std::string frame, double duration) {
    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time::now();
    goal.target.header.frame_id = frame;
    goal.target.point.x = x;
    goal.target.point.y = y;
    goal.target.point.z = z;
    goal.min_duration = ros::Duration(duration);
    client.sendGoal(goal);

    ROS_INFO("[point_head] waitForResult...");
    bool result = client.waitForResult();
    if (result) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO_STREAM("[point_head] " << state.toString() << ": " << state.getText());
    }
    else
        ROS_INFO("[point_head] Action did not finish before the time out.");
}

void PointHeadClient::relatively_look_at(double x, double y, double z) {
    look_at(x, y, z + 0.98, "base_link"); // head is 0.98 above base
}

void PointHeadClient::look_front() {
    relatively_look_at(1, 0, 0);
}
