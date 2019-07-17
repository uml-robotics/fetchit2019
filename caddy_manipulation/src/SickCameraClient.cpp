#include "caddy_manipulation/SickCameraClient.h"
#include <ros/ros.h>

SickCamera::SickCamera() : c("/sick_camera") {
    ROS_INFO_STREAM("[SickCamera] waitForServer...");
    this->c.waitForServer();
}

void SickCamera::activate() {
    ROS_INFO_STREAM("[SickCamera::activate]");
    fetchit_challenge::SickCameraGoal goal;
    // Creates a goal to send to the action server.
    goal.trigger = 0;
    this->c.sendGoal(goal);
    this->c.waitForResult();

    if (this->c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("[SickCamera::activate] Activated!");
    ROS_INFO_STREAM("[SickCamera::activate] Current State: " << this->c.getState().toString());
}
