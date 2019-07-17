#include "schunk_machine/SchunkMachine.hpp"
#include <ros/ros.h>

SchunkMachine::SchunkMachine() : c("/schunk_machine") {
    ROS_INFO_STREAM("[SchunkMachine] waitForServer...");
    this->c.waitForServer();
}

void SchunkMachine::open() {
    ROS_INFO_STREAM("[SchunkMachine::open]");
    fetchit_challenge::SchunkMachineGoal goal;
    goal.state = 0;
    this->c.sendGoal(goal);

    ros::Duration(2).sleep();

    this->c.waitForResult();
    if (this->c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("[SchunkMachine::open] Opened!");
    ROS_INFO_STREAM("[SchunkMachine::open] Current State: " << this->c.getState().toString());

    ros::Duration(3).sleep();
}

void SchunkMachine::close() {
    ROS_INFO_STREAM("[SchunkMachine::close]");
    fetchit_challenge::SchunkMachineGoal goal;
    goal.state = 1;
    this->c.sendGoal(goal);

    ros::Duration(2).sleep();

    this->c.waitForResult();
    if (this->c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("[SchunkMachine::close] Closed!");
    ROS_INFO_STREAM("[SchunkMachine::close] Current State: " << this->c.getState().toString());

    ros::Duration(3).sleep();
}
