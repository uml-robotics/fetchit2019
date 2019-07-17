#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

// http://docs.fetchrobotics.com/api_overview.html#gripper-interface
class FetchGripper {

public:
    const static double wrist_roll_gripper_z_diff;
    const static double wrist_roll_gripper_tip_z_diff;
    const static double MAX;

    FetchGripper();
    void open(double position = MAX);
    void close();
    double getPosition();

private:
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client;
    void move_to_position(double position);
};