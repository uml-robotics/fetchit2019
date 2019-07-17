#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//http://docs.fetchrobotics.com/api_overview.html#arm-and-torso
class TorsoLiftClient {

public:
    TorsoLiftClient();
    void lift_to(double height, double max_duration = 2);
    void lift_to_highest();
    void lift_to_lowest() { lift_to(0); }

private:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client;

    const float MAX_HEIGHT = 0.4;

    double get_torso_height();
};