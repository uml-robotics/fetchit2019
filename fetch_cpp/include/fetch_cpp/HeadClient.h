#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <angles/angles.h>

// http://docs.fetchrobotics.com/api_overview.html#head-interface
class HeadClient {

public:
    constexpr static auto DEFAULT_WHOLE_PAN_DURATION = 4;

    HeadClient();

    // -90 - 90
    void pan_head(int degrees, double whole_pan_duration=DEFAULT_WHOLE_PAN_DURATION);

private:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client;
};