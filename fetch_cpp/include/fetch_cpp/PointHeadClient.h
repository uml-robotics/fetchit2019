#pragma once

#include <string>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PoseStamped.h>

// Fetch doc: http://docs.fetchrobotics.com/api_overview.html#head-interface
class PointHeadClient {

public:
    PointHeadClient();

    void look_at(double x, double y, double z, std::string frame, double duration=1.0);
    void look_at(geometry_msgs::PoseStamped &p) {
        look_at(p.pose.position.x, p.pose.position.y, p.pose.position.z, p.header.frame_id);
    }

    void look_front();

    // relative to head_tilt_link
    void relatively_look_at(double x, double y, double z);

private:
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> client;
};