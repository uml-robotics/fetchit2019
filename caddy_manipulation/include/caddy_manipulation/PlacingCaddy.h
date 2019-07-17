#pragma once

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class PlacingCaddy {
public:
    PlacingCaddy();

private:
    tf::TransformBroadcaster broadcaster;

    void lower_and_release_caddy();

    void broadcast_frame(geometry_msgs::Pose &p, const std::string &frame_id);

    ros::Publisher cmd_pub;
    ros::Subscriber laser_sub;
};