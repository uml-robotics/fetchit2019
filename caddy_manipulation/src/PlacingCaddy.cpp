#include <ros/ros.h>
#include <fetch_cpp/FetchGripper.h>
#include <caddy_manipulation/OctomapBuilder.h>
#include "caddy_manipulation/MoveFetch.h"
#include "caddy_manipulation/move_group_util.h"
#include <kit_detector/Caddy.h>
#include "caddy_manipulation/CaddyGrasping.h"
#include "caddy_manipulation/SickCameraClient.h"
#include <fetch_cpp/PointHeadClient.h>
#include <sick_drop_off_detection/SickDropOffDetector.h>

#include "caddy_manipulation/PlacingCaddy.h"

void trigger_sick_camera() {
    SickCamera _;
    _.activate();
}

void move_arm_to_leftmost(MoveFetch& move_fetch) {
//    for (auto _ : move_fetch.getCurrentJointValues())
//        ROS_INFO_STREAM(_);
//    for (auto _ : move_fetch.getJoints())
//        ROS_INFO_STREAM(_);
    auto joint_values = move_fetch.getCurrentJointValues();
    joint_values.at(1) = 1.31; // 1 is shoulder_pan_joint
    move_group_util::set_joints_target_and_execute(move_fetch, move_fetch.getJoints(), joint_values);
}

void move_arm_close(MoveFetch& move_fetch) {
    auto pose = move_fetch.getCurrentPose();
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.4;
    pose.pose.position.z += 0.1;

    move_group_util::set_pose_target_and_execute(move_fetch, pose.pose);
}

//void callback(laser_callback)
//{

//}

PlacingCaddy::PlacingCaddy() {

    ros::NodeHandle nh;

    // start move group

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    auto laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");
    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;
    twist.angular.z = 0.0;

    ros::Rate loop_rate(10);

    while( ros::ok() )//&& laser_scan->ranges[laser_scan->ranges.size() / 2] > 0.1)
    {
        laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");
        double range;
        for(int j = 320; j < 360; j++)
        {
            if(laser_scan->ranges[j] > 0)
                range = laser_scan->ranges[j];
        }

        if(range < 0.22)
            break;
        //ROS_INFO_STREAM("MOVING CLOSER " << range); //662
        cmd_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    twist.linear.x = 0.0;
    for(int count = 0; count < 20; count++)
    {
        laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");

        double left_side = 0, right_side = 0;

        for(int j = 310; j < 320; j++)
        {
            if(laser_scan->ranges[j] > 0)
                left_side = laser_scan->ranges[j];
        }

        for(int j = 370; j > 360; j--)
        {
            if(laser_scan->ranges[j] > 0)
                left_side = laser_scan->ranges[j];
        }

        if(left_side > right_side)
        {
            twist.angular.z = 0.05;
        }
        else if (right_side > left_side)
        {
            twist.angular.z = -0.05;
        }

        cmd_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }


    ROS_INFO_STREAM("FINISHED MOVING " << laser_scan->ranges[laser_scan->ranges.size()/2]);

    //laser_sub = nh.subscribe("/base_scan", 1, laser_callback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("move_group: wait_for_servers...");
    MoveFetch move_fetch;


    auto pose = move_fetch.getCurrentPose().pose;

    ROS_INFO_STREAM("curr pose: " << pose);



    const double move_change = CaddyGrasping::LIFT_OFFSET - 0.1; // 0.1 moving up
    pose.position.z -= move_change;
    ROS_INFO_STREAM("move down pose: " << pose);

    move_group_util::set_pose_target_and_execute(move_fetch, pose);

    // even not moved down, we still do the following...

    // release
    FetchGripper().open();

    // lift its arm
    pose = move_fetch.getCurrentPose().pose;
    ROS_INFO_STREAM("curr pose: " << pose);
    pose.position.z += move_change;
    ROS_INFO_STREAM("move up pose: " << pose);
    move_group_util::set_pose_target_and_execute(move_fetch, pose);

    twist.linear.x = -0.1;

    while( ros::ok() )//&& laser_scan->ranges[laser_scan->ranges.size() / 2] > 0.1)
    {
        laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");
        double range;
        for(int j = 330; j < 350; j++)
        {
            if(laser_scan->ranges[j] > 0)
                range = laser_scan->ranges[j];
        }

        if(range > 0.6)
            break;
        //ROS_INFO_STREAM("MOVING CLOSER " << range); //662
        cmd_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    trigger_sick_camera();
}

void PlacingCaddy::broadcast_frame(geometry_msgs::Pose &p, const std::string &frame_id) {
    tf::Transform _;
    tf::poseMsgToTF(p, _);
    broadcaster.sendTransform(tf::StampedTransform(_, ros::Time::now(), "base_link", frame_id));
}