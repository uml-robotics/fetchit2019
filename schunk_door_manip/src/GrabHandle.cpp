#include "schunk_door_manip/GrabHandle.h"
#include <schunk_door_detector/HandleDetector.hpp>
#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include <fetch_cpp/HeadClient.h>
#include <caddy_manipulation/MoveFetch.h>
#include <caddy_manipulation/OctomapBuilder.h>
#include <caddy_manipulation/move_group_util.h>
#include <fetch_cpp/FetchGripper.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

geometry_msgs::PoseStamped GrabHandle::convert_to_gripper_pose(tf::Transform &handle_pose) {
    tf::Transform grasp_t;
    grasp_t = handle_pose;
    grasp_t.setRotation(grasp_t * tf::Quaternion(tf::Vector3(0, 0, 1), M_PI_2) * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2));   /* so not colliding with standing large gear */
    tf::Transform move_to_wrist_roll(tf::Quaternion(0,0,0,1), tf::Vector3(FetchGripper::wrist_roll_gripper_z_diff * -1, 0, 0));
    grasp_t *= move_to_wrist_roll;

    geometry_msgs::PoseStamped p;
    tf::poseTFToMsg(grasp_t, p.pose);

    return p;
}


void GrabHandle::broadcast_frame(geometry_msgs::PoseStamped &p, const std::string &frame_id) {
    tf::Transform _;
    tf::poseMsgToTF(p.pose, _);
    broadcaster.sendTransform(tf::StampedTransform(_, ros::Time::now(), "base_link", frame_id));
}

void GrabHandle::open(){
    //Get into position to scan
    TorsoLiftClient torso;
    PointHeadClient point_head;
    HeadClient hc;
    torso.lift_to_highest();
    point_head.look_front();
    hc.pan_head(-10);

    //Detect handle
    HandleDetector hd;
    auto topic = "/head_camera/depth_downsample/points";
    auto handle_pose = hd.detect_from_scene_cloud(topic);
    //Get cloud for the octomap so we can push handle
    no_handle_cloud = hd.get_no_handle_cloud();

    //TODO: Use this to verify that handle is where we expect before and after
    geometry_msgs::PoseStamped h_pose;
    tf::poseTFToMsg(handle_pose, h_pose.pose);
    ROS_INFO_STREAM("handle pose: " << h_pose);

    //TODO: calculate distance to move
    double s_offset, e_offset;
    s_offset = -0.15;
    e_offset = 0.55;

    //TODO: Change values based on where handle is
    if(h_pose.pose.position.y > 0.15){
	ROS_INFO_STREAM("[GripperHandle::open] DOOR LOOKS TO BE OPEN!");
        return;
    }

    //Handle pose -> gripper pose
    auto gripper_pose = convert_to_gripper_pose(handle_pose);
    broadcast_frame(gripper_pose, "handle_gripper_pose");

    //start = gripper pose + offset and up 0.01 to prevent collision
    auto start_pose = gripper_pose;
    start_pose.pose.position.y += s_offset;
    start_pose.pose.position.z += 0.01;
    broadcast_frame(start_pose, "insert_start_pose");

    //end = start + end offset
    auto end_pose = start_pose;
    end_pose.pose.position.y += e_offset;
    broadcast_frame(end_pose, "insert_end_pose");

    //pose to move back (gets out of handle if gripper goes in)
    auto back_pose = end_pose;
    back_pose.pose.position.y += s_offset/2;
    broadcast_frame(back_pose, "insert_back_pose");

    //pose to go up to get away from machine before reset
    auto reset_pose = back_pose;
    reset_pose.pose.position.z += 0.1;
    broadcast_frame(reset_pose, "insert_reset_pose");

    insert_start_pose = start_pose;
    insert_end_pose = end_pose;
    insert_back_pose = back_pose;
    insert_reset_pose = reset_pose;

    //Build octomap used to get to the start position
    octomap_builder.clear_map();

    std::vector<Eigen::Vector3d> poses;
    poses.push_back(Eigen::Vector3d(0.7, 0 , 0.0));
    poses.push_back(Eigen::Vector3d(0.7, -0.5, 0.0 ));
    poses.push_back(Eigen::Vector3d(0.7, 0.5, 0.0 ));
    poses.push_back(Eigen::Vector3d(0.7, 0.5, -0.8 ));
    poses.push_back(Eigen::Vector3d(0.7, -0.5,-0.8 ));
    octomap_builder.relatively_look_around_to_build_map(poses);

    //Look back at the table
    point_head.look_front();

    execute();

    auto new_handle_pose = hd.detect_from_scene_cloud(topic);

    //TODO: Use this to verify that handle is where we expect before and after
    geometry_msgs::PoseStamped new_h_pose;
    tf::poseTFToMsg(new_handle_pose, new_h_pose.pose);
    ROS_INFO_STREAM("new handle pose: " << new_h_pose);

    //TODO: Change values based on where handle is
    if(!h_pose.pose.position.y < 0){
	ROS_INFO_STREAM("[GripperHandle::open] FAILED TO OPEN DOOR!");
        return;
    }
    else{
	ROS_INFO_STREAM("[GripperHandle::open] SUCCESS!");
    }

}

void GrabHandle::close(){
    //Get into position to scan
    TorsoLiftClient torso;
    PointHeadClient point_head;
    HeadClient hc;
    torso.lift_to_highest();
    point_head.look_front();
    hc.pan_head(10);

    //Detect handle
    HandleDetector hd;
    auto topic = "/head_camera/depth_registered/points";
    auto handle_pose = hd.detect_from_scene_cloud(topic);
    //Get cloud for the octomap so we can push handle
    no_handle_cloud = hd.get_no_handle_cloud();

    //TODO: Use this to verify that handle is where we expect before and after
    geometry_msgs::PoseStamped h_pose;
    tf::poseTFToMsg(handle_pose, h_pose.pose);
    ROS_INFO_STREAM("handle pose: " << h_pose);

    //TODO: calculate distance to move
    double s_offset, e_offset;
    s_offset = 0.1;
    e_offset = -0.5;

    //TODO: Change values based on where handle is
    if(h_pose.pose.position.y < 0){
	ROS_INFO_STREAM("[GripperHandle::close] DOOR LOOKS TO BE CLOSED!");
        return;
    }

    //Handle pose -> gripper pose
    auto gripper_pose = convert_to_gripper_pose(handle_pose);
    broadcast_frame(gripper_pose, "handle_gripper_pose");

    //start = gripper pose + offset and up 0.01 to prevent collision
    auto start_pose = gripper_pose;
    start_pose.pose.position.y += s_offset;
    start_pose.pose.position.z += 0.01;
    broadcast_frame(start_pose, "insert_start_pose");

    //end = start + end offset
    auto end_pose = start_pose;
    end_pose.pose.position.y += e_offset;
    broadcast_frame(end_pose, "insert_end_pose");

    //pose to move back (gets out of handle if gripper goes in)
    auto back_pose = end_pose;
    back_pose.pose.position.y += s_offset/2;
    broadcast_frame(back_pose, "insert_back_pose");

    //pose to go up to get away from machine before reset
    auto reset_pose = back_pose;
    reset_pose.pose.position.z += 0.1;
    broadcast_frame(reset_pose, "insert_reset_pose");

    insert_start_pose = start_pose;
    insert_end_pose = end_pose;
    insert_back_pose = back_pose;
    insert_reset_pose = reset_pose;

    //Build octomap used to get to the start position
    octomap_builder.clear_map();

    std::vector<Eigen::Vector3d> poses;
    poses.push_back(Eigen::Vector3d(0.7, 0 , 0.0));
    poses.push_back(Eigen::Vector3d(0.7, -0.5, 0.0 ));
    poses.push_back(Eigen::Vector3d(0.7, 0.5, 0.0 ));
    poses.push_back(Eigen::Vector3d(0.7, 0.5, -0.8 ));
    poses.push_back(Eigen::Vector3d(0.7, -0.5,-0.8 ));
    octomap_builder.relatively_look_around_to_build_map(poses);

    //Look back at the table
    point_head.look_front();

    execute();

    auto new_handle_pose = hd.detect_from_scene_cloud(topic);

    //TODO: Use this to verify that handle is where we expect before and after
    geometry_msgs::PoseStamped new_h_pose;
    tf::poseTFToMsg(new_handle_pose, new_h_pose.pose);
    ROS_INFO_STREAM("new handle pose: " << new_h_pose);

    //TODO: Change values based on where handle is
    if(!h_pose.pose.position.y > 0.15){
	ROS_INFO_STREAM("[GripperHandle::close] FAILED TO CLOSE DOOR!");
        return;
    }
    else{
	ROS_INFO_STREAM("[GripperHandle::close] SUCCESS!");
    }

}

void GrabHandle::execute() {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("[GrabHandle::execute] Waiting for move group...");
    MoveFetch move_fetch;

    //Go to the start pose using a full octomap and no constraints on movement
    if(!move_group_util::set_pose_target_and_execute(move_fetch, insert_start_pose.pose)){
        ROS_INFO_STREAM("[GrabHandle::execute] Could not find start pose");
        //TODO: Handle errors
    }

    //Add constraints to the gripper so that it only moves horizontally
    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "gripper_link";
    pcm.header.frame_id = "base_link";
    pcm.target_point_offset.x = 0.01;
    pcm.target_point_offset.y = 0.01;
    pcm.target_point_offset.z = 0.01;
    moveit_msgs::Constraints test_constraints;
    test_constraints.position_constraints.push_back(pcm);
    move_fetch.setPathConstraints(test_constraints);

    //Remove the handle and machine top from octomap so we can push the handle
    octomap_builder.clear_map();
    octomap_builder.publish(no_handle_cloud);

    move_fetch.lower_speed(0.2, 0.2);

    //Move to the goal position (open/close the door)
    move_group_util::set_pose_target_and_execute(move_fetch, insert_end_pose.pose);
    move_fetch.lower_speed();
    //Gripper used to get stuck in the handle so move back for safety
    move_group_util::set_pose_target_and_execute(move_fetch, insert_back_pose.pose);
    //Remove the constraints becuase arm needs to reset
    move_fetch.clearPathConstraints();
    //Move up to get away from machine
    move_group_util::set_pose_target_and_execute(move_fetch, insert_reset_pose.pose);
    //TODO: use the call in main... reset the arm
    prepare_to_pick();
}

//TODO: Should use the one in main
void GrabHandle::prepare_to_pick() {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MoveFetch move_fetch;
    ROS_DEBUG_STREAM("[GrabHandle::prepare_to_pick] Model frame" << move_fetch.getRobotModel()->getModelFrame());

    // https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo/scripts/prepare_simulated_robot.py#L41
    std::vector<std::string> joints = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
    std::vector<double> values = {0.36, 1.57, 0, 0.0, -1.7, 0.0, -0.57, 0.0};

    move_group_util::set_joints_target_and_execute(move_fetch, joints, values);
}
