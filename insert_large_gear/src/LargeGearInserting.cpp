#include "insert_large_gear/LargeGearInserting.h"

#include <chuck_detector/ChuckDetector.h>
#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>
#include <fetch_cpp/HeadClient.h>
#include <caddy_manipulation/MoveFetch.h>
#include <caddy_manipulation/OctomapBuilder.h>
#include <caddy_manipulation/move_group_util.h>
#include <fetch_cpp/FetchGripper.h>
#include "common_manipulation/LargeGearCollisionObject.h"
#include <schunk_machine/SchunkMachine.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

geometry_msgs::PoseStamped LargeGearInserting::convert_to_gripper_pose(geometry_msgs::PoseStamped &chuck_pose) {

    tf::Transform chuck_pose_t;
    tf::poseMsgToTF(chuck_pose.pose, chuck_pose_t);

    tf::Transform grasp_t;
    grasp_t = chuck_pose_t;
    grasp_t.setRotation(grasp_t * tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2 / 3.0) * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2));   /* so not colliding with standing large gear */
    tf::Transform move_to_wrist_roll(tf::Quaternion(0,0,0,1), tf::Vector3(FetchGripper::wrist_roll_gripper_z_diff * -1, 0, 0));
    grasp_t *= move_to_wrist_roll;

    geometry_msgs::PoseStamped p;
    tf::poseTFToMsg(grasp_t, p.pose);

    return p;
}


void LargeGearInserting::broadcast_frame(geometry_msgs::PoseStamped &p, const std::string &frame_id) {
    tf::Transform _;
    tf::poseMsgToTF(p.pose, _);
    broadcaster.sendTransform(tf::StampedTransform(_, ros::Time::now(), "base_link", frame_id));
}

bool LargeGearInserting::detect() {
    TorsoLiftClient torso;
    PointHeadClient point_head;
    HeadClient head;

    torso.lift_to_highest();
    point_head.relatively_look_at(1, 0.5, -0.4);

    ChuckDetector cd;
    geometry_msgs::PoseStamped chuck_pose;
    bool did_it = false;
    for (int i = 0; i < 5; ++i) {
        try {
            chuck_pose = cd.detect_chuck();
            ROS_INFO_STREAM("chuck is detected");
            did_it = true;
            break;
        }
        catch (std::invalid_argument &_) {
        }
    }
    if ( ! did_it) {
        ROS_INFO_STREAM("chuck detection failed.");
        return false;
    }

    ROS_INFO_STREAM("chuck pose: " << chuck_pose);
    auto gripper_pose = convert_to_gripper_pose(chuck_pose);
    ROS_INFO_STREAM("gripper pose: " << gripper_pose);
    broadcast_frame(gripper_pose, "chuck_gripper_pose");

    torso.lift_to_highest();

    // now build octomap
    octomap_builder.clear_map();

    octomap_builder.relatively_look_around_to_build_map({
        Eigen::Vector3d(0.5, 0.5, 0),  // left
        Eigen::Vector3d(0.5, 0, 0), // front
        Eigen::Vector3d(0.5, 0, -0.8), // front down
        Eigen::Vector3d(0.5, 0.5, -0.8)  // left down
    });

    point_head.look_at(chuck_pose);

    const double GRASP_ERROR_Y = 0.015;

    auto start_pose = gripper_pose;
    start_pose.pose.position.y -= 0.17 - GRASP_ERROR_Y;
    ROS_INFO_STREAM("start_pose: " << start_pose);
    broadcast_frame(start_pose, "insert_start_pose");

    auto end_pose = gripper_pose;
    end_pose.pose.position.y -= 0.05 - GRASP_ERROR_Y;
    ROS_INFO_STREAM("end_pose: " << end_pose);
    broadcast_frame(end_pose, "insert_end_pose");

    approach_retreat_pose = start_pose;
    approach_retreat_pose.pose.position.x -= 0.35;
    approach_retreat_pose.pose.position.y += 0.05;
    approach_retreat_pose.pose.position.z += 0.2;
    broadcast_frame(approach_retreat_pose, "approach_retreat_pose");

    insert_start_pose = start_pose;
    insert_end_pose = end_pose;

    return true;
}

LargeGearInserting::LargeGearInserting() : octomap_builder(nh) {

}

bool LargeGearInserting::is_inserted(boost::shared_ptr<sensor_msgs::JointState const> &before_values) {
    auto curr = get_valid_joint_values()->effort.at(6);
    ROS_INFO_STREAM("now shoulder pan effert: " << curr);
    auto before = before_values->effort.at(6);
    ROS_INFO_STREAM("bef shoulder pan effert: " << before);

    if (abs(curr - before) > 15) {
        ROS_INFO_STREAM("> 15!!!!!!");
        return false;
    }

    return true;
}

bool LargeGearInserting::insert() {
    if (! detect()) {
        return false;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("waiting for move group...");
    MoveFetch move_fetch;
    FetchGripper gripper;
    SchunkMachine schunk_machine;

    LargeGearCollisionObject gear_object;
    gear_object.attach();
    move_group_util::set_pose_target_and_execute(move_fetch, approach_retreat_pose.pose);



    // insert
    bool inserted = false;

    std::vector<Eigen::Vector2d> x_z_tries = {
            Eigen::Vector2d{0,0},

            Eigen::Vector2d{0,1},
            Eigen::Vector2d{1,1},
            Eigen::Vector2d{1,0},
            Eigen::Vector2d{1,-1},
            Eigen::Vector2d{1,-1},
            Eigen::Vector2d{0,-1},
            Eigen::Vector2d{-1,-1},
            Eigen::Vector2d{-1,0},
            Eigen::Vector2d{-1,1},

            Eigen::Vector2d{2,2},
            Eigen::Vector2d{2,1},
            Eigen::Vector2d{2,0},
            Eigen::Vector2d{2,-1},
            Eigen::Vector2d{2,-2},
            Eigen::Vector2d{1,-2},
            Eigen::Vector2d{0,-2},
            Eigen::Vector2d{-1,-2},
            Eigen::Vector2d{-2,-2},
            Eigen::Vector2d{-2,-1},
            Eigen::Vector2d{-2,0},
            Eigen::Vector2d{-2,1},
            Eigen::Vector2d{-2,2},
            Eigen::Vector2d{-1,2},
            Eigen::Vector2d{0,2},
            Eigen::Vector2d{1,2},
    };

    // save original one to add on
    geometry_msgs::PoseStamped original_start_pose = insert_start_pose;
    geometry_msgs::PoseStamped original_end_pose = insert_end_pose;

    for(auto curr_try_xz : x_z_tries) {
        // actuall moveit staff
        ROS_INFO_STREAM("TRYING TO INSERT LARGE");
        move_group_util::set_pose_target_and_execute(move_fetch, insert_start_pose.pose);
        auto before = get_valid_joint_values();
        gear_object.detach_and_remove();


//        move_fetch.lower_speed(0.3);
        // 2 middle points
        auto insert_middle_pose = insert_start_pose; insert_middle_pose.pose.position.y += 0.04;
        move_group_util::set_pose_target_and_execute(move_fetch, insert_middle_pose.pose);
        insert_middle_pose.pose.position.y += 0.04;
        move_group_util::set_pose_target_and_execute(move_fetch, insert_middle_pose.pose);

        move_group_util::set_pose_target_and_execute(move_fetch, insert_end_pose.pose);
//        move_fetch.lower_speed();


        // inserted?
        inserted = is_inserted(before);

        if (inserted) {
            schunk_machine.close();
//            std::cout << "Press Enter to open gripper..."; std::cin.ignore();
            gripper.open();
            std::cout << "inserted!!!?";
            break;
        } else {
            ROS_INFO_STREAM("not inserted, adding " << curr_try_xz);
            insert_start_pose.pose.position.x = original_start_pose.pose.position.x + curr_try_xz[0] * 0.01;
            insert_start_pose.pose.position.z = original_start_pose.pose.position.z + curr_try_xz[1] * 0.01;

            ROS_INFO_STREAM("(x=" << insert_start_pose.pose.position.x << ", z=" << insert_start_pose.pose.position.z);
            std::cout << "gonna retry!!!";
        }
    }

    // back to start pose
    move_group_util::set_pose_target_and_execute(move_fetch, insert_start_pose.pose);

    // retreat
    if ( ! inserted) {
        gear_object.attach();
    }
    move_group_util::set_pose_target_and_execute(move_fetch, approach_retreat_pose.pose);

    // go to home position
    std::vector<std::string> joints = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
    std::vector<double> values = {0.36, 1.57, 0, 0.0, -1.7, 0.0, -0.57, 0.0};
    move_group_util::set_joints_target_and_execute(move_fetch, joints, values);

    return inserted;
}

void LargeGearInserting::remove() {
    if (! detect()) {
        return;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("waiting for move group...");
    MoveFetch move_fetch;
    FetchGripper gripper;
    SchunkMachine schunk_machine;

    gripper.open();
    move_group_util::set_pose_target_and_execute(move_fetch, approach_retreat_pose.pose);
    move_group_util::set_pose_target_and_execute(move_fetch, insert_start_pose.pose);
    move_group_util::set_pose_target_and_execute(move_fetch, insert_end_pose.pose);
    gripper.close();
    schunk_machine.open();
    move_group_util::set_pose_target_and_execute(move_fetch, insert_start_pose.pose);
    LargeGearCollisionObject gear_object;
    gear_object.attach();
    move_group_util::set_pose_target_and_execute(move_fetch, approach_retreat_pose.pose);

    std::vector<std::string> joints = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
    std::vector<double> values = {0.36, 1.57, 0, 0.0, -1.7, 0.0, -0.57, 0.0};
    move_group_util::set_joints_target_and_execute(move_fetch, joints, values);
    gear_object.detach_and_remove();
}