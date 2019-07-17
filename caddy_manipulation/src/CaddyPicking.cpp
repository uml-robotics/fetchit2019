#include <tf/tf.h>
#include <kit_detector/KitDetector.h>
#include <fetch_cpp/FetchGripper.h>
#include <caddy_manipulation/move_group_util.h>
#include "caddy_manipulation/OctomapBuilder.h"
#include "caddy_manipulation/MoveFetch.h"
#include "caddy_manipulation/CaddyGrasping.h"
#include "caddy_manipulation/CaddyPicking.h"
#include "caddy_manipulation/prepare_manipulation.h"

void convert_to_grasp_pose(geometry_msgs::PoseStamped &handle_center, geometry_msgs::PoseStamped &grasp_pose) {

    tf::Transform handle_center_t;
    tf::poseMsgToTF(handle_center.pose, handle_center_t);

    tf::Transform grasp_t;
    grasp_t.setRotation(handle_center_t.getRotation() * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2) * tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI_2) );
    grasp_t.setOrigin(handle_center_t.getOrigin() + tf::Vector3(0, 0, FetchGripper::wrist_roll_gripper_z_diff));
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(grasp_t, ros::Time::now(), "base_link", "caddy_0_grasp_pose"));

    grasp_pose = handle_center;
    tf::poseTFToMsg(grasp_t, grasp_pose.pose);
}

CaddyPicking::CaddyPicking() {
    for (int i = 0; i < 5; ++i) {
        bool succeeded = pick();
        if (succeeded) {
            return;
        }
    }
}

bool CaddyPicking::pick() {

    ros::NodeHandle nh;

    prepare_caddy_manipulation();

    // detect caddy and get the handle center pose
    KitDetector kd;
    std::vector<KitPoses> kit_poses = kd.detect_from_scene_cloud("/head_camera/depth_downsample/points");
    geometry_msgs::PoseStamped handle_center = kit_poses[0].handle_center;

    // caddy handle grasp pose, the x axis must be pointing downwards, aligning with gripper_link frame

    geometry_msgs::PoseStamped grasp_pose;
    convert_to_grasp_pose(handle_center, grasp_pose);

    // move group

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("move_group: wait_for_servers...");
    MoveFetch move_fetch;

    // pick prepare

    bool plan_only = false;
    CaddyGrasping grasping(move_fetch, handle_center, grasp_pose);

    // build octomap

    OctomapBuilder octomap_builder(nh);
    octomap_builder.clear_map();
    octomap_builder.relatively_look_around_to_build_map({
        Eigen::Vector3d(0.5, 0.5, 0),  // left
        Eigen::Vector3d(0.5, 0, 0), // front
        Eigen::Vector3d(0.5, -0.5, 0), // right
        Eigen::Vector3d(0.5, -0.5, -0.8), // right down
        Eigen::Vector3d(0.5, 0, -0.8), // front down
        Eigen::Vector3d(0.5, 0.5, -0.8)  // left down
    });

    // pick
    bool approach_result = grasping.approach(plan_only, FetchGripper::MAX, 0.1);
    if (approach_result == false) {
        return approach_result;
    }

    // very important! If not cleared, robot will try to shift a little bit
    // because during reaching handle bar, it is not 100% as perceived
    octomap_builder.clear_map();
    grasping.grasp(plan_only);
    grasping.lift_up(plan_only);
    octomap_builder.restore_map();
}
