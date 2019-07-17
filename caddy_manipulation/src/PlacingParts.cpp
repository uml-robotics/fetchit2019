#include "caddy_manipulation/PlacingParts.h"

#include <ros/ros.h>
#include <kit_detector/KitDetector.h>
#include <fetch_cpp/FetchGripper.h>
#include <kit_detector/Caddy.h>
#include "caddy_manipulation/Placing.h"
#include "caddy_manipulation/MoveFetch.h"
#include "caddy_manipulation/OctomapBuilder.h"
#include "caddy_manipulation/prepare_manipulation.h"
#include <boost/algorithm/string/predicate.hpp>
#include <common_manipulation/LargeGearCollisionObject.h>

class PlacingLargeGear : public Placing {
public:
    PlacingLargeGear(MoveGroupInterface &move_group, OctomapBuilder& octomap_builder, geometry_msgs::PoseStamped& place_pose, double approach_z_diff)
    : Placing(move_group, octomap_builder, place_pose, approach_z_diff) {

    }

    void place() override {
        o.attach();
        Placing::place();
    }

    void release(bool plan_only = false) override {
        Placing::release(plan_only);
        o.detach_and_remove();
    }

private:
    LargeGearCollisionObject o;
};

void convert_to_place_pose(geometry_msgs::PoseStamped &compartment_pose, const std::string &compartment_name, geometry_msgs::PoseStamped &place_pose) {
    tf::Transform t;
    tf::poseMsgToTF(compartment_pose.pose, t);

    tf::Transform place_t;
    place_t.setRotation(t.getRotation() * tf::Quaternion(tf::Vector3(0, 1, 0), M_PI_2) * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2));
    place_t.setOrigin(t.getOrigin() + tf::Vector3(0, 0, CaddyModel::top_z + FetchGripper::wrist_roll_gripper_tip_z_diff));
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(tf::StampedTransform(place_t, ros::Time::now(), "base_link", compartment_name + "_place_pose"));

    place_pose = compartment_pose;
    tf::poseTFToMsg(place_t, place_pose.pose);
}

void PlacingParts::place_bolt() {
    place("bolt");
}

void PlacingParts::place_large_gear() {
    place("large_gear");
}

void PlacingParts::place_small_gear() {
    place("small_gear");
}

void PlacingParts::place_gearbox() {
    place("gearbox");
}

void PlacingParts::place(const std::string &what) {

    ros::NodeHandle nh;

    prepare_caddy_manipulation();

    // detect caddy
    KitDetector kd;
    std::vector<KitPoses> kits_poses = kd.detect_from_scene_cloud("/head_camera/depth_downsample/points");

    KitPoses kit_poses = kits_poses[0];

    geometry_msgs::PoseStamped bolt_place_pose;
    convert_to_place_pose(kit_poses.bolt_compartment, "bolt", bolt_place_pose);

    geometry_msgs::PoseStamped gearbox_place_pose;
    convert_to_place_pose(kit_poses.gearbox_compartment, "gearbox", gearbox_place_pose);

    geometry_msgs::PoseStamped gears_place_pose;
    convert_to_place_pose(kit_poses.gears_compartment, "gears", gears_place_pose);


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

    // move group

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("move_group: wait_for_servers...");
    MoveFetch move_fetch;

    geometry_msgs::PoseStamped place_pose;
    if (what == "bolt") {
        place_pose = bolt_place_pose;
    }
    else if (boost::algorithm::ends_with(what, "gear")) {
        place_pose = gears_place_pose;
    }
    else if (what == "gearbox") {
        place_pose = gearbox_place_pose;
    }
    else {
        ROS_ERROR_STREAM("cannot place " << what);
    }
    place_pose.pose.position.z += 0.03;

    if (what == "large_gear") {
        PlacingLargeGear placing(move_fetch, octomap_builder, place_pose, 0.1);
        placing.place();
    }
    else {
        Placing placing(move_fetch, octomap_builder, place_pose, 0.1);
        placing.place();
    }
}
