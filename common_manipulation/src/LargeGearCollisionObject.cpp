#include "common_manipulation/LargeGearCollisionObject.h"

LargeGearCollisionObject::LargeGearCollisionObject() {
    construct_collision_object();
}

void LargeGearCollisionObject::attach() {
    ROS_INFO("Add and attach large gear to robot...");
    attached_collision_object.object.operation = moveit_msgs::CollisionObject::ADD;

    planning_scene_interface.applyAttachedCollisionObject(attached_collision_object);
}

void LargeGearCollisionObject::detach_and_remove() {
    ROS_INFO("Detach and remove large gear to the planning scene...");
    attached_collision_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

    planning_scene_interface.applyAttachedCollisionObject(attached_collision_object);
    planning_scene_interface.applyCollisionObject(attached_collision_object.object);
}

void LargeGearCollisionObject::construct_collision_object(const double Z_OFFSET) {

    moveit_msgs::CollisionObject collision_object;

    collision_object.id = ID;

    // as a cylinder
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = HEIGHT; // height
    primitive.dimensions[1] = WIDTH / 2.0; // radius

    // pose
    collision_object.header.frame_id = "gripper_link";
    geometry_msgs::Pose pose;
    pose.position.z = Z_OFFSET;
    pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);

    attached_collision_object.object = collision_object;
    attached_collision_object.link_name = "gripper_link";
    attached_collision_object.touch_links = {"l_gripper_finger_link", "r_gripper_finger_link"};
}
