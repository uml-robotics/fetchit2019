#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

class MoveFetch : public moveit::planning_interface::MoveGroupInterface {

public:
    explicit MoveFetch(const std::string& group="arm_with_torso") : MoveGroupInterface(group) {
        lower_speed();

        // Without this, setPlannerParams doesn't work
        setPlannerId("geometric::RRTConnect");

        // Set maximum_waypoint_distance to avoid table edge avoidance
        auto planner_params = std::map<std::string, std::string>{
            {"maximum_waypoint_distance", "0.01"},
            {"optimization_objective", "MaximizeMinClearanceObjective"}  // moved to ompl_planning.yaml
        };
        setPlannerParams(getPlannerId(), group, planner_params);
    }

    void lower_speed(double max_acceleration_scaling_factor=0.5, double max_velocity_scaling_factor=0.5) {
        setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
        setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    }
    // /joint_states topic may only return for [l_gripper_finger_joint, r_gripper_finger_joint]
    //   run multiple times `rostopic echo -n 1 /joint_states`
    geometry_msgs::PoseStamped getCurrentPose(const std::string& end_effector_link = "") {
        auto pose = MoveGroupInterface::getCurrentPose();
        ROS_DEBUG_STREAM("getCurrentPose: " << pose);
        while (pose.pose.position.y < 0.000001 && pose.pose.position.y > -0.000001) {
            ROS_DEBUG_STREAM("getCurrentPose y is btw -0.000001 and 0.000001. Retrying...");
            pose = MoveGroupInterface::getCurrentPose();
            ROS_DEBUG_STREAM("getCurrentPose: " << pose);
        }
        return pose;
    }
};