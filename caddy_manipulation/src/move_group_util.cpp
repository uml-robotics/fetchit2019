#include <moveit_msgs/RobotTrajectory.h>
#include "caddy_manipulation/move_group_util.h"

namespace move_group_util {

    bool ask_() {
//        return true;
    }

    void follow_traj(
            moveit::planning_interface::MoveGroupInterface &move_group,
            const std::vector<geometry_msgs::Pose> &waypoints,
            bool plan_only) {
        double step = 0.01;
        double disable_jump = 0.0;
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, step, disable_jump, trajectory, true);
        ROS_INFO("follow_trajectory computeCartesianPath: %.2f%% achieved", fraction * 100.0);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto success = move_group.plan(plan);
        ROS_INFO_STREAM("follow_trajectory plan " << success);

	
        if (ask_()) {
            std::cout << "Press Enter to Continue executing the plan";
            std::cin.ignore();
        }

        auto result = move_group.execute(plan);
        ROS_INFO_STREAM("follow_trajectory result:" << result);
    }

    moveit::planning_interface::MoveItErrorCode
    _plan_until_succeed(
            moveit::planning_interface::MoveGroupInterface &move_group,
            moveit::planning_interface::MoveGroupInterface::Plan &plan,
            const int MAX_RETRY) {

        int n_times = 0;

        auto result = move_group.plan(plan);
        ROS_INFO_STREAM("plan result: " << result);
        while (result !=  moveit_msgs::MoveItErrorCodes::SUCCESS && n_times < MAX_RETRY) {
            ROS_INFO_STREAM("retry " << n_times << "...");
            result = move_group.plan(plan);
            ROS_INFO_STREAM("plan result: " << result);
            n_times += 1;
        }

        return result;
    }

    moveit::planning_interface::MoveItErrorCode
    _execute_until_succeed(
            moveit::planning_interface::MoveGroupInterface &move_group,
            moveit::planning_interface::MoveGroupInterface::Plan &plan,
            const int MAX_RETRY) {

        int n_times = 0;

        auto result = move_group.execute(plan);
        ROS_INFO_STREAM("execute result: " << result);
        while (result !=  moveit_msgs::MoveItErrorCodes::SUCCESS && n_times < MAX_RETRY) {
            if(result == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
            {
                ROS_INFO_STREAM("Timed out Execution, probably nothing..." );
                result = moveit_msgs::MoveItErrorCodes::SUCCESS;
                break;
            }
            ROS_INFO_STREAM("retry " << n_times << "...");
            auto plan_result = _plan_until_succeed(move_group, plan, MAX_RETRY);
            if (plan_result == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                result = move_group.execute(plan);
                ROS_INFO_STREAM("execute result: " << result);
            }

            n_times += 1;
        }

        return result;
    }

    bool plan_and_execute_with_retry(
            moveit::planning_interface::MoveGroupInterface &move_group,
            const int MAX_RETRY,
            bool plan_only) {

        bool result = true;

        // plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = _plan_until_succeed(move_group, plan, MAX_RETRY);

        // execute
        if ( ! plan_only) {
            if (plan_result !=  moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_INFO_STREAM("skip executing - plan failed...");
                return false;
            }
            else {
                if (ask_()) {
                    std::cout << "Press Enter to Continue executing the plan";
                    std::cin.ignore();
                }

                _execute_until_succeed(move_group, plan, MAX_RETRY);
            }
        }
        else {
            ROS_INFO_STREAM("skip executing - plan only...");
        }

        return result;
    }

    bool set_pose_target_and_execute(
            moveit::planning_interface::MoveGroupInterface &move_group,
            const geometry_msgs::Pose& target,
            const int MAX_RETRY,
            bool plan_only) {

        move_group.setPoseTarget(target);

        return plan_and_execute_with_retry(move_group, MAX_RETRY, plan_only);
    }

    bool set_joints_target_and_execute(
            moveit::planning_interface::MoveGroupInterface &move_group,
            std::vector<std::string> joints,
            std::vector<double> values,
            const int MAX_RETRY,
            bool plan_only) {

        std::map<std::string, double> variable_values;
        for (int i = 0; i < joints.size(); ++i) {
            variable_values[*(joints.begin()+i)] = *(values.begin()+i);
        }
        move_group.setJointValueTarget(variable_values);

        return plan_and_execute_with_retry(move_group, MAX_RETRY, plan_only);
    }

}
