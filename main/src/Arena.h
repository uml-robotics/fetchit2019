#pragma once

#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <caddy_manipulation/MoveFetch.h>
#include <caddy_manipulation/move_group_util.h>

#include "MapPose.h"
#include <kit_detector/CaddyRotationParam.h>

struct ArenaSpec {

    const double DISTANCE = 0.1; // default distance away to table

    const double ARENA_LENGTH = 3.05;
    const double ARENA_LENGTH_2 = ARENA_LENGTH / 2.0;
    const double ARENA_EMPTY_SPACE_CENTER_X = ARENA_LENGTH / 2.0;
    const double ARENA_EMPTY_SPACE_CENTER_Y = (ARENA_LENGTH - SCHUNK_TABLE_WIDTH - TABLE_WIDTH) / 2.0;


    const double TABLE_LENGTH = 0.94;
    const double TABLE_LENGTH_2 = TABLE_LENGTH / 2.0;
    const double TABLE_WIDTH = TABLE_LENGTH_2 + 0.02;

    const double SCHUNK_TABLE_LENGTH = 1.1;
    const double SCHUNK_TABLE_WIDTH = 0.7;

    const double DROPOFF_TABLE_DISTANCE_TO_WALL = 0.145;
    const double KIT_STATION_DISTANCE_TO_WALL = 1.46;
    const double GEARBOX_TABLE_DISTANCE_TO_WALL = 1.03;
    const double SCHUNK_TABLE_DISTANCE_TO_DROPOFF_TABLE = 0.165;

    const double ROBOT_RADIUS = 0.3;
};

// Contains the logic to assemble a kit
class Arena : protected ArenaSpec {
public:

    int n_th_caddy;

    bool should_ask = true; // also change move_group_util.cpp. use `rosed caddy_manipulation move_group_util.cpp`

    Arena(const int n_th_caddy = 1, bool wait_move_base_server = true) : n_th_caddy(n_th_caddy), nav_client("move_base") {
        if (wait_move_base_server) {
            ROS_INFO("Waiting for the move_base action server...");
            nav_client.waitForServer();
        }
    }

    void run_forever() {
        while (true) {
            ROS_INFO_STREAM("n_th_caddy=" << n_th_caddy);
            run();
            n_th_caddy += 1;
        }
    }

    void ask(std::string what) {
//        std::cout << what; std::cin.ignore();
    }


    virtual void run() {
        std::cout << "Did you run\n`rostopic hz`\n`Jordan's widget detector & manipulation node`\ninitialize pose for move_base? Press Enter to run..."; std::cin.ignore();

        ROS_INFO_STREAM("Running now...");

        ROS_INFO_STREAM("Remembering rotation for caddy " << n_th_caddy << "...");
        CaddyRotationParam first_caddy_rotation_param;
        first_caddy_rotation_param.forget(); // always start clear; may not forget if stop early
        first_caddy_rotation_param.remember();

        ask("Press Enter to go to safe place for arm movement...");
        go_to_safest_place_for_arm_movement();

        ask("Press Enter to prepare to pick...");
        ros::AsyncSpinner spinner(1);
        spinner.start();
        MoveFetch move_fetch;
        prepare_to_pick(move_fetch);

        ask("Press Enter to machine large gear...");
        go_machine_large_gear();

        ask("Press Enter to pick small gear...");
        go_pick_small_gear();
        ask("Press Enter to place small gear...");
        go_place_small_gear();
        prepare_to_pick(move_fetch);

        ask("Press Enter to pick gearbox top...");
        go_pick_gearbox_top();
        ask("Press Enter to place a gearbox top...");
        go_place_gearbox();
        prepare_to_pick(move_fetch);

        ask("Press Enter to pick gearbox bottom...");
        go_pick_gearbox_bottom();
        ask("Press Enter to place a gearbox bottom...");
        go_place_gearbox();
        prepare_to_pick(move_fetch);

        // 2 screws
        ask("Press Enter to pick 1st screw...");
        go_pick_screw();
        go_between_screw_and_kit_station_in_manhattan_style();
        ask("Press Enter to place 1st screw...");
        go_place_screw();
        prepare_to_pick(move_fetch);
        go_between_screw_and_kit_station_in_manhattan_style();
        ask("Press Enter to pick 2nd screw...");
        go_pick_screw();
        go_between_screw_and_kit_station_in_manhattan_style();
        ask("Press Enter to place 2nd screw...");
        go_place_screw();
        prepare_to_pick(move_fetch);

        ask("Press Enter to pick machined large gear...");

        go_pick_machined_large_gear();
        ask("Press Enter to place machined large gear...");
        go_place_machined_large_gear();
        prepare_to_pick(move_fetch);

        ask("Press Enter to pick caddy...");
        go_pick_caddy();
        ask("Press Enter to drop off caddy...");
        go_drop_off_caddy();

        ROS_INFO_STREAM("Forgetting leftmost caddy rotation...");
        first_caddy_rotation_param.forget();

        ROS_INFO_STREAM("Delivered on kit!!!");
    }

    void go_to_safest_place_for_arm_movement() {
        nav_to_table_front(MapPose{ARENA_LENGTH - SCHUNK_TABLE_WIDTH - DISTANCE - 0.1 - ROBOT_RADIUS,
                                   TABLE_WIDTH + (ARENA_LENGTH - TABLE_WIDTH - TABLE_WIDTH) / 2.0 + 0.1,
                                   M_PI});
    }

    void prepare_to_pick(MoveFetch &move_fetch) {
        // https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo/scripts/prepare_simulated_robot.py#L41
        std::vector<std::string> joints = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
        std::vector<double> values = {0.36, 1.57, 0, 0.0, -1.7, 0.0, -0.57, 0.0};

        move_group_util::set_joints_target_and_execute(move_fetch, joints, values);
    }



    void go_machine_large_gear() {
        ask("Going to schunk table (pick large gear)...");
        nav_to_table_front(get_schunk_table_pose(0.3, 2));
        ROS_INFO_STREAM("Picking large gear...");
        pick_large_gear();

        ask("Going to schunk table (insert into chuck)...");
        nav_to_see_chuck();
        ask("Inserting large gear into chuck...");
        bool result = insert_large_gear();
        nav_to_see_chuck_backup();

        ask("Going to schunk table (insert into chuck)...");
        nav_to_see_chuck();
        ask("Inserting large gear into chuck...");
        insert_large_gear();
        nav_to_see_chuck_backup();
/*
        ask("Going to schunk table (close door)...");
        nav_to_see_door_handle();
        ask("Closing schunk machine door...");
        close_schunk_machine_door();
        nav_to_see_door_handle_backup();
*/    }

    virtual MapPose get_schunk_table_pose(double percentage_to_left = 0.5, double percentage_of_distance = 1) = 0;

    MapPose get_schunk_table_pose_to_see_chuck() {
        return get_schunk_table_pose(0.2, 0);
//        auto _ = get_schunk_table_pose(0.9, 0);
//        _.x = 3.05 - 0.95;
//        return _;
    }

    void nav_to_see_chuck() {
//nav_to_table_front(get_schunk_table_pose_to_see_chuck(), MapPose{-0.5, 0, 0});
        nav_to_table_front(get_schunk_table_pose_to_see_chuck(), MapPose{0, 0, 0});
    }


    void nav_to_see_chuck_backup() {
        auto pose = get_schunk_table_pose_to_see_chuck();
        pose.x -= 0.5;
        nav_to_table_front(pose);
    }

    MapPose get_schunk_table_pose_to_see_handle() {
        return get_schunk_table_pose(0.3, 0);
//        auto _ = get_schunk_table_pose(0.3, 0);
//        _.x = 3.05 - 0.95;
//        return _;
    }

    void nav_to_see_door_handle() {
        nav_to_table_front(get_schunk_table_pose_to_see_handle(), MapPose{-0.5, 0, 0});
    }


    void nav_to_see_door_handle_backup() {
        auto pose = get_schunk_table_pose_to_see_handle();
        pose.x -= 0.5;
        nav_to_table_front(pose);
    }

    virtual void pick_large_gear() = 0;

    virtual bool insert_large_gear() = 0;

    virtual void close_schunk_machine_door() = 0;

    void go_pick_small_gear() {
        ROS_INFO_STREAM("Going to schunk table...");
        nav_to_table_front(get_schunk_table_pose(0.7));

        ROS_INFO_STREAM("Picking up a small gear...");
        pick_small_gear();
    }

    virtual void pick_small_gear() = 0;


    void go_place_small_gear() {
        ROS_INFO_STREAM("Going to kit station...");
        nav_to_table_front(get_kit_station_pose());

        ROS_INFO_STREAM("Placing a small gear...");
        place_small_gear();
    }

    virtual void place_small_gear() = 0;


    void go_pick_screw() {
        ROS_INFO_STREAM("Going to screw bin pick station...");
        nav_to_table_front(get_screw_bin_station_pose());

        ROS_INFO_STREAM("Picking up a screw...");
        pick_up_screw();
    }

    virtual MapPose get_screw_bin_station_pose() = 0;

    void go_between_screw_and_kit_station_in_manhattan_style() {
        double x = get_screw_bin_station_pose().x;
        double y = get_kit_station_pose().y;
        double angle_towards_origin = M_PI / 4.0 * 3.0;
        nav_to_table_front(MapPose{x, y, angle_towards_origin});
    }

    virtual void pick_up_screw() = 0;



    void go_place_screw() {
        ROS_INFO_STREAM("Going to kit station...");
        nav_to_table_front(get_kit_station_pose());

        ROS_INFO_STREAM("Placing a screw...");
        place_screw();
    }

    virtual void place_screw() = 0;



    void go_pick_gearbox_top() {
        ROS_INFO_STREAM("Going to gearbox pick station...");
        nav_to_table_front(get_gearbox_station_pose(1.0/3.0));

        ROS_INFO_STREAM("Picking up a gearbox top...");
        pick_up_gearbox_top();
    }

    void go_pick_gearbox_bottom() {
        ROS_INFO_STREAM("Going to gearbox pick station...");
        nav_to_table_front(get_gearbox_station_pose(0.76));

        ROS_INFO_STREAM("Picking up a gearbox bottom...");
        pick_up_gearbox_bottom();
    }

    virtual MapPose get_gearbox_station_pose(double percentage_to_right = 0.5) = 0;

    virtual void pick_up_gearbox_top() = 0;

    virtual void pick_up_gearbox_bottom() = 0;



    void go_place_gearbox() {
        ROS_INFO_STREAM("Going to kit station...");
        nav_to_table_front(get_kit_station_pose());

        ROS_INFO_STREAM("Placing a gearbox top/bottom...");
        place_gearbox();
    }

    virtual void place_gearbox() = 0;



    void go_pick_machined_large_gear() {
        //ask("Going to schunk table (open door)...");
        //nav_to_see_door_handle();
        //ask("Opening schunk machine door...");
        //open_schunk_machine_door();
        //nav_to_see_door_handle_backup();

        ask("Going to schunk table (insert into chuck)...");
        nav_to_see_chuck();
        ask("Removing large gear into chuck...");
        remove_large_gear();
        nav_to_see_chuck_backup();
    }

    virtual void open_schunk_machine_door() = 0;

    virtual void remove_large_gear() = 0;



    void go_place_machined_large_gear() {
        ROS_INFO_STREAM("Going to kit station...");
        nav_to_table_front(get_kit_station_pose());

        ROS_INFO_STREAM("Placing large gear...");
        place_machined_large_gear();
    }

    virtual void place_machined_large_gear() = 0;



    void go_pick_caddy() {
        ROS_INFO_STREAM("Going to kit station...");
        nav_to_table_front(get_kit_station_pose());

        ROS_INFO_STREAM("Picking up a caddy...");
        pick_up_caddy();
    }

    virtual MapPose get_kit_station_pose() = 0;

    virtual void pick_up_caddy() = 0;



    void go_drop_off_caddy() {
        ROS_INFO_STREAM("Going to drop off station...");
        nav_to_table_front(get_drop_off_station_pose());

        ROS_INFO_STREAM("Dropping off the caddy...");
        drop_off_caddy();
    }

    virtual MapPose get_drop_off_station_pose() = 0;

    virtual void drop_off_caddy() = 0;



    ~Arena() = default;

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_client;

    bool nav_to_table_front(MapPose map_pose, MapPose before_translation = {0, 0, 0}) {
//        ROS_INFO_STREAM("Navigating to arena empty space center...");
//        nav_to(MapPose{ARENA_EMPTY_SPACE_CENTER_X, ARENA_EMPTY_SPACE_CENTER_Y, map_pose.theta});
        if ( ! before_translation.empty()) {
            auto before = map_pose;
            before.x += before_translation.x;
            before.y += before_translation.y;
            before.theta += before_translation.theta;
            nav_to(before);
        }
        nav_to(map_pose);
    }

    bool nav_to(MapPose map_pose) {
        ROS_INFO_STREAM("Navigating to" << map_pose << "...");

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = map_pose.to_pose_stamped_msg();

        ROS_INFO("Sending goal...");
        nav_client.sendGoal(goal);

        ROS_INFO("Waiting for result...");
        nav_client.waitForResult();

        bool result = nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
        if (result) {
            ROS_INFO_STREAM("Goal succeeded - " << nav_client.getState().getText());
        }
        else {
            ROS_INFO_STREAM("Goal failed - " << nav_client.getState().getText());
        }

        return result;
    }
};
