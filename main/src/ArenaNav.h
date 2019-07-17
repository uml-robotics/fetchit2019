#pragma once

#include "Arena.h"

class ArenaNav : public Arena {

public:

    ArenaNav(const int n_th_caddy = 1, bool wait_move_base_server = true) : Arena(n_th_caddy, wait_move_base_server) {}

    // changed
    MapPose get_schunk_table_pose(double percentage_to_left, double percentage_of_distance) override {
//        static const double DISTANCE_TO_WALL = 0.7;
//
//        return MapPose{DISTANCE_TO_WALL + SCHUNK_TABLE_LENGTH * percentage_to_left,
//                       ARENA_LENGTH - SCHUNK_TABLE_WIDTH - DISTANCE * percentage_of_distance - ROBOT_RADIUS, M_PI_2};

        return MapPose{ARENA_LENGTH - SCHUNK_TABLE_WIDTH - DISTANCE * percentage_of_distance - ROBOT_RADIUS,
                       DROPOFF_TABLE_DISTANCE_TO_WALL + TABLE_LENGTH + SCHUNK_TABLE_DISTANCE_TO_DROPOFF_TABLE + SCHUNK_TABLE_LENGTH * percentage_to_left, 0};
    }

    MapPose get_screw_bin_station_pose() override {
        static const double DISTANCE_TO_WALL = 0.415;

        return MapPose{DISTANCE_TO_WALL + TABLE_LENGTH_2+0.15,
                       TABLE_WIDTH + DISTANCE + ROBOT_RADIUS, -M_PI_2};
    }

    // changed
    MapPose get_gearbox_station_pose(double percentage_to_right) override {
//        return MapPose{ARENA_LENGTH - TABLE_WIDTH - DISTANCE - ROBOT_RADIUS,
//                       DROPOFF_TABLE_DISTANCE_TO_WALL + TABLE_LENGTH + GEARBOX_TABLE_DISTANCE_TO_DROPOFF_TABLE + TABLE_LENGTH * percentage_to_right, 0};

        return MapPose{GEARBOX_TABLE_DISTANCE_TO_WALL + TABLE_LENGTH * percentage_to_right,
                       ARENA_LENGTH - TABLE_WIDTH - DISTANCE - ROBOT_RADIUS, M_PI_2};
    }

    MapPose get_kit_station_pose() override {
        double y = KIT_STATION_DISTANCE_TO_WALL;
        if (n_th_caddy == 1) {
            y += 0.15;
        }
        else if (n_th_caddy == 2) {
            y += TABLE_LENGTH / 2.0;
        }
        else {
            ROS_ERROR_STREAM("n_th_caddy should be either 1 or 2 !!!!!!!");
        }

        const double GAP = 0.05;

        return MapPose{GAP + TABLE_WIDTH + DISTANCE + ROBOT_RADIUS, y, M_PI};
    }


    MapPose get_drop_off_station_pose() override {
        return MapPose{1.54207295303, 1.11742887246, -0.18};
         MapPose{ARENA_LENGTH - TABLE_WIDTH - DISTANCE - ROBOT_RADIUS,
                       DROPOFF_TABLE_DISTANCE_TO_WALL + TABLE_LENGTH, 0};
    }

private:

};
