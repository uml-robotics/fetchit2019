#pragma once

#include "ArenaNav.h"
#include <std_srvs/SetBool.h>
#include <caddy_manipulation/CaddyPicking.h>
#include <caddy_manipulation/PlacingCaddy.h>
#include <caddy_manipulation/PlacingParts.h>
#include <insert_large_gear/LargeGearInserting.h>
#include <schunk_door_manip/GrabHandle.h>

class TestCourse : public ArenaNav {

public:

    ros::NodeHandle nh;

    TestCourse(const int n_th_caddy = 1, bool wait_move_base_server = true) : ArenaNav(n_th_caddy, wait_move_base_server) {}

    void pick_large_gear() override {
        ros::NodeHandle n;
        ros::ServiceClient enable_sensing_client = n.serviceClient<std_srvs::SetBool>("/large_gear/enable_sensing");
        ros::ServiceClient grasp_client = n.serviceClient<std_srvs::SetBool>("/large_gear/grasp");

        ros::ServiceClient disable_sensing_client = n.serviceClient<std_srvs::SetBool>("/disable_sensing");

        enable_sensing_client.waitForExistence(ros::Duration(-1));

        std_srvs::SetBool srv;
        srv.request.data = 0;

        sleep(5);

        bool object_grasped = false;

        while( !object_grasped)
        {
            enable_sensing_client.call(srv);
            sleep(5);
            disable_sensing_client.call(srv);
            for(int i =0; i < 3; i++)
            {
                object_grasped = grasp_client.call(srv);
                if(object_grasped)
                    break;
            }
        }
    }

    bool insert_large_gear() override {
        LargeGearInserting i;
        return i.insert();
    }

    void close_schunk_machine_door() override {
        GrabHandle gh;
        gh.close();
    }

    void pick_small_gear() override {
        ros::NodeHandle n;
        ros::ServiceClient enable_sensing_client = n.serviceClient<std_srvs::SetBool>("/small_gear/enable_sensing");
        ros::ServiceClient grasp_client = n.serviceClient<std_srvs::SetBool>("/small_gear/grasp");

        ros::ServiceClient disable_sensing_client = n.serviceClient<std_srvs::SetBool>("/disable_sensing");

        enable_sensing_client.waitForExistence(ros::Duration(-1));

        std_srvs::SetBool srv;
        srv.request.data = 0;

        sleep(5);

        bool object_grasped = false;

        while( !object_grasped)
        {
            enable_sensing_client.call(srv);
            sleep(5);
            disable_sensing_client.call(srv);
            for(int i =0; i < 3; i++)
            {
                object_grasped = grasp_client.call(srv);
                if(object_grasped)
                    break;
            }
        }
    }

    void place_small_gear() override {
        PlacingParts p;
        p.place_small_gear();
    }

    void pick_up_screw() override {
        ros::NodeHandle n;
        ros::ServiceClient enable_sensing_client = n.serviceClient<std_srvs::SetBool>("/screw/enable_sensing");
        ros::ServiceClient grasp_client = n.serviceClient<std_srvs::SetBool>("/screw/grasp");

        ros::ServiceClient disable_sensing_client = n.serviceClient<std_srvs::SetBool>("/disable_sensing");

        enable_sensing_client.waitForExistence(ros::Duration(-1));

        std_srvs::SetBool srv;
        srv.request.data = 0;

        sleep(5);


        bool object_grasped = false;

        while( !object_grasped)
        {
            enable_sensing_client.call(srv);
            sleep(5);
            disable_sensing_client.call(srv);
            for(int i =0; i < 3; i++)
            {
                object_grasped = grasp_client.call(srv);
                if(object_grasped)
                    break;
            }
        }
    }

    void place_screw() override {
        PlacingParts p;
        p.place_bolt();
    }

    void pick_up_gearbox_top() override {
        ros::NodeHandle n;
        ros::ServiceClient enable_sensing_client = n.serviceClient<std_srvs::SetBool>("/top_box/enable_sensing");
        ros::ServiceClient grasp_client = n.serviceClient<std_srvs::SetBool>("/top_box/grasp");

        ros::ServiceClient disable_sensing_client = n.serviceClient<std_srvs::SetBool>("/disable_sensing");

        enable_sensing_client.waitForExistence(ros::Duration(-1));

        std_srvs::SetBool srv;
        srv.request.data = 0;

        sleep(5);

        bool object_grasped = false;

        while( !object_grasped)
        {
            enable_sensing_client.call(srv);
            sleep(5);
            disable_sensing_client.call(srv);
            for(int i =0; i < 3; i++)
            {
                object_grasped = grasp_client.call(srv);
                if(object_grasped)
                    break;
            }
        }
    }

    void pick_up_gearbox_bottom() override {
        ros::NodeHandle n;
        ros::ServiceClient enable_sensing_client = n.serviceClient<std_srvs::SetBool>("/bottom_box/enable_sensing");
        ros::ServiceClient grasp_client = n.serviceClient<std_srvs::SetBool>("/bottom_box/grasp");

        ros::ServiceClient disable_sensing_client = n.serviceClient<std_srvs::SetBool>("/disable_sensing");

        enable_sensing_client.waitForExistence(ros::Duration(-1));

        std_srvs::SetBool srv;
        srv.request.data = 0;

        sleep(5);

        bool object_grasped = false;

        while( !object_grasped)
        {
            enable_sensing_client.call(srv);
            sleep(5);
            disable_sensing_client.call(srv);
            for(int i =0; i < 3; i++)
            {
                object_grasped = grasp_client.call(srv);
                if(object_grasped)
                    break;
            }
        }
    }

    void place_gearbox() override {
        PlacingParts p;
        p.place_gearbox();
    }

    void pick_machined_large_gear() {
        sleep(2);
    }

    void open_schunk_machine_door() override {
        GrabHandle gh;
        gh.open();
    }

    void remove_large_gear() override {
        LargeGearInserting i;
        i.remove();
    }

    void place_machined_large_gear() override {
        PlacingParts p;
        p.place_large_gear();
    }

    void pick_up_caddy() override {
        CaddyPicking();
    }

    void drop_off_caddy() override {
        PlacingCaddy();
    }

private:

};
