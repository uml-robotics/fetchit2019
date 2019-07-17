#include <ros/ros.h>

#include <fetch_cpp/TorsoLiftClient.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "fetch_look_at");
    ros::NodeHandle nh;

    TorsoLiftClient torsoLiftClient;
    torsoLiftClient.lift_to_highest();
//    torsoLiftClient.lift_to_lowest();
}