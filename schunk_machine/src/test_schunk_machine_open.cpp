#include "schunk_machine/SchunkMachine.hpp"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_schunk_machine_open");

    SchunkMachine s;
    s.open();

    return 0;
}
