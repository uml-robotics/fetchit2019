#include "chuck_detector/ChuckDetector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "chuck_detector");

    ChuckDetector cd;
    cd.detect_chuck();
}