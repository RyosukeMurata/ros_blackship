#include <iostream>
#include "ros/ros.h"
#include "ros_blackship/CBlackship.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "blackship_cotrol");

    CBlackship bs;
    bs.run();

    return 0;
}
