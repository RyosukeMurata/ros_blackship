#ifndef CBLACKSHIP_H
#define CBLACKSHIP_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "bs_if.h"
#include "bs_config.h"
#include <iostream>


class CBlackship {

public:
    CBlackship();
    ~CBlackship();
    void run();

private:
    bool init();
    ros::NodeHandle _node;
    ros::Subscriber _subInput;
    ros::Publisher _pubOdmetry;

    double limitVel(double v, double w);

#endif


