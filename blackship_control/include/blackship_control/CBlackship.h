#ifndef CBLACKSHIP_H
#define CBLACKSHIP_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "bs_if.h"
#include "bs_config.h"
#include "myUtils.h"
#include <iostream>
#include <stdlib.h>


class CBlackship {

public:
    CBlackship();
    ~CBlackship();
    void run();

private:
    ros::NodeHandle mNode;
    ros::Subscriber mSubInput;
    //ros::Publisher mPubOdmetry;
    void inputCallback(const geometry_msgs::TwistStamped::ConstPtr& _input);

    CBlackshipIF mBsIF;
    double mInputVel;
    double mInputAVel;
    bool mStopFlag;

    bool init();
    bool setInputFromRos(geometry_msgs::Twist _input);
    bool limitVel(double _vel, double _avel);
    void setSpeed(bool setFlag);
};
#endif


