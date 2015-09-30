#ifndef CBLACKSHIP_H
#define CBLACKSHIP_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include "bs_if.h"
#include "bs_config.h"
#include "myUtils.h"
#include <iostream>
#include <stdlib.h>


class CBlackship {

public:
    CBlackship(const ros::NodeHandle& nh);
    ~CBlackship();
    bool initialize();
    void activate();
    void publish_odom();

private:
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mSubInput;
    ros::ServiceServer mStartService;
    ros::ServiceServer mStopService;

    //ros::Publisher mPubOdmetry;
    void inputCallback(const geometry_msgs::TwistStamped::ConstPtr& _input);
    bool setStopFlagServiceHandler(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res);
    bool setStartFlagServiceHandler(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);

    CBlackshipIF mBsIF;
    double mInputVel;
    double mInputAVel;
    bool mStopFlag;

    bool setInputFromRos(geometry_msgs::Twist _input);
    bool limitVel(double _vel, double _avel);
    void setSpeed(bool setFlag);
};

#endif


