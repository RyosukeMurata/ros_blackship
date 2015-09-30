
#include "bs_controller/CBlackship.h"

CBlackship::CBlackship(const ros::NodeHandle& nh) :
    mNodeHandle(nh, "bs_controller") {

    mInputVel = 0.0;
    mInputAVel = 0.0;
    mStopFlag = false;
    // mPubOdmetry = mNode.advertise<nav_msgs::Odometry::ConstPtr>("bs_odmetry", 100);
}

CBlackship::~CBlackship() {
    mBsIF.blackship_close();
}

bool CBlackship::initialize() {
    return mBsIF.blackship_open(bs::STR_PORT);
}

void CBlackship::inputCallback(const geometry_msgs::TwistStamped::ConstPtr& _input) {
    setSpeed(setInputFromRos(_input->twist));
}

bool CBlackship::setInputFromRos(geometry_msgs::Twist _input) {
    return limitVel(_input.linear.x, _input.angular.z);
}

bool CBlackship::limitVel(double _vel, double _avel) {

    mInputVel = myUtils::limitMaxMin(_vel, bs::MAX_V, -bs::MAX_V);
    mInputAVel = myUtils::limitMaxMin(_avel, bs::MAX_W, -bs::MAX_W);

    return (fabs(mInputVel) <= bs::MAX_V) && (fabs(mInputAVel) <= bs::MAX_W);
}

void CBlackship::setSpeed(bool _setFlag) {

    double vel = mStopFlag ? 0.0 : bs::MPS2BS * bs::K_SPEED * mInputVel;
    double k_slip = 1;
    if (bs::USE_SLIP) {
        k_slip = 1 / (1.0 - 2.0 * bs::K_SLIP);
    }
    double avel = mStopFlag ? 0.0 : k_slip * bs::MPS2BS * bs::K_TURN * (bs::WHEELWIDTH * 0.5 * mInputAVel);

    if (_setFlag) {
        int leftWheelVel = (int)myUtils::limitMaxMin((vel - avel), 100, -100);
        int rightWheelVel = (int)myUtils::limitMaxMin((vel + avel), 100, -100);
        mBsIF.blackship_set_speed(leftWheelVel, rightWheelVel);
    }
}

void CBlackship::activate() {
    mSubInput = mNodeHandle.subscribe("bs_input", 100, &CBlackship::inputCallback, this);
    mStopService = mNodeHandle.advertiseService("stop_service", &CBlackship::setStopFlagServiceHandler, this);
}

void CBlackship::publish_odom() {
    // TODO
}

bool CBlackship::setStopFlagServiceHandler(std_srvs::Trigger::Request& req,
                                           std_srvs::Trigger::Response& res) {
    mStopFlag = !mStopFlag;
    res.success = true;
    res.message = mStopFlag ? "true" : "false";
    return true;
}


