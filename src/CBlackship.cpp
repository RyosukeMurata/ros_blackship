
#include "ros_blackship/CBlackship.h"

CBlackship::CBlackship() {

    mInputVel = 0.0;
    mInputAVel = 0.0;
    mStopFlag = false;
    // mPubOdmetry = mNode.advertise<nav_msgs::Odometry::ConstPtr>("bs_odmetry", 100);
    mSubInput = mNode.subscribe("bs_input", 100, &CBlackship::inputCallback, this);

}

CBlackship::~CBlackship() {
    mBsIF.blackship_close();
}

bool CBlackship::init() {
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

    double vel = !mStopFlag ? bs::MPS2BS * bs::K_SPEED * mInputVel : 0.0;
    double k_slip = 1;
    if (bs::USE_SLIP) {
        k_slip = 1 / (1.0 - 2.0 * bs::K_SLIP);
    }
    double avel = !mStopFlag ? k_slip * bs::MPS2BS * bs::K_TURN * (bs::WHEELWIDTH * 0.5 * mInputAVel) : 0.0;

    if (_setFlag) {
        int leftWheelVel = (int)myUtils::limitMaxMin((vel - avel), 100, -100);
        int rightWheelVel = (int)myUtils::limitMaxMin((vel + avel), 100, -100);
        mBsIF.blackship_set_speed(leftWheelVel, rightWheelVel);
    }
}


void CBlackship::run() {

    if (init()) {
        ros::Rate loop_rate(30);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
            //TODO: get odometry data
        }
    }
}

