
#include <boost/thread.hpp>
#include <bs_simulator/bs_gazebo_plugin.h>
#include <ros/time.h>

using namespace gazebo;


BlackshipPlugin::BlackshipPlugin():
    wheel_fl_(), wheel_fr_(), wheel_rl_(), wheel_rr_()
{
    // Initialize the ROS node and subscribe to cmd_vel
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_blackship", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    rosnode_ = new ros::NodeHandle( node_namespace_ );

    drive_sub_ = rosnode_->subscribe("cmd_vel", 1, &BlackshipPlugin::OnDrive, this );
//    encoder_pub_  = rosnode_->advertise<sensor_msgs::JointState>("encoders", 1);
    joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);
}

BlackshipPlugin::~BlackshipPlugin()
{
  delete rosnode_;
  delete spinner_thread_;
}

void BlackshipPlugin::FiniChild()
{
  rosnode_->shutdown();
  spinner_thread_->join();
}

void BlackshipPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();

  this->node_namespace_ = getSDFElement(_sdf, "robotNamespace", std::string(""));
  std::string fl_joint_name = getSDFElement(_sdf, "frontLeftWheelJoint", std::string("joint_blackship_wheel_frontleft"));
  std::string fr_joint_name = getSDFElement(_sdf, "frontRightWheelJoint", std::string("joint_blackship_wheel_frontright"));
  std::string rl_joint_name = getSDFElement(_sdf, "rearLeftWheelJoint", std::string("joint_blackship_wheel_rearleft"));
  std::string rr_joint_name = getSDFElement(_sdf, "rearRightWheelJoint",std::string("joint_blackship_wheel_rearright"));
  wheel_torque_ = getSDFElement(_sdf, "wheelTorque", 15.0);
  wheel_radius_ = getSDFElement(_sdf, "wheelRadius", 0.160);
  wheel_track_ = getSDFElement(_sdf, "wheelTrack", 0.340);

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");
  gzdbg << "plugin model name: " << modelName << "\n";

  this->wheel_fl_.initialize(this->model_, fl_joint_name);
  this->wheel_fr_.initialize(this->model_, fr_joint_name);
  this->wheel_rl_.initialize(this->model_, rl_joint_name);
  this->wheel_rr_.initialize(this->model_, rr_joint_name);

  js_push_back(wheel_fl_);
  js_push_back(wheel_fr_);
  js_push_back(wheel_rl_);
  js_push_back(wheel_rr_);

  prev_update_time_ = 0;
  last_cmd_vel_time_ = 0;

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &BlackshipPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&BlackshipPlugin::UpdateChild, this));
}


void BlackshipPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  wheel_fl_.update(wheel_ang_vel_.front_left, wheel_torque_);
  wheel_fr_.update(wheel_ang_vel_.front_right, wheel_torque_);
  wheel_rl_.update(wheel_ang_vel_.rear_left, wheel_torque_);
  wheel_rr_.update(wheel_ang_vel_.rear_right, wheel_torque_);

  js_update(0, wheel_fl_);
  js_update(1, wheel_fr_);
  js_update(2, wheel_rl_);
  js_update(3, wheel_rr_);

//  encoder_pub_.publish( js_ );
  joint_state_pub_.publish( js_ );

  // Timeout if we haven't received a cmd in <0.1 s
  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
  if (time_since_last_cmd.Double() > 0.1)
  {
    wheel_ang_vel_.front_left = 0;
    wheel_ang_vel_.front_right = 0;
    wheel_ang_vel_.rear_left = 0;
    wheel_ang_vel_.rear_right = 0;
  }
}


void BlackshipPlugin::OnDrive( const geometry_msgs::TwistStampedConstPtr &msg)
{
  last_cmd_vel_time_ = this->world_->GetSimTime();
  float v = msg->twist.linear.x;
  float w = msg->twist.angular.z;
  wheel_ang_vel_.front_right = wheel_ang_vel_.rear_right = (1/wheel_radius_)*(v + w*wheel_track_/2.0);
  wheel_ang_vel_.front_left = wheel_ang_vel_.rear_left = (1/wheel_radius_)*(v - w*wheel_track_/2.0);
}

void BlackshipPlugin::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(BlackshipPlugin);
