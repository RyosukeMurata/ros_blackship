
#include <boost/thread.hpp>
#include <bs_simulator/bs_gazebo_plugin_test.h>
#include <ros/time.h>
#include <math.h>

using namespace gazebo;


BlackshipPlugin::BlackshipPlugin()
{

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
  this->input_vel_topic_name_ = getSDFElement(_sdf, "inputVelTopicName", std::string("cmd_vel"));
  std::string controlled_link_name = getSDFElement(_sdf, "controlledLink", std::string("blackship_base_for_calc"));
  this->controlled_link_ =  _parent->GetLink(controlled_link_name);


  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo_blackship", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  drive_sub_ = rosnode_->subscribe(this->input_vel_topic_name_, 1, &BlackshipPlugin::OnDrive, this );
  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("encoder", 1);


  std::string wheel_joint_names[] = {"joint_blackship_wheel_frontleft", "joint_blackship_wheel_frontright",
                                     "joint_blackship_wheel_rearleft","joint_blackship_wheel_rearright"};
  for(int i=0;i<4;i++){
    encoder_js_.name.push_back(wheel_joint_names[i]);
    encoder_js_.position.push_back(0);
    encoder_js_.velocity.push_back(0);
  }

  prev_update_time_ = last_cmd_vel_time_ = 0;

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

//  // Timeout if we haven't received a cmd in <0.1 s
//  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
//  if (time_since_last_cmd.Double() > 0.1){
//      body_linear_vel_.Set(0,0,0);
//      body_angular_vel_.Set(0,0,0);
//  }

  controlled_link_->SetLinearVel(body_linear_vel_);
  controlled_link_->SetAngularVel(body_angular_vel_);

  joint_state_pub_.publish( encoder_js_ );
}


void BlackshipPlugin::OnDrive( const geometry_msgs::TwistStampedConstPtr &msg)
{
  last_cmd_vel_time_ = this->world_->GetSimTime();
  float v = msg->twist.linear.x;
  float w = msg->twist.angular.z;
  float theta = controlled_link_->GetWorldPose().rot.GetAsEuler().z;

  body_linear_vel_.Set(v*cos(theta),v*sin(theta),0);

  body_angular_vel_.Set(0,0,w);
}

void BlackshipPlugin::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(BlackshipPlugin);
