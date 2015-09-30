#ifndef BS_GAZEBO_PLUGINS_H
#define  BS_GAZEBO_PLUGINS_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


namespace gazebo
{
    struct WheelAngularVelocity{
        float front_left;
        float front_right;
        float rear_left;
        float rear_right;
    };

    class BlackshipWheel{
    public:
        BlackshipWheel():is_initialized(false){}
        ~BlackshipWheel(){}
        void initialize(physics::ModelPtr _model, std::string _joint_name){
            model_ = _model;
            joint_name_ = _joint_name;
            axis_joint_ = model_->GetJoint(joint_name_);
            is_initialized = true;
            update(0.0, 0.0);
        }
        void update(double _ang_vel, double _torque ){
            if (is_initialized){
                axis_joint_->SetVelocity( 0, _ang_vel);
                axis_joint_->SetMaxForce( 0, _torque );
            } else {
                std::cout<<"Not initialized!" << std::endl;
            }
        }
        float joint_pos(){return axis_joint_->GetAngle(0).Radian();}
        float joint_vel(){return axis_joint_->GetVelocity(0);}
        std::string joint_name(){return joint_name_;}
    private:
        bool is_initialized;
        physics::JointPtr axis_joint_;
        physics::ModelPtr model_;
        std::string joint_name_;
    };



  class BlackshipPlugin : public ModelPlugin
  {
    public:
      BlackshipPlugin();
      virtual ~BlackshipPlugin();
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnDrive( const geometry_msgs::TwistStampedConstPtr &msg);
      template <class T>
      T getSDFElement(sdf::ElementPtr _sdf, const std::string& element_name, const T& default_element) {
          if (_sdf->HasElement(element_name)){
              return _sdf->GetElement(element_name)->Get<T>();
          } else {
              return default_element;
          }
      }
      /// HACK
      void js_push_back(BlackshipWheel _wheel){
          encoder_js_.name.push_back(_wheel.joint_name());
          encoder_js_.position.push_back(_wheel.joint_pos());
          encoder_js_.velocity.push_back(_wheel.joint_vel());
      }
      /// HACK
      void js_update(int idx, BlackshipWheel _wheel){
          encoder_js_.name[idx] = _wheel.joint_name();
          encoder_js_.position[idx] = _wheel.joint_pos();
          encoder_js_.velocity[idx] = _wheel.joint_vel();
      }

      BlackshipWheel wheel_fl_;
      BlackshipWheel wheel_fr_;
      BlackshipWheel wheel_rl_;
      BlackshipWheel wheel_rr_;

      /// Parameters
      std::string node_namespace_;
      std::string input_vel_topic_name_;
      float wheel_radius_;
      float wheel_track_;
      float wheel_torque_;

      ros::NodeHandle *rosnode_;
      ros::Subscriber drive_sub_;
      ros::Publisher joint_state_pub_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      /// Speeds of the wheels
      WheelAngularVelocity wheel_ang_vel_;
      sensor_msgs::JointState encoder_js_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      void spin();
      boost::thread *spinner_thread_;
      event::ConnectionPtr contact_event_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}

#endif











