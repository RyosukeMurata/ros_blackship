#ifndef BS_GAZEBO_PLUGINS_H
#define  BS_GAZEBO_PLUGINS_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/gzmath.hh"

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


namespace gazebo
{
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
      void OnDrive( const geometry_msgs::TwistStampedConstPtr &msg);
      template <class T>
      T getSDFElement(sdf::ElementPtr _sdf, const std::string& element_name, const T& default_element) {
          if (_sdf->HasElement(element_name)){
              return _sdf->GetElement(element_name)->Get<T>();
          } else {
              return default_element;
          }
      }
      /// Parameters
      std::string node_namespace_;
      std::string input_vel_topic_name_;
      physics::LinkPtr controlled_link_;

      ros::NodeHandle *rosnode_;
      ros::Subscriber drive_sub_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      math::Vector3 body_linear_vel_;
      math::Vector3 body_angular_vel_;
      physics::JointPtr wheel_fl_joint_;
      physics::JointPtr wheel_fr_joint_;
      physics::JointPtr wheel_rl_joint_;
      physics::JointPtr wheel_rr_joint_;

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











