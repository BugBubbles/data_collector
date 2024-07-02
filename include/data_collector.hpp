#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

namespace gazebo
{
  class GravityPlugin : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

  protected:
    ros::Subscriber pose_sub_, image_sub_, event_sub;
    ros::NodeHandle node_handle_;
  };
}