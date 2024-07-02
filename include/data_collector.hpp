#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace gazebo
{
  class DataCollector : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msgs);
    void EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msgs);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &image_msgs);
    void ImageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs);

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

  protected:
    ros::Subscriber pose_sub_, image_sub_, event_sub_;
    ros::NodeHandle node_handle_;
  };
}