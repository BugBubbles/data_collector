#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <fstream>

namespace gazebo
{
  class DataCollector : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();
    void EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msgs);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &image_msgs);
    void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs);
    void EventInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs);

  private:
    void SdfParse(const sdf::ElementPtr _sdf, std::string &image_sub, std::string &event_sub, std::string &output_dir);
    void dirCheck();
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection;

  protected:
    ros::Subscriber image_sub_, event_sub_, info_cam_sub_, info_evt_sub_;
    ros::NodeHandle node_handle_;
    std::string output_dir_;
    std::ofstream f_info, f_pose, f_events;
    bool camera_write_ = false, event_write_ = false;
  };
}
