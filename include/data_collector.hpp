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
#define PI 3.1415926585
namespace gazebo
{
  class DataCollector : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();
    void EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msgs);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &image_msgs);
    void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs);

  private:
    void SdfParse(const sdf::ElementPtr _sdf, std::string &image_sub, std::string &event_sub, std::string &output_dir, std::string &label_dir);
    void dirCheck(std::string dir, std::string &output_dir);
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::vector<double> X, Y, R, Lat;
    long long int data_len;
    double scale_x = 61440 / 1737.4 / 2 * 3 / PI;
    double scale_y = 184320 / 1737.4 / 2 / PI;

  protected:
    ros::Subscriber image_sub_, event_sub_, info_sub_;
    ros::NodeHandle node_handle_;
    std::string output_dir;
    std::ofstream f_info, f_pose, f_events;
    bool info_write = false, is_write = false;
  };
  void split(std::string str, char del, std::vector<double> &res);
}
