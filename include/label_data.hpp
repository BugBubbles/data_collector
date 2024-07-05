#pragma once
#include <gazebo/gazebo.hh>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <fstream>
namespace gazebo
{

  class LabelData
  {
    typedef std::unique_ptr<LabelData> LabelDataPtr;

  public:
    LabelData(std::string label_dir, int row, int col, int row_num = 3, int col_num = 5);
    LabelData();
    ~LabelData();
    cv_bridge::CvImagePtr getLabelImage(ignition::math::Pose3d &pose);
    LabelDataPtr getPtr();

  private:
    std::string labels_dir;
    std::vector<double> X, Y, R, Lat;
    std::ifstream f_label;
  };
}
