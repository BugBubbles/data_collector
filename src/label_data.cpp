#include "label_data.hpp"
namespace gazebo
{
  LabelData::LabelData()
  {
  }

  LabelData::LabelData(std::string label_dir, int row, int col, int row_num, int col_num)
  {
    label_dir = label_dir + "/" + std::to_string(0) + "/lbl_" + std::to_string(0) + "_" + std::to_string(0) + "_960.txt";
    // std::string label_path;
    // for (int i = 0; i < row_num; i++)
    // {
    //   for (int j = 0; j < col_num; j++)
    //   {
    //     label_path = label_dir + "/" + std::to_string(row + i) + "/lbl_" + std::to_string(row + i) + "_" + std::to_string(col + j) + "_960.txt";
    //     if (!std::filesystem::exists(label_path))
    //     {
    //       ROS_INFO("[LabelData] Label file %s does not exist.", label_path.c_str());
    //       continue;
    //     }
    //     else
    //     {
    //       this->f_label.open(label_path, std::ios_base::in);
    //       while (!this->f_label.eof())
    //       {
    //         double x, y, r, lat;
    //         std::string buffer;
    //         this->f_label.getline(buffer, 256, ',');
    //         // std::cout << buffer << std::endl;
    //         ROS_INFO("[LabelData] %s", buffer.c_str())
    //         // this->f_label >> x >> y >> r >> lat;
    //         // this->X.push_back(x);
    //         // this->Y.push_back(y);
    //         // this->R.push_back(r);
    //         // this->Lat.push_back(lat);
    //       }
    //     }
    //   }
    // }
  }
  LabelData::~LabelData()
  {
  }
  cv_bridge::CvImagePtr getLabelImage(ignition::math::Pose3d &pose)
  {
  }
  LabelData::LabelDataPtr getPtr()
  {
    return this;
  }
}