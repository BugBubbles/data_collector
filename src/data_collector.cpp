#include <cmath>
#include "data_collector.hpp"
// #include "label_data.hpp"
namespace gazebo
{
  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(DataCollector)
  void DataCollector::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    if (!_model)
      gzerr << "Invalid sensor pointer." << std::endl;
    if (_sdf->HasElement("enable") && _sdf->GetElement("enable")->Get<bool>())
    {

      std::string event_sub, image_sub, label_dir;
      DataCollector::SdfParse(_sdf, image_sub, event_sub, this->output_dir, label_dir);
      DataCollector::dirCheck(this->output_dir, this->output_dir);

      // DataCollector::loadLabel(label_dir, 0, 0);

      this->f_pose.open(this->output_dir + "/pose.csv");
      this->f_events.open(this->output_dir + "/events.csv");

      gzmsg << "[DataCollector] Subscribing to: " << image_sub << event_sub << std::endl;
      this->image_sub_ = this->node_handle_.subscribe(image_sub, 100, &DataCollector::ImageCallback, this);
      this->event_sub_ = this->node_handle_.subscribe(event_sub, 100, &DataCollector::EventCallback, this);

      this->model = _model;
      // 订阅更新事件
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DataCollector::OnUpdate, this));
    }
    else
    {
      ROS_INFO("[DataCollector] Enable fail. This plugin will not be enabled.");
    }
  }

  // 在每个仿真步骤中调用
  void DataCollector::OnUpdate()
  {
    // 获取模型的位置
    auto pos = this->model->WorldPose().Pos();
    auto ori = this->model->WorldPose().Rot();
    auto time = ros::Time::now();
    this->f_pose << time.toSec() << "," << pos.X() << "," << pos.Y() << "," << pos.Z() << "," << ori.Pitch() << "," << ori.Yaw() << "," << ori.Roll() << std::endl;
    // this->model->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
  }
  void DataCollector::EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msgs)
  {
    // 从消息中提取事件信息
    auto events = event_msgs->events;
    for (const auto &event : events)
    {
      this->f_events << "x: " << event.x << ", y: " << event.y << ", t: " << event.ts.toSec() << ", p: " << event.polarity << std::endl;
    }
  }
  void DataCollector::ImageCallback(const sensor_msgs::Image::ConstPtr &image_msgs)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;

      std::stringstream name;
      name << this->output_dir << "/images/" << image_msgs->header.stamp.toSec() << ".png";
      cv::imwrite(name.str(), image);
    }
    catch (cv_bridge::Exception &e)
    {
      gzerr << "cv_bridge exception: " << e.what() << std::endl;
    }
  }
  void DataCollector::SdfParse(const sdf::ElementPtr _sdf, std::string &image_sub, std::string &event_sub, std::string &output_dir, std::string &label_dir)
  {
    // parse the sdf files
    if (_sdf->HasElement("image_sub"))
      image_sub = _sdf->GetElement("image_sub")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a image_sub." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("event_sub"))
      event_sub = _sdf->GetElement("event_sub")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a event_sub." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("label_dir"))
      label_dir = _sdf->GetElement("label_dir")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a label_dir." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("output_dir"))
      output_dir = _sdf->GetElement("output_dir")->Get<std::string>();
    else
    {
      gzwarn << "[DataCollector] Implicit output directory, default output directory was set '~/.ros/data'." << std::endl;
      output_dir = "data";
    }
  }
  void DataCollector::dirCheck(std::string dir, std::string &output_dir)
  {
    if (!std::filesystem::exists(dir))
    {
      std::filesystem::create_directories(dir);
    }
    else
    {
      ROS_INFO("%s exists, skipping creating.", dir.c_str());
    }
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
    output_dir = dir + "/" + ss.str();
    std::filesystem::create_directories(output_dir);
    std::filesystem::create_directories(output_dir + "/images");
  }
  void DataCollector::loadLabel(std::string label_dir, int row, int col, int row_num, int col_num)
  {
    std::string label_path;
    for (int i = 0; i < row_num; i++)
    {
      for (int j = 0; j < col_num; j++)
      {
        label_path = label_dir + "/" + std::to_string(row + i) + "/lbl_" + std::to_string(row + i) + "_" + std::to_string(col + j) + "_960.txt";
        if (!std::filesystem::exists(label_path))
        {
          ROS_INFO("[LabelData] Label file %s does not exist.", label_path.c_str());
          continue;
        }
        else
        {
          std::ifstream f_label(label_path, std::ios::in);
          std::string line;
          while (std::getline(f_label, line, '\n'))
          {
            std::vector<double> data;
            split(line, ',', data);
            this->X.push_back(data[0]);
            this->Y.push_back(data[1]);
            this->R.push_back(data[7]);
            this->Lat.push_back(data[5]);
          }
        }
      }
    }
    this->data_len = this->X.size();
  }
  void DataCollector::getLabelImage(ignition::math::Pose3d &pose, cv::Mat &img)
  {
    // TODO : 先画出图来
    auto pos = this->model->WorldPose().Pos();
    auto ori = this->model->WorldPose().Rot();

    for (int i = 0; i < this->data_len; i++)
    {
      auto x = this->X[i];
      auto y = this->Y[i];
      ignition::math::Vector3<double> point_world;
      auto r = this->R[i];
      auto lat = this->Lat[i];
      double R_x = r * this->scale_x;
      double R_y = r * this->scale_y / cos(lat * PI / 180);
    }
    // 世界坐标系向相机坐标系的变换
    auto x = pos.X() * cos(ori.Yaw()) + pos.Y() * sin(ori.Yaw());
    auto y = -pos.X() * sin(ori.Yaw()) + pos.Y() * cos(ori.Yaw());
    auto z = pos.Z();
    // 相机坐标系向图像坐标系的变换
    auto u = x / z;
    auto v = y / z;
    // 图像坐标系向像素坐标系的变换
    auto row = 960 * (1 - v) / 2;
    auto col = 960 * (1 + u) / 2;
    // 画出图像
    cv::circle(img, cv::Point(col, row), 5, cv::Scalar(0, 0, 255), -1);
  }
  void split(std::string str, char del, std::vector<double> &res)
  {
    std::stringstream ss(str);
    std::string tmp;
    while (getline(ss, tmp, del))
    {
      res.push_back(std::stod(tmp));
    }
  }
}
