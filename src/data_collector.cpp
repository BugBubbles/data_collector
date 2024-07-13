#include <cmath>
#include "data_collector.hpp"
namespace gazebo
{
  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(DataCollector)
  void DataCollector::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    if (!_model)
    {
      ROS_ERROR("Invalid sensor pointer.");
      return;
    }
    if (_sdf->HasElement("enable") && _sdf->GetElement("enable")->Get<bool>())
    {

      std::string event_sub, camera_sub, label_dir;
      DataCollector::SdfParse(_sdf, camera_sub, event_sub, this->output_dir, label_dir);
      DataCollector::dirCheck(this->output_dir, this->output_dir);

      this->f_pose.open(this->output_dir + "/pose.csv");
      this->f_events.open(this->output_dir + "/events.csv");
      this->f_info.open(this->output_dir + "/info.txt");

      gzmsg << "[DataCollector] Subscribing to: " << camera_sub << event_sub << std::endl;
      this->image_sub_ = this->node_handle_.subscribe(camera_sub + "/image_raw", 100, &DataCollector::ImageCallback, this);
      this->event_sub_ = this->node_handle_.subscribe(event_sub, 100, &DataCollector::EventCallback, this);
      this->info_sub_ = this->node_handle_.subscribe(camera_sub + "/camera_info", 100, &DataCollector::InfoCallback, this);

      this->model = _model;
      // 订阅更新事件
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DataCollector::OnUpdate, this));
    }
    else
    {
      ROS_WARN("[DataCollector] Enable fail. This plugin will not be enabled.");
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
      if (this->is_write == false && this->info_write == false)
      {
        this->info_write = true;
      }
    }
    catch (cv_bridge::Exception &e)
    {
      gzerr << "cv_bridge exception: " << e.what() << std::endl;
    }
  }
  void DataCollector::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs)
  {
    if (this->info_write == true && this->is_write == false)
    {
      this->f_info << "P: ";
      for (const auto &p : info_msgs->P)
      {
        this->f_info << p << ",";
      }
      this->f_info << std::endl;
      this->f_info << "Height: " << info_msgs->height << std::endl;
      this->f_info << "Width: " << info_msgs->width << std::endl;
      this->is_write = true;
      this->info_write = false;
      this->f_info.close();
    }
  }
  void DataCollector::SdfParse(const sdf::ElementPtr _sdf, std::string &camera_sub, std::string &event_sub, std::string &output_dir, std::string &label_dir)
  {
    // parse the sdf files
    if (_sdf->HasElement("camera_sub"))
      camera_sub = _sdf->GetElement("camera_sub")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a camera_sub." << std::endl;
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
