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

      std::string event_sub, camera_sub;
      DataCollector::SdfParse(_sdf, camera_sub, event_sub, output_dir_);
      DataCollector::dirCheck();

      this->f_pose.open(output_dir_ + "/pose.csv");
      this->f_events.open(output_dir_ + "/events.csv");
      this->f_info.open(output_dir_ + "/config.yaml");

      ROS_INFO_STREAM("[DataCollector] Subscribing to: " << camera_sub << " and " << event_sub);
      this->image_sub_ = this->node_handle_.subscribe(camera_sub + "/image_raw", 100, &DataCollector::ImageCallback, this);
      this->event_sub_ = this->node_handle_.subscribe(event_sub + "/events", 100, &DataCollector::EventCallback, this);
      this->info_cam_sub_ = this->node_handle_.subscribe(camera_sub + "/camera_info", 100, &DataCollector::CameraInfoCallback, this);
      this->info_evt_sub_ = this->node_handle_.subscribe(event_sub + "/camera_info", 100, &DataCollector::EventInfoCallback, this);
      // write the headlines
      this->f_events << "x,y,t,p" << std::endl;
      model_ = _model;
      this->f_pose << "t,x,y,z,qx,qy,qz,qw" << std::endl;
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
    auto pos = model_->WorldPose().Pos();
    auto ori = model_->WorldPose().Rot();
    auto time = ros::Time::now();
    this->f_pose << time.toSec() << "," << pos.X() << "," << pos.Y() << "," << pos.Z() << "," << ori.X() << "," << ori.Y() << "," << ori.Z() << "," << ori.W() << std::endl;
    // model_->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
  }
  void DataCollector::EventCallback(const dvs_msgs::EventArray::ConstPtr &event_msgs)
  {
    // 从消息中提取事件信息
    auto events = event_msgs->events;
    for (const auto &event : events)
    {
      this->f_events << event.x << ',' << event.y << ',' << event.ts.toSec() << ',' << static_cast<int>(event.polarity) << std::endl;
    }
  }
  void DataCollector::ImageCallback(const sensor_msgs::Image::ConstPtr &image_msgs)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;

      std::string name = output_dir_ + "/images/" + std::to_string(image_msgs->header.stamp.toSec()) + ".png";
      cv::imwrite(name, image);
      if (event_write_ == false && camera_write_ == false)
      {
        camera_write_ = true;
      }
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
  }
  void DataCollector::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs)
  {
    // 从消息中提取相机信息
    if (event_write_ == false && camera_write_ == true)
    {
      this->f_info << "camera:" << std::endl;
      char camera_yaml = ' ';
      this->f_info << camera_yaml << "P: [";
      for (const auto &p : info_msgs->P)
      {
        this->f_info << p << ",";
      }
      this->f_info << "]" << std::endl;

      this->f_info << camera_yaml << "height: " << info_msgs->height << std::endl;
      this->f_info << camera_yaml << "width: " << info_msgs->width << std::endl;
      event_write_ = true;
      camera_write_ = false;
      info_cam_sub_.shutdown();
    }
  }

  void DataCollector::EventInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs)
  {
    if (event_write_ == true && camera_write_ == false)
    {
      this->f_info << "event_camera:" << std::endl;
      char camera_yaml = ' ';
      this->f_info << camera_yaml << "P: [";
      for (const auto &p : info_msgs->P)
      {
        this->f_info << p << ",";
      }
      this->f_info << "]" << std::endl;

      this->f_info << camera_yaml << "height: " << info_msgs->height << std::endl;
      this->f_info << camera_yaml << "width: " << info_msgs->width << std::endl;
      event_write_ = true;
      camera_write_ = true;
      this->f_info.close();
      info_evt_sub_.shutdown();
    }
  }
  void DataCollector::SdfParse(const sdf::ElementPtr _sdf, std::string &camera_sub, std::string &event_sub, std::string &output_dir)
  {
    // parse the sdf files
    if (_sdf->HasElement("cameraTopic"))
      camera_sub = _sdf->GetElement("cameraTopic")->Get<std::string>();
    else
    {
      ROS_ERROR("[DataCollector] Please specify a camera topic.");
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("eventTopic"))
      event_sub = _sdf->GetElement("eventTopic")->Get<std::string>();
    else
    {
      ROS_ERROR("[DataCollector] Please specify a event topic.");
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("outputDirectory"))
      output_dir = _sdf->GetElement("outputDirectory")->Get<std::string>();
    else
    {
      ROS_WARN("[DataCollector] Implicit output directory, default is set '~/.ros/data'.");
      output_dir = "data";
    }
  }
  void DataCollector::dirCheck()
  {
    if (!std::filesystem::exists(output_dir_))
    {
      std::filesystem::create_directories(output_dir_);
    }
    else
    {
      ROS_INFO("%s exists, skipping creating.", output_dir_.c_str());
    }
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
    output_dir_ = output_dir_ + "/" + ss.str();
    std::filesystem::create_directories(output_dir_);
    std::filesystem::create_directories(output_dir_ + "/images");
  }
}
