/*
Before your deployment, you should first change this definition below:
*/
// #define pose_sub "/mavros/local_position/pose"
// #define event_sub "/dvs_rendering"
// #define image_sub "/iris/usb_cam/image_raw"
// #define camera_info_sub "/iris/usb_cam/camera_info"

#include <cmath>
#include "data_collector.hpp"

namespace gazebo
{

  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(DataCollector)
  void DataCollector::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    if (!_model)
      gzerr << "Invalid sensor pointer." << std::endl;

    std::string pose_sub, event_sub, image_sub, camera_info_sub;
    DataCollector::SdfParse(_sdf, pose_sub, camera_info_sub, image_sub, event_sub, this->output_dir);

    ROS_INFO("[DataCollector] Subscribing to: %s, %s, %s, %s", pose_sub.c_str(), camera_info_sub.c_str(), image_sub.c_str(), event_sub.c_str());
    this->pose_sub_ = this->node_handle_.subscribe(pose_sub, 100, &DataCollector::PoseCallback, this);
    this->image_sub_ = this->node_handle_.subscribe(event_sub, 100, &DataCollector::EventCallback, this);
    this->event_sub_ = this->node_handle_.subscribe(image_sub, 100, &DataCollector::ImageCallback, this);
    // 临时订阅一下相机的参数，并写入至输出流中
    ros::Subscriber cam_info_sub_ = this->node_handle_.subscribe(camera_info_sub, 100, &DataCollector::ImageInfoCallback, this);

    this->model = _model;
    // 订阅更新事件
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DataCollector::OnUpdate, this));
  }

  // 在每个仿真步骤中调用
  void DataCollector::OnUpdate()
  {
    // 获取模型的位置
    auto pos = this->model->WorldPose().Pos();
    this->model->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
  }
  void DataCollector::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msgs)
  {
    // 从消息中提取位置信息
    auto pos = pose_msgs->pose.position;
    ROS_INFO("[DataCollector] Position: x: %f, y: %f, z: %f", pos.x, pos.y, pos.z);

    auto ori = pose_msgs->pose.orientation;
    ROS_INFO("[DataCollector] Orientation: x: %f, y: %f, z: %f, w: %f", ori.x, ori.y, ori.z, ori.w);
  }
  void DataCollector::EventCallback(const dvs_msgs::EventArray::ConstPtr &pose_msgs)
  {
    // 从消息中提取事件信息
    auto events = pose_msgs->events;
    ROS_INFO("[DataCollector] Event: x: %d, y: %d, t: %f, p: %d", events[0].x, events[0].y, events[0].ts.toSec(), events[0].polarity);
  }
  void DataCollector::ImageCallback(const sensor_msgs::Image::ConstPtr &pose_msgs)
  {
    // 从消息中提取图像信息
    auto header = pose_msgs->header;
    ROS_INFO("[DataCollector] Image: squence: %d", header.seq);
  }
  void DataCollector::ImageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs)
  {
    // 从消息中提取相机参数信息
    auto K = info_msgs->K;
    ROS_INFO("[DataCollector] Camera Info: fx: %f, fy: %f, cx: %f, cy: %f", K[0], K[4], K[2], K[5]);
  }
  void DataCollector::SdfParse(const sdf::ElementPtr _sdf, std::string &pose_sub, std::string &camera_info_sub, std::string &image_sub, std::string &event_sub, std::string &output_dir)
  { // parse the sdf files
    if (_sdf->HasElement("pose_sub"))
      pose_sub = _sdf->GetElement("pose_sub")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a pose_sub." << std::endl;
      std::exit(EXIT_FAILURE);
    }
    if (_sdf->HasElement("camera_info_sub"))
      camera_info_sub = _sdf->GetElement("camera_info_sub")->Get<std::string>();
    else
    {
      gzerr << "[DataCollector] Please specify a camera_info_sub." << std::endl;
      std::exit(EXIT_FAILURE);
    }
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
    if (_sdf->HasElement("output_dir"))
      output_dir = _sdf->GetElement("output_dir")->Get<std::string>();
    else
    {
      gzwarn << "[DataCollector] Implicit output directory, default output directory was set '~/.gazebo/data'." << std::endl;
      output_dir = "~/.gazebo/data";
    }
    DataCollector::dir_check(output_dir);
  }
  void DataCollector::dir_check(std::string dir)
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
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    dir = dir + "/" + ss.str();
    std::filesystem::create_directories(dir);
    std::filesystem::create_directories(dir + "/images");
    std::filesystem::create_directories(dir + "/events");
  }
}
