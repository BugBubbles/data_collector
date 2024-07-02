#include <cmath>
#include "data_collector.hpp"

namespace gazebo
{

  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(DataCollector)
  void DataCollector::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->pose_sub_ = this->node_handle_.subscribe("/mavros/local_position/pose", 100, &DataCollector::PoseCallback, this);
    this->image_sub_ = this->node_handle_.subscribe("/dvs_rendering", 100, &DataCollector::EventCallback, this);
    this->event_sub_ = this->node_handle_.subscribe("/iris/usb_cam/image_raw", 100, &DataCollector::ImageCallback, this);
    // 临时订阅一下相机的参数，并写入至输出流中
    ros::Subscriber cam_info_sub_ = this->node_handle_.subscribe("/iris/usb_cam/camera_info", 100, &DataCollector::ImageInfoCallback, this);

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
    ROS_INFO("[DataCollector] Event: x: %d, y: %d, t: %d, p: %d", events[0].x, events[0].y, events[0].ts.toSec(), events[0].polarity);
  }
  void DataCollector::ImageCallback(const sensor_msgs::Image::ConstPtr &pose_msgs)
  {
    // 从消息中提取图像信息
    auto img = pose_msgs->data;
    ROS_INFO("[DataCollector] Image: size: %d", img.size());
  }
  void DataCollector::ImageInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &info_msgs)
  {
    // 从消息中提取相机参数信息
    auto K = info_msgs->K;
    ROS_INFO("[DataCollector] Camera Info: fx: %f, fy: %f, cx: %f, cy: %f", K[0], K[4], K[2], K[5]);
  }
}