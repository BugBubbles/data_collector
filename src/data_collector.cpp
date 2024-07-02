#include <cmath>
#include "data_collector.hpp"

namespace gazebo
{

  // 注册这个插件
  GZ_REGISTER_MODEL_PLUGIN(GravityPlugin)
  void GravityPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->pose_sub_ = this->node_handle_.subscribe("/mavros/local_position/pose", 100, &DvsPlugin::MotionCallback, this);
    this->image_sub_ = this->node_handle_.subscribe("/dvs_rendering", 100, &DvsPlugin::MotionCallback, this);
    this->event_sub_ = this->node_handle_.subscribe("/iris/usb_cam/image_raw", 100, &DvsPlugin::MotionCallback, this);
    // 临时订阅一下相机的参数，并写入至输出流中
    ros::Subscriber cam_info_sub_ = this->node_handle_.subscribe("/iris/usb_cam/camera_info", 100, &DvsPlugin::MotionCallback, this);

    
    this->model = _model;
    // 订阅更新事件
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GravityPlugin::OnUpdate, this));
  }

  // 在每个仿真步骤中调用
  void GravityPlugin::OnUpdate()
  {
    // 获取模型的位置
    auto pos = this->model->WorldPose().Pos();
    this->model->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
  }
}