# 说明
本插件是附加在无人机SDF文件中无人机飞行姿态和位置，主要获取的传感器为：
 - FPV相机
 - [事件相机](https://github.com/BugBubbles/gazebo_dvs_plugin)

此外，本插件的前置协议是Mavlink，并通过Mavlink底层接口直接获取无人机在全局坐标系下的位置和姿态，

## 使用
请在预置有FPV相机和事件相机的无人机SDF模型的`<model>`标签内新增以下内容：
```xml
    <plugin name='data_collector' filename='libdata_collector.so'>
      <enable>true</enable>
      <image_sub>/iris/usb_cam/image_raw</image_sub>
      <event_sub>/dvs/events</event_sub>
      <output_dir>data_collected</output_dir>
      <label_dir>Data/labels/circle</label_dir>
    </plugin>
```
其中`<image_sub>`标签内填写的内容是无人机发布的图像订阅话题，`<event_sub>`填写的是无人机发布的事件流订阅话题（**注意不是事件图像**），`<output_dir>`填写的是保存的数据位置，如果不加根目录符号`/`，则目录将自动保存在以`~/.ros`开头的目录中，`<label_dir>`是用于读取当前已知的陨石坑标注数据的目录。
