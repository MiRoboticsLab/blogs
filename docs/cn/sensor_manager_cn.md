# sensor_manager 

##  概述

``sensor_manager`` 是``cyberdog_tof``、``cyberdog_ultrasonic``、``cyberdog_lidar``等功能模块的管理模块;为各模块提供状态机管理、服务回调、消息发布的能力。各模块以ros plugin形式加载到device_manager。

##  软件设计

<center>

 ![avatar](./image/sensor_manager/sensor_manager.png)

</center>

##  功能设计

### 模块加载
- [ROS plugin](https://github.com/ros2/ros2_documentation/blob/galactic/source/Tutorials/Beginner-Client-Libraries/Pluginlib.rst)

- ``sensor_manager``使用 ``pluginlib::ClassLoader``加载``cyberdog_tof``等各模块。

```
  // 加载过程参考ros样例
pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle"); 
```

### 状态机管理
- 状态机管控各外设模块，如:控制``cyberdog_tof``等模块的“低功耗”和“激活”状态切换；

- ``sensor_manager``继承``cyberdog::machine::MachineActuator``，客户端可通过``cyberdog::machine::MachineActuator``提供的服务接口，控制``sensor_manager``状态切换。在状态机内部，对已加载的各模块依次进行对应状态的控制。

- 状态机详细介绍参考: [状态机设计](/cn/cyberdog_machine_cn.md)

### Topic&Service

- ``sensor_manager``提供的ros message接口，客户端可订阅传感器数据topic获取传感器数据。
- ``sensor_manager``提供的ros service接口，客户端可通过传感器控制服务控制传感器状态。
- ROS接口: 
  - ``protocol::msg::GpsPayload``:GPS消息
  - ``ScanMsg``：雷达消息
  - ``sensor_msgs::msg::Range``：超声消息
  - ``protocol::msg::HeadTofPayload``：头部Tof消息
  - ``protocol::msg::RearTofPayload``：尾部Tof消息
  - ``protocol::srv::SensorOperation``：传感器控制服务

### 模块插件
- [GPS模块](/cn/cyberdog_gps_cn.md)
- [TOF模块](/cn/cyberdog_tof_cn.md)
- [雷达模块](/cn/cyberdog_lidar_cn.md)
- [超声模块](/cn/cyberdog_ultrasonic_cn.md)

## 调试命令
  - 获取sensor_manager状态机服务：
  ```
  ros2 topic list | grep sensor_manager
  ```

  - 状态机切换(切换到“Active”状态)：

  ```
  ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/sensor_managermachine_service  protocol/srv/FsMachine  "{target_state: "Active"}"
  ```
