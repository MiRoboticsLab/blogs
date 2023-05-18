# device_manager

##  概述

``device_manager`` 是``cyberdog_touch``、``cyberdog_uwb``、``cyberdog_bms``等功能模块的管理模块;为各模块提供状态机管理、服务回调、消息发布的能力。

##  软件设计

<center>

 ![avatar](./image/device_manager/device_manager.png)

</center>

##  功能设计

- 状态机管控各外设模块，如:控制``cyberdog_led``等模块的“低功耗”和“激活”状态切换；
- 为ros2平台客户端提供与各外设模块的通信和控制的消息通道；如：client可以通过``protocol::srv::LedExecute``服务控制``cyberdog_led``的灯带变化，subscriber通过订阅``protocol::msg::BmsStatus``消息，获取电池状态信息。
