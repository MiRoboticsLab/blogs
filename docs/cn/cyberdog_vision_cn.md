# cyberdog_vision

## 模块简介

cyberdog_vision是一个ROS2功能包，主要用于AI算法调度管理及AI算法推理，包含的AI算法有：人脸识别(包含年龄及表情识别)，人体检测、万物跟踪、骨骼点检测、静态手势识别、行人重识别6类算法。

## 模块架构

![](./image/cyberdog_vision/vision_arch_cn.png)

- Sensor数据层：提供数据源的硬件设备

- Camera应用层：基于硬件设备获取数据流并进行业务相关处理

- AI应用层：基于AI算法及业务需求进行算法调度管理及算法推理

- Tracking业务层：根据AI算法结果进行行人及万物跟随的目标位置估计

备注：红色虚线框为规划模块或方案变更导致变动的模块。

## 处理流程

![](./image/cyberdog_vision/vision_workflow_cn.png)

- 模块输入：RGB图像
- 模块输出：AI算法处理结果
- 状态发布：选框中、跟随中
