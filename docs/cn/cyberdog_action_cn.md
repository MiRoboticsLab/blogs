# Cyberdog_action 设计文档

## 1. 功能概述

目前手势识别模块提供一个service和一个topic，service用于激活手势识别功能，激活后，在规定的timeout时间内，该节点会发布检测到的手势动作id的topic。

## 2. 架构设计

<center>

 ![avatar](./image/cyberdog_action/cyberdog_action_flow.png)
 cyberdog_action 功能架构图

</center>


- app：为调用gesture_action的功能模块，例如可视化编程。
- cyberdog_action提供ros2 service接口用于控制手势动作识别的开启、关闭和timeout。
- 开启处理流程： cyberdog_action收到开启算法的request后，需要依次打开camera、加载模型、逐帧推理以及将识别结果已ros2 topic的形式发布出去。
- 关闭处理流程： cyberdog_action收到关闭算法的request或者请求时间timeout，需要依次关闭camera、卸载模型。

## 3. 接口文件描述

- 协议文件 ：

protocol/srv/GestureActionControl.srv

protocol/msg/GestureActionResult.msg



- 配置文件

位置：/params/toml_config/interaction/gesture_action.toml

```Makefile
device = 0                  # cuda device id 
queue_size = 5              # 预测结果队列长度
frames_number = 3           # 相同预测结果帧数阈值
refine_output = true        # 是否采用调优策略
softmax_thres = 0.5         # softmax阈值
fps = 30                    # 相机帧率,当前ai相机sdk最高支持30fps
engine_path = "/SDCARD/vision/gesture_action/gesture_action.trt" # 引擎路径
is_specified = false        # 是否指定下载fds中指定版本模型，若为false，下载最新版本模型。
                            # 若为true，下载最新版本模型
version = "0.0"             # 模型的指定版本
```



## 4. 一些测试用到的cmd

```C%2B%2B
//开启动作识别功能，timeout设置为60s
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 0,timeout: 100}"
// 关闭动作识别功能
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 1,timeout: 100}

```

## 5. 模型文件位置

- 算法模型放置位置：/SDCARD/vision/gesture_action/gesture_action.trt。

## 6. 引用
- 仓库中所使用的模型来至于<https://github.com/mit-han-lab/temporal-shift-module>，感谢其很棒的工作。
