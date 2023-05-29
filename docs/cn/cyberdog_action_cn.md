# Cyberdog_action 设计文档

## 1. 功能概述

目前手势识别模块提供一个service和一个topic，service用于激活手势识别功能，激活后，在规定的timeout时间内，该节点会发布检测到的手势动作id的topic。源代码位置：
https://github.com/MiRoboticsLab/interaction/tree/rolling/cyberdog_actionhttps://github.com/MiRoboticsLab/interaction/tree/rolling/cyberdog_action

## 2. 架构设计

<center>

 ![avatar](./image/cyberdog_action/cyberdog_action_flow.png)
 cyberdog_action 功能架构图

</center>


- app：为调用gesture_action的功能模块，例如可视化编程。
- cyberdog_action提供ros2 service接口用于控制手势动作识别的开启、关闭和timeout。
- 开启处理流程： cyberdog_action收到开启算法的request后，需要依次打开camera、加载模型、逐帧推理以及将识别结果已ros2 topic的形式发布出去。
- 关闭处理流程： cyberdog_action收到关闭算法的request或者请求时间timeout，需要依次关闭camera、卸载模型。


## 3. API接口
```Makefile
- bool ReadTomlConfig():从配置文件读取相关参数，例如帧率、模型路径、版本等。
- int activation_function_softmax(const _Tp * src, _Tp * dst, int length, int & max_index);:对模型的分类结果进行softmax操作。
-   void Gesture_Action_Rec_Fun(
    const std::shared_ptr<GestureActionSrv::Request> request,
    std::shared_ptr<GestureActionSrv::Response> response):处理client的请求，根据请求的参数打开算法一段时间或者立即关闭算法。
- void CameraSignalCallback(const CameraMsg::SharedPtr msg):camera图像数据topic的回调函数。
- void Camera_Operate():图像数据处理线程，包含更新推理模型、加载模型、cuda内存初始化以及控制算法持续运行时间，终止算法等操作。
- void Inference_Operate():模型推理与优化后处理部分。
- uint8_t ControlCamera(uint8_t _action):打开和关闭camera数据流。
- int doInference():模型推理
- int process_history(int max_index):通过对多帧数据进行累计判定，提高动作识别的正确率。
- int proprecess(Gesture_ori max_index):过滤容易识别错的手势动作。
- bool LoadEngineIntoCuda():加载tensorrt模型。
- bool DestroyCuda():释放gpu内存和上下文资源
- bool cudaMemoryPro():处理cuda内存以及推理所需要上下文资源。
- void WifiSignalCallback(const WifiMsg::SharedPtr msg):WIFI信号监测的回调函数，用于在有网络的情况下，进行模型在线更新。
```
## 4. 接口文件描述

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



## 5. 一些测试用到的cmd

```C%2B%2B
//开启动作识别功能，timeout设置为60s
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 0,timeout: 100}"
// 关闭动作识别功能
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 1,timeout: 100}

```

## 6. 模型文件位置

- 算法模型放置位置：/SDCARD/vision/gesture_action/gesture_action.trt。

## 7. 引用
- 仓库中所使用的模型来至于<https://github.com/mit-han-lab/temporal-shift-module>，感谢其很棒的工作。
