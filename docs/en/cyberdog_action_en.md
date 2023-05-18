# Cyberdog_action Design

## 1. Functional Overview

At present, the gesture recognition module provides a service and a topic. The service is used to activate the gesture recognition function. After activation, within the specified timeout, the node will publish the topic of the detected gesture action id.

## 2. Architecture Design

<center>

 ![avatar](./image/cyberdog_action/cyberdog_action_flow.png)
 cyberdog_action architecture Diagram


</center>


- app：It is a function module that calls gesture_action,   such as a visual programming module
- cyberdog_action provides ros2 service interface to control the gesture action recognition on, off and timeout。
- Start the processing flow: After cyberdog_action receives the request to start the algorithm, it needs to turn on the camera, load the model, reason frame by frame, and publish the recognition result in the form of ros2 topic
- Closing process: cyberdog_action needs to close the camera and release the model after receiving the request to close the algorithm or the request timeout

## 3. Interface

- Interface File ：

protocol/srv/GestureActionControl.srv
protocol/msg/GestureActionResult.msg

- Configuration File ：

Locate：/params/toml_config/interaction/gesture_action.toml

```Makefile
device = 0                  # cuda device id 
queue_size = 5              # Prediction result queue length
frames_number = 3           # filter threshold
refine_output = true        # refine output or not
softmax_thres = 0.5         # softmax threshold
fps = 30                    # fps
engine_path = "/SDCARD/vision/gesture_action/gesture_action.trt" # model path
is_specified = false        # download the newest model from fds or the special verison
version = "0.0"             # fixed model version
```



## 4. some usefull case

```C%2B%2B
//Turn on the motion recognition function, and set the timeout to 60s
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 0,timeout: 100}"
// Turn off motion recognition

ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/gesture_action_control protocol/srv/GestureActionControl "{command: 1,timeout: 100}

```

## 5. model path

/SDCARD/vision/gesture_action/gesture_action.trt。

## 6. References:

- the model in this repo comes from <https://github.com/mit-han-lab/temporal-shift-module>，thanks to their great work.