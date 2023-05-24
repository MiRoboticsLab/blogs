# Cyberdog_ai_sports Design

## 1. Function Overview

- ``cyberdog_ai_sports`` is Keypoints based sport counting detection function.  ``cyberdog_ai_sports`` contains a ROS2 service and a ROS2 topic. The service is used to request the opening of the underlying vision_manager's Keypoints recognition algorithm and Human Body recognition algorithm, and the topic is used to listen to the vision_manager's recognized Keypoints and human body Bounding Box. With the Keypoints identified by the underlying vision_manager, ``cyberdog_ai_sports`` extracts features such as joint angles and relative position changes between bone points based on the positions of adjacent bone points, which are used as the basis for motion recognition to realize the motion counting function.

## 2. Design of sport counts detection software

### 2.1 Software framework

<center>

 ![avatar](./image/cyberdog_ai_sports/cyberdog_ai_sports.png)

</center>

### 2.2 Overall design flow

<center>

 ![avatar](./image/cyberdog_ai_sports/cyberdog_ai_sports_flow.png)

</center>

### 2.3 Detection counting flow (Squat)

<center>

 ![avatar](./image/cyberdog_ai_sports/sport_counts_detection_flow.png)

</center>

## 3. Sport counts function

 ``cyberdog_ai_sports`` counts the number of times the target movement occurs or lasts for a certain period of time, including **push-ups**, *jump jack**, **deep squats**, **high knees**, **sit-ups**, **plank**.

 ### 3.1 ROS protocol

- Source code path: ``bridges/protocol/ros``.
- Sport counts protocol file
  - ``protocol/srv/SportManager.srv``：The service protocol of sport counts allows the client to send motion counting requests.
  - ``sport_manager``: Sport Counts Service Name.
  - ``protocol/msg/SportCountsResult.msg``: The  topic protocol of Sport Counts allows the client to receive motion count results.
  - ``sport_counts_msg``: Sport Counts Topic Name。

 ### 3.2 Configure file
-  ``cyberdog_ai_sports`` performs data fusion and filtering of  Keypoint angles for each frame processed, where the path to the configuration parameters: ``/opt/ros2/cyberdog/share/params/toml_config/interaction/sports.toml``.
   - ``DataFusionNumber``: The size of the data fusion.
   - ``DataFilterNumber``: the size of the data filter.
   - ``ThreMinAngle, ThreMaxAngle, ThreMidAngle1, etc.``: Threshold angle of the count judgment.


## 4. Running ROS2 program for cyberdog_ai_sports

- Running ROS2 program for cyberdog_ai_sports
```
ros2 run cyberdog_ai_sports cyberdog_ai_sports --ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`
```

- Request 10 squat counts within 30 seconds
```
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/sport_manager protocol/srv/SportManager "{sport_type: 1,command: true,counts: 10,timeout: 30}"
```
- Turn off sport counting algorithm
```
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/sport_manager protocol/srv/SportManager "{sport_type: ,command: false,counts: ,timeout: }"
```

## 5. API interface

  - ``Get_Angle_Point(std::vector<XMPoint> & keypoint, const std::string & pos)``: Get the three keypoints of the joint angle from the 17 keypoints.
  - ``Angle_Between_Points(XMPoint p0, XMPoint p1, XMPoint p2)``: Get the angle between the three points.
  - ``Length_Between_Points(XMPoint p0, XMPoint p1)``: Get the distance between two points.
  - ``Angle_Nose(std::vector<XMPoint> & keypoint)``: Calculate the angle between the nose and the left and right ankles.
  - ``Angle_Left_Elbow(std::vector<XMPoint> & keypoint)``: Calculates the angle between the left shoulder - left elbow - left wrist.
  - ``Angle_Left_Knee(std::vector<XMPoint> & keypoint)``: Computes the angle between the left hip-left knee-left ankle.
  - ``Angle_Left_Ankle(std::vector<XMPoint> & keypoint)``: Calculates the angle between left shoulder-left hip-left ankle.
  - ``Angle_Left_Shoulder(std::vector<XMPoint> & keypoint)``: Computes the angle between the left elbow-left shoulder-left hip.
  - ``Angle_Left_Hip(std::vector<XMPoint> & keypoint)``: Calculates the angle of the left shoulder-left span-left knee.
  - ``Angle_Left_PushUps(std::vector<XMPoint> & keypoint)``: Computes the angle between the left lower point of the bounding box - left shoulder - right lower point of the bounding box.
  - ``Angle_Right_Elbow(std::vector<XMPoint> & keypoint)``: Computes the angle between the right shoulder-right elbow-right wrist.
  - ``Angle_Right_Knee(std::vector<XMPoint> & keypoint)``: Computes the angle of the right hip-right knee-right ankle.
  - ``Angle_Right_Ankle(std::vector<XMPoint> & keypoint)``: Computes the angle of the right shoulder-right hip-right ankle.
  - ``Angle_Right_Shoulder(std::vector<XMPoint> & keypoint)``: Computes the angle of the right elbow-right shoulder-right hips.
  - ``Angle_Right_Hip(std::vector<XMPoint> & keypoint)``: Computes the angle of the right shoulder-right straddle-right knee.
  - ``Angle_Right_PushUps(std::vector<XMPoint> & keypoint)``: Computes the angle between the left lower point of the bounding box - right shoulder - right lower point of the bounding box.
  - ``MedianFilter(std::vector<float> & n, int & size)``: Median filter.
  - ``DataFusion(std::vector<float> & num, int size)``: Data fusion.
  - ``SquatCounts(std::vector<std::vector<int>> & num, Squart & sports)``: Squat counts.
  - ``SitUpCounts(std::vector<std::vector<int>> & num, SitUp & sports)``: Sit-up counts.
  - ``HighKneesCounts(std::vector<std::vector<int>> & num, HighKnees & sports)``: HighKnees counts.
  - ``PlankTime(std::vector<std::vector<int>> & num, Plank & sports)``: Plank support time.
  - ``PushUpCounts(std::vector<std::vector<int>> & num, PushUp & sports)``: Pushup counts.
  - ``JumpJackCounts(std::vector<std::vector<int>> & num, JumpJack & sports)``: Jump jack counts.
  - ``Init()``: cyberdog_ai_sports node initialization.
  - ``Run()``: cyberdog_ai_sports node spin.
  - ``ResetAlgo()``: reset cyberdog_ai_sports condition.
  - ``ReadTomlConfig()``: Read the parameters in the configuration file.
  - ``Is_Configure()``: Configure vision_manager's lifecycle.
  - ``Is_Active()``: The vision_manager's lifecycle is activated.
  - ``Is_Deactive()``: The vision_manager's lifecycle is deactivated.
  - ``UpdateStatus()``: Determine if cyberdog_ai_sports count timeout.
  - ``ProcessKeypoint(std::vector<std::vector<XMPoint>> & multHuman,uint32_t & height, uint32_t & width)``: cyberdog_ai_sports processes  keypoints data.
  - ``Ai_Sport_Process_Fun(const std::shared_ptr<SportSrv::Request> request,std::shared_ptr<SportSrv::Response> response)``: cyberdog_ai_ sports handles the service callback function that accepts sport requests.
  - ``Ai_Sport_Result_Callback(VisionMsg msg)``: The cyberdog_ai_sports callback function that handles accepting keypoints.
  - ``Ai_Sport_Request_Load(AlgoManagerRequest sport_counts_request)``: The cyberdog_ai_sports algorithm loads the request configuration.