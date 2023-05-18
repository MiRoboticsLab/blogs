# cyberdog_train design document

## 1 Overview
This document aims to outline the design requirements and details related to the integration of the Xiao Ai training plan. The main goal of this feature is to allow Cyberdog to recognize custom voice commands and trigger corresponding actions through the Xiao Ai training plan interface.
The training plan scenario requires the following conditions:
1. The robot must be in a WiFi-enabled environment that has access to the Internet.
2. The app must establish a connection with the bionic robot.

## 2 Functionality
1. When Xiao Ai recognizes the training trigger word, Cyberdog should respond accordingly, including but not limited to voice, movement, and lighting effects.
2. Users should have the ability to customize training words and their corresponding responses.
3. The system will retain a limited number of training words (currently 6), and users will typically be able to use them without the ability to query or modify them, though exceptions may be made in certain circumstances.

## 3 Architecture
![](./image/cyberdog_train/cyberdog_train_en_1.svg)
## 4 Initialization and Trigger Process
### 4.1 Initialization
![](./image/cyberdog_train/cyberdog_train_en_2.svg)
### 4.2 Trigger
![](./image/cyberdog_train/cyberdog_train_en_3.svg)
## 5 Training Plan Grouping
### 5.1 System reserved fields
  System reserved fields cannot be queried, deleted, or modified by users.
  ```json 
  {"伸懒腰":["motion", "1"]}
  {"伸左手":["motion", "2"]}
  {"伸右手":["motion", "3"]} 
  ```
### 5.2 Visual reserved fields
  Visual reserved fields cannot be queried, deleted, or modified by users.
  ```json 
  {"终止任务":["vp_task", "shutdown"]}
  {"暂停任务":["vp_task", "suspent"]}
  {"继续任务":["vp_task", "recover"]}
  ```
### 5.3 User-defined fields
  ```json 
  {"trigger1":["type", "value"]}
  {"trigger2":["type", "value"]}
  {"trigger3":["type", "value"]}
  ......
  ```
## 6 Interface
1. Training word publishing interface (basic interface)
   Interface form：ros topic  
   topic name："train_plan_word"  
   message file："protocol/msg/TrainPlan.msg"  
   message content:
   ```json
   string trigger         # 张三、李四、王五    
   string type            # 未定义（undefined)、运动(motion)、语音(audio)、LED(led)、导航(navigation)、跟随(follow)
   string value           # 后空翻
   ```
2. Training word addition interface  
   Interface form：ros service  
   service name："set_train_plan"  
   message file："protocol/msg/SetTrainPlan.msg"  
   message content:
   ```json
   string trigger
   string type
   string value
   ---
   int32 skill_id
   int32 code
   ```
3. Training word deletion interface  
   Interface form：ros service  
   service name："delete_train_plan"  
   message file："protocol/msg/TrainPlan.msg"  
   message content:
   ```json
   string type
   string value
   ---
   int code
   ```
4. Training word modification interface 
   Interface form：ros service  
   service name："modify_train_plan"  
   message file："protocol/msg/SetTrainPlan.msg"  
   message content:
   ```json
   string trigger
   string type
   string value
   ---
   int code
   ```
5. Interface for querying all training words  
   Interface form：ros service  
   service name："query_all_train_plan"  
   message file："protocol/msg/TrainPlanAll.msg"  
   message content:
   ```json
   ---
   int code
   TrainPlan[] training_set
   ```
6. Interface for querying a specific training word  
   Interface form：ros service  
   service name："query_train_plan"  
   message file："protocol/msg/TrainPlan.msg"  
   message content:
   ```json
   string type
   string value
   ---
   int code
   TrainPlan[] training_set
   ```
   