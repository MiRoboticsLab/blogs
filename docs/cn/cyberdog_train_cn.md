# cyberdog_train设计文档

## 1 概述
本文旨在说明与小爱训练计划结合相关的设计需求、设计细节等。该功能主要目的是cyberdog通过小爱训练计划接口，实现自定义语音指令词及触发该指令词后机器的对应动作。
训练计划场景主要有以下条件：
1. 仿生机器人处于WiFi环境中，能够访问互联网；
2. App已与仿生机器人建立链接；

## 2 需求
1. 小爱接收训练触发词后，cyberdog能作出响应（包括但不限于语音、动作、灯效等）
2. 用户能够自定义训练词及其响应
3. 系统将保留有限个训练词（目前6个），用户只能使用，不得查询、修改。

## 3 架构
![](./image/cyberdog_train/cyberdog_train_cn_1.png)
## 4 初始化及触发流程
### 4.1 初始化流程
![](./image/cyberdog_train/cyberdog_train_cn_1.png)
### 4.2 触发流程
![](./image/cyberdog_train/cyberdog_train_cn_1.png)
## 5 训练计划分组
### 5.1 系统保留字段
  系统保留字段不能被用户查询、删除、修改;
  ```json 
  {"伸懒腰":["motion", "1"]}
  {"伸左手":["motion", "2"]}
  {"伸右手":["motion", "3"]} 
  ```
### 5.2 可视化保留字段
  可视化保留字段不能被用户查询、删除、修改;
  ```json 
  {"终止任务":["vp_task", "shutdown"]}
  {"暂停任务":["vp_task", "suspent"]}
  {"继续任务":["vp_task", "recover"]}
  ```
### 5.3 用户自定义字段
  ```json 
  {"trigger1":["type", "value"]}
  {"trigger2":["type", "value"]}
  {"trigger3":["type", "value"]}
  ......
  ```
## 6 接口
1. 训练词发布接口（基础接口）
   接口形式：ros topic  
   接口名字："train_plan_word"  
   消息文件："protocol/msg/TrainPlan.msg"  
   消息内容:
   ```json
   string trigger         # 张三、李四、王五    
   string type            # 未定义（undefined)、运(motion)、语音(audio)、LED(led)、导航(navigation)、跟随(follow)
   string value           # 后空翻
   ```
2. 训练词增加接口  
   接口形式：ros service  
   接口名字："set_train_plan"  
   消息文件："protocol/msg/SetTrainPlan.msg"  
   消息内容:
   ```json
   string trigger
   string type
   string value
   ---
   int32 skill_id
   int32 code
   ```
3. 训练词删除接口  
   接口形式：ros service  
   接口名字："delete_train_plan"  
   消息文件："protocol/msg/TrainPlan.msg"  
   消息内容:
   ```json
   string type
   string value
   ---
   int code
   ```
4. 训练词修改接口  
   接口形式：ros service  
   接口名字："modify_train_plan"  
   消息文件："protocol/msg/SetTrainPlan.msg"  
   消息内容:
   ```json
   string trigger
   string type
   string value
   ---
   int code
   ```
5. 查询所有训练词接口  
   接口形式：ros service  
   接口名字："query_all_train_plan"  
   消息文件："protocol/msg/TrainPlanAll.msg"  
   消息内容:
   ```json
   ---
   int code
   TrainPlan[] training_set
   ```
6. 查询指定训练词接口  
   接口形式：ros service  
   接口名字："query_train_plan"  
   消息文件："protocol/msg/TrainPlan.msg"  
   消息内容:
   ```json
   string type
   string value
   ---
   int code
   TrainPlan[] training_set
   ```
   