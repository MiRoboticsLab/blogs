# cyberdog_manager设计文档


## 1 概述
cyberdog_manager为系统顶层管理器，用于接收机器人能力调用指令，管控机器人状态；
## 2 架构
## 2.1 框架设计
![](./image/cyberdog_manager/cyberdog_manager_architecture_cn.svg)
## 2.2 数据流
![](./image/cyberdog_manager/cyberdog_manager_data_flow_cn.svg)
## 3 主要模块功能设计
### 3.1 电量管理模块（BatteryCapacityInfo）
1. 根据当前电量，调用led展示特定灯效
2. 电量为特定值时，调用audio提示用户剩余电量
### 3.2 功耗模块（PowerConsumptionInfo)
1. 关机、重启、低功耗进入与退出的实现（关机、重启、低功耗都调用系统组提供的接口）
2. 根据运动状态变化进入低功耗
3. 进出低功耗时设置对应led灯效
进入低功耗的顺序为：
切换状态机 --> 进入低功耗
充电状态、OTA状态下默认不进入低功耗
退出低功耗顺序为：
退出低功耗 --> 切换状态机
电量低于5%拒绝唤醒，状态机切换中途拒绝唤醒
### 3.3 状态机切换（MachineStateSwitchContext）
1. 状态机配置管理入口
2. 根据实时电量对整机状态进行切换
3. 低功耗启用/禁用的实现
4. 与CyberdogMchine关联，同步切换各模块状态
### 3.4 touch模块（TouchInfo）
1. touch管理 
### 3.5 audio模块（AudioInfo）
1. audio初始化及自检
2. CyberdogManager自检状态语音播报
### 3.6 led模块（LedInfo）
1. 依据实时电量更新默认灯效
### 3.7 帐号管理模块（AccountInfo）
1. 家庭成员帐号的增、删、查处理
### 3.8 error_context模块（ErrorContxt）
1. node错误状态记录与发布
### 3.9 heart模块（HeartContext）
1. 定时发布心跳数据
### 3.10 query模块（QueryInfo）
1. 查询并向app端上报设备sn码、版本、音量、 电量、运动、wifi等数据。
### 3.11 ready模块（ReadyInfo）
1. 向app端发布自检状态信息

