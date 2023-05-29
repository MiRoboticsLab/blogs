# cyberdog_system设计文档

# 概述

cyberdog_system是机器人全局状态码及错误码的封装包。

## 全局状态码

全局状态码分：未初始化、设置、自检、活跃、非活跃、低功耗、保护模式、升级模式、错误状态、关机状态、未知状态。其中，在使用的状态：未初始化、设置、活跃、低功耗、保护模式、升级模式、错误状态。

```C++
enum class MachineState : uint8_t
{
  MS_Uninitialized = 0,    // 未初始化
  MS_SetUp = 1,            // 设置
  MS_TearDown = 2,         // 关机状态
  MS_SelfCheck = 3,        // 自检
  MS_Active = 4,           // 活跃
  MS_DeActive = 5,         // 非活跃
  MS_Protected = 6,        // 保护模式
  MS_LowPower = 7,         // 低功耗
  MS_OTA = 8,              // 升级
  MS_Error = 9,            // 错误
  MS_Unkown = 10           // 未知
};  // enum class MachineState
```

## 错误码

错误码分：全局错误码和模块自定义错误码。全局错误码是所有模块公用的错误码，主要包含如下代码块的错误码信息。模块自定义错误码，由模块节点按照自己的业务需求实现自定义错误码。

### 全局错误码

| 码值 | 说明                       | 码值 | 说明                         |
| ---- | -------------------------- | ---- | ---------------------------- |
| 0    | OK                         |      |                              |
| 1    | 无特别定义的失败，统一使用 | 11   | 状态忙碌，独占性调用接口     |
| 2    | 未初始化                   | 12   | 硬件错误，外设、传感器类使用 |
| 3    | 状态机不允许               | 13   | 低电量                       |
| 4    | 模块状态错误               | 14   |                              |
| 5    | 网络错误                   | 15   |                              |
| 6    | 无操作权限                 | 16   |                              |
| 7    | 超时                       | 17   |                              |
| 8    | 指令不支持                 | 18   |                              |
| 9    | 自检失败                   | 19   |                              |
| 10   | 参数不合法                 | 20   |                              |

# 功能

封装了CyberdogCode类，各功能包引用返回节点模块具体的错误码信息。通过实例化CyberdogCode，传入ModuleCode（模块代码）关联具体的节点模块，在节点发生错误时，设置相关错误码：包括全局错误码和模块自定义错误码，把错误信息同步到全局管理节点（cyberdog_manager)。

```C++
template<typename Code_T>
class CyberdogCode final
{
public:
  explicit CyberdogCode(ModuleCode module_code)
  {
    ModuleImpl_ = module_code;
  }
  /**
   * @brief Get the Key Code object
   *
   * @param code 预留关键码
   * @return int32_t 转为int值的代码
   */
  int32_t GetKeyCode(KeyCode code)
  {
    return code ==
           KeyCode::kOK ? static_cast<int32_t>(KeyCode::kOK) : static_cast<int32_t>(ModuleImpl_) +
           static_cast<int32_t>(code);
  }

  /**
   * @brief 获取模块自定义结果码
   *
   * @param code 自定义结果码
   * @return int32_t 转为int值的代码
   */
  int32_t GetCode(Code_T code)
  {
    return static_cast<int32_t>(ModuleImpl_) + static_cast<int32_t>(code);
  }
};  // calss CyberdogCode
```

其中，ModuleCode代表模块代码，用于标识模块。所有模块代码如下表：

| manager          | vision      | core |                |      |                 |                |      |      |
| ---------------- | ----------- | ---- | -------------- | ---- | --------------- | -------------- | ---- | ---- |
| 模块             | 码值        | 说明 | 模块           | 码值 | 说明            | 模块           | 码值 | 说明 |
| cyberdog_manager | 100         |      | Follow         | 4100 |                 | Audio-NX       | 5000 |      |
| server           | 200,400,500 |      | AI             | 4200 |                 | Audio-Board    | 5100 |      |
| bridge-grpc      | 300         |      | MapNav         | 4300 |                 | Connector      | 5200 |      |
| bridge-net       | 600         |      | CameraServer   | 4400 |                 | Navigation     | 5300 |      |
| APP              | 700,800     |      |                |      |                 | Transmission   | 5400 |      |
|                  |             |      |                |      |                 | Fence          | 5500 |      |
|                  |             |      |                |      |                 | Aft            | 5600 |      |
|                  |             |      |                |      |                 | OTA            | 5700 |      |
|                  |             |      |                |      |                 | Model          | 5800 |      |
| device           | motion      | Face | 5900           |      |                 |                |      |      |
| 模块             | 码值        | 说明 | 模块           | 码值 | 说明            | Sport          | 7000 |      |
| device_manager   | 1000        |      | motion_manager | 3000 |                 | VP             | 8000 |      |
| LED              | 1100        |      | motion-borad   | 3100 | 运控板          | unlock_request | 9000 |      |
| Wifi             | 1300        |      | motion-utils   |      |                 |                |      |      |
| BMS              | 1400        |      |                |      |                 |                |      |      |
| Touch            | 1500        |      |                |      |                 |                |      |      |
| bluetooth        | 1600        |      |                |      |                 |                |      |      |
|                  |             |      |                |      |                 |                |      |      |
|                  |             |      |                |      |                 |                |      |      |
| sensor           | bridges     |      |                |      |                 |                |      |      |
| 模块             | 码值        | 说明 | 模块           | 码值 | 说明            |                |      |      |
| sensor_manager   | 2000        |      | bes_transmit   | 6000 | 后端通信        |                |      |      |
| Lidar            | 2100        |      | cyberdog_grpc  | 6100 | app请求的错误码 |                |      |      |
| ToF              | 2200        |      |                |      |                 |                |      |      |
| Ultrasonic       | 2300        |      |                |      |                 |                |      |      |
| GPS              | 2400        |      |                |      |                 |                |      |      |