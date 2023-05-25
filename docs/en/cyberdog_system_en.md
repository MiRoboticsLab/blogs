# cyberdog_system Design Document

# Overview

cyberdog_system is the packaging package of the robot's global status code and error code.

## Global status code

Global status code points: uninitialized, setup, self-test, active, inactive, low power consumption, protection mode, upgrade mode, error status, shutdown status, unknown status. Among them, the state in use: uninitialized, set, active, low power consumption, protection mode, upgrade mode, error state.

```C++
enum class MachineState : uint8_t
{
  MS_Uninitialized = 0,  
  MS_SetUp = 1,        
  MS_TearDown = 2,         // Off state
  MS_SelfCheck = 3,      
  MS_Active = 4,      
  MS_DeActive = 5,        
  MS_Protected = 6,       
  MS_LowPower = 7,         
  MS_OTA = 8,             
  MS_Error = 9,          
  MS_Unkown = 10         
};  // enum class MachineState
```

## Error code

Error code points: global error code and module custom error code. The global error code is a common error code for all modules, mainly including the error code information of the following code blocks. Module custom error code, the module node implements custom error code according to its own business needs.

### Global error code

| 码值 | 说明                                             | 码值 | 说明                                                        |
| ---- | ------------------------------------------------ | ---- | ----------------------------------------------------------- |
| 0    | OK                                               |      |                                                             |
| 1    | Failure not specifically defined, used uniformly | 11   | The status is busy, and the interface is called exclusively |
| 2    | Uninitialized                                    | 12   | Hardware error, use of peripherals and sensors              |
| 3    | The state machine does not allow                 | 13   | Low power                                                   |
| 4    | Module status error                              | 14   |                                                             |
| 5    | Network error                                    | 15   |                                                             |
| 6    | No operation authority                           | 16   |                                                             |
| 7    | Overtime                                         | 17   |                                                             |
| 8    | Command does not support                         | 18   |                                                             |
| 9    | Self check failed                                | 19   |                                                             |
| 10   | The parameter is invalid                         | 20   |                                                             |

# Function

The CyberdogCode class is encapsulated, and each function package references and returns the specific error code information of the node module. By instantiating CyberdogCode, passing in ModuleCode (module code) to associate specific node modules, when an error occurs in the node, set the relevant error code: including global error code and module custom error code, and synchronize the error information to the global management node (cyberdog_manager ).

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
   * @param code reserved key code
   * @return int32_t Code to convert to int value
   */
  int32_t GetKeyCode(KeyCode code)
  {
    return code ==
           KeyCode::kOK ? static_cast<int32_t>(KeyCode::kOK) : static_cast<int32_t>(ModuleImpl_) +
           static_cast<int32_t>(code);
  }

  /**
   * @brief Get module custom result code
   *
   * @param code Custom result code
   * @return int32_t Code to convert to int value
   */
  int32_t GetCode(Code_T code)
  {
    return static_cast<int32_t>(ModuleImpl_) + static_cast<int32_t>(code);
  }
};  // calss CyberdogCode
```

Among them, ModuleCode represents the module code and is used to identify the module. All module codes are as follows:

| manager          | vision      | core        |                |            |                        |                |            |             |
| ---------------- | ----------- | ----------- | -------------- | ---------- | ---------------------- | -------------- | ---------- | ----------- |
| Module           | Code Value  | Description | Module         | Code Value | Description            | Module         | Code Value | Description |
| cyberdog_manager | 100         |             | Follow         | 4100       |                        | Audio-NX       | 5000       |             |
| server           | 200,400,500 |             | AI             | 4200       |                        | Audio-Board    | 5100       |             |
| bridge-grpc      | 300         |             | MapNav         | 4300       |                        | Connector      | 5200       |             |
| bridge-net       | 600         |             | CameraServer   | 4400       |                        | Navigation     | 5300       |             |
| APP              | 700,800     |             |                |            |                        | Transmission   | 5400       |             |
|                  |             |             |                |            |                        | Fence          | 5500       |             |
|                  |             |             |                |            |                        | Aft            | 5600       |             |
|                  |             |             |                |            |                        | OTA            | 5700       |             |
|                  |             |             |                |            |                        | Model          | 5800       |             |
| device           | motion      | Face        | 5900           |            |                        |                |            |             |
| Module           | Code Value  | Description | Module         | Code Value | Description            | Sport          | 7000       |             |
| device_manager   | 1000        |             | motion_manager | 3000       |                        | VP             | 8000       |             |
| LED              | 1100        |             | motion-borad   | 3100       | Motion Board           | unlock_request | 9000       |             |
| Wifi             | 1300        |             | motion-utils   |            |                        |                |            |             |
| BMS              | 1400        |             |                |            |                        |                |            |             |
| Touch            | 1500        |             |                |            |                        |                |            |             |
| bluetooth        | 1600        |             |                |            |                        |                |            |             |
|                  |             |             |                |            |                        |                |            |             |
|                  |             |             |                |            |                        |                |            |             |
| sensor           | bridges     |             |                |            |                        |                |            |             |
| Module           | Code Value  | Description | Module         | Code Value | Description            |                |            |             |
| sensor_manager   | 2000        |             | bes_transmit   | 6000       | back-end communication |                |            |             |
| Lidar            | 2100        |             | cyberdog_grpc  | 6100       | app request error code |                |            |             |
| ToF              | 2200        |             |                |            |                        |                |            |             |
| Ultrasonic       | 2300        |             |                |            |                        |                |            |             |
| GPS              | 2400        |             |                |            |                        |                |            |             |

