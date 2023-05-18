# State Machine Design

## Design principles

- Use soft encoding for mapping.
- Support expansion.
- There is no constraint relationship between status codes, and constraints are imposed at the business level.

## State Process

### State Transition
![](./image/cyberdog_machine/cyberdog_machine.svg)

### Special Design

1. setup, selfcheck, and teardown are themselves states, and special handling is done in the business layer to require the executor to automatically complete the setup state before initializing the state machine service. Setup includes the configure and init operations；
2. Selfcheck provides special handling for the controller. If there is a problem with this state, the state machine is required to return a switch failure, and the controller will handle the subsequent control processing；
3. Extending is reserved for future expansion. The states shown in the diagram are required by the system, and the specific state descriptions are as follows：



| State      | Start Time & Method  | Transition Process   | Transitioned State       | Remarks |
| --------- | -------------- | -------------- | -------------------------------------------- | ---- |
| SetUp     | automatic startup     | load resources   | ROS communication is ready, state machine service is ready, start sending heartbeat |      |
| TearDown  |                | release resources     | terminate the program lifecycle    |      |
| SekfCheck | controller trigger | perform self-checking instruction   | self-checking result          |      |
| Active    | controller trigger | start execution state   | executable ROS command                 |      |
| DeActive  | controller trigger | close execution state   | refuse to execute ROS command             |      |
| LowPower  | controller trigger | start low power mode | enter low power mode                               |      |
| Protected | controller trigger | start protect mode | enter protect mode                               |      |
| OTA       | controller trigger | start OTA mode    | enter protect mode                                  |      |

## Code framework design

1. The inter-process communication of the state machine is completed using the built-in ROS service, and the node pointer is passed in during initialization；
2. The state is expressed in the string data format；
3. The initialization function will perform a configuration file check and return failure if there is a configuration error；
4. Implement state constraints through configuration files, Such as:

```Makefile
[controller]
actuators = ["test1", "test2"]
states = ["SetUp", "TearDown", "SelfCheck", "Active", "DeActive", "Protected", "LowPower", "OTA", "Error"]
default_time = 100 # milliseconds
default_state = "Uninitialized"


[actuator.test1]
states = ["SetUp", "TearDown", "SelfCheck", "Active", "DeActive", "Protected", "LowPower", "OTA", "Error"]
times = [100, 100, 100, 100, 100, 100, 100, 100, 100]


[actuator.test2]
states = ["SetUp", "TearDown", "SelfCheck", "Active", "DeActive", "Protected", "LowPower", "OTA", "Error"]
times = [100, 100, 100, 100, 100, 100, 100, 100, 100]
```

其中：

- The controller configuration uses 'default' as the default parameter; 'actuators' and 'state' need to correspond to the actual ones, which will affect whether the state machine initialization passes or not.；
- The actuator is the configuration of the executor, and only those added in the controller will be detected；
- The 'states' and 'times' of the actuator are lists of the same size, strictly corresponding to the switching overhead of each state.

### State Machine Controller

***The controller is responsible for switching the state***

1. The executor needs to maintain the 'controller' section of the configuration file；
2. In accordance with C++ conventions, the controller appears as a member of the business code and inheritance is not recommended.
3. The basic API for the controller is as follows:
- Initialization operation, the configuration file has been selected by default；

```PHP
/**
    * @brief initialize controller
    * 1. Detect the validity of the configuration file, see the state machine design document for details of the legality rules
    * 2. Construct a series of management containers of the controller
    * 3. After the initialization work is completed, the runtime no longer depends on the toml file
    *
    * @param node_ptr ros pointer for built-in switching communication with executors
    * @return true The state machine can only be used if the initialization is successful
    * @return false If the initialization fails, the state machine is unavailable, otherwise its behavior is undefined in switching and querying
    */
  bool MachineControllerInit(const std::string & config_file=kConfigFile, rclcpp::Node::SharedPtr node_ptr=nullptr);
```

   - Waiting for successful loading of the executor, now in the 'setup' state

```PHP
/**
    * @brief blocking function, waiting for all executors to connect to the state machine
    *
    * @return true All access is successful
    * @return false timeout
    */
  bool WaitActuatorsSetUp();
```

   - Query operations support obtaining the global dictionary and the status of a single executor

```PHP
/**
    * @brief Get state machine management dictionary, reserved function
    *
    * @param state_map
    */
  void GetStateMap(std::map<std::string, std::string> & state_map)
  

/**
    * @brief Query the status of a specific executor
    * 1. The state is maintained by the controller's map, no dynamic inter-process query is required
    * 2. The legitimacy of the actuator name is managed by the controller configuration, and the state machine dictionary has guaranteed security during initialization detection at this time
    *
    * @param target_actuator query target
    * @param state query result
    * @return true Whether the parameter is legal
    * @return false When the return fails, the reference state may be empty, and the program behavior after its use is undefined
    */
  bool GetState(const std::string & target_actuator, std::string & state);
```

   - Set operations support global settings and settings for specific executors

```PHP
/**
    * @brief Set the state of a specific Actuator
    *
    * @param target_actuator target name
    * @param target_state target state
    * @return true slightly.
    * @return false If the return fails, the target state is the previous state, unaffected
    */
  bool SetState(const std::string & target_actuator, const std::string & target_state)
  

  /**
    * @brief set the global state machine
    *
    * @param target_state target state
    * @return true
    * @return false
    */
  bool SetState(const std::string & target_state);
```

   - Execution process

Strictly follow the state transition diagram and control in the order of 'Init', 'waitsetup', 'selfcheck', 'active'.

   - Integration suggestions

```C%2B%2B
# Default string variable
std::string Uninitialized_V = std::string("Uninitialized");
std::string SetUp_V = std::string("SetUp");
std::string SelfCheck_V = std::string("SelfCheck");
std::string Active_V = std::string("Active");
std::string DeActive_V = std::string("DeActive");
std::string Protected_V = std::string("Protected");
std::string LowPower_V = std::string("LowPower");
std::string OTA_V = std::string("OTA");
std::string Error_V = std::string("Error");
std::string Actuator_1 = std::string("test1");
std::string Actuator_2 = std::string("test2");

# Hold the controller as a member pointer
auto machine_controller_ptr_ = std::make_shared<cyberdog::machine::MachineController>();
```

   - Demo code：https://github.com/MiRoboticsLab/utils/-/blob/dev/cyberdog_machine/test/fs_machine_test_controller.cpp

### The controller is responsible for switching states

***The controller is responsible for switching the state***

1. he executor needs to maintain the state machine configuration file 'actuator' section;
2. In accordance with C++ conventions, the executor is expanded as the parent class of business code through inheritance;
3. The basic API of the executor is as follows:
   - Initialization operation, the configuration file has been selected by default;

```PHP

/**
    * @brief initialize the state machine
    * If the operation fails, subsequent state machine behavior is undefined
    *
    * @param node_ptr
    * @return true
    * @return false
    */
  bool MachineActuatorInit(
    const std::string & config_file = kConfigFile,
    rclcpp::Node::SharedPtr node_ptr = nullptr);
```

   - Register callback functions, which must strictly correspond to the configuration file.

```PHP
/**
    * @brief Register the callback function, you need to register all the state machines supported in the configuration
    *
    * @param state target state
    * @param callback callback function
    */
  void RegisterStateCallback(const std::string & state, std::function<int32_t(void)> callback)；
```

   - After registration, an explicit 'start' operation is required, and the success or failure should be judged. If it fails, the program should exit directly and report an error.
```PHP

/**
    * @brief start the state machine executor
    *
    * @return true If successful, the state machine is running normally
    * @return false return failed, the state machine is unavailable
    */
  bool ActuatorStart();
```

   - Demo code：https://github.com/MiRoboticsLab/utils/-/blob/dev/cyberdog_machine/test/fs_machine_test_actuator1.cpp