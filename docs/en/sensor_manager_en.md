# sensor_manager 

## Overview

``sensor_manager`` is the management module of ``cyberdog_tof``, ``cyberdog_ultrasonic``, ``cyberdog_lidar`` and other functional modules; it provides the ability of find-state machine management, service callback, and message release for each module. Each module is loaded into the device_manager in the form of ros plugin.
## Software design

<center>

 ![avatar](./image/sensor_manager/sensor_manager.png)

</center>

## Functional design

- Finite-state machine controls various peripheral modules, such as: controlling ``low power`` consumption and ``active`` state switching of modules such as ``cyberdog_tof``;
- Provide a message channel for communication and control of each peripheral module for the ros2 platform client side; for example, the client can control the opening and closing of the ``cyberdog_tof`` through the ``protocol:: srv:: SensorOperation`` service, and the subscriber can subscribe to the ``sensor_msgs:: msg:: Range`` message to obtain ultrasonic sensor status information.

### Module loading
- [ROS plugin](https://github.com/ros2/ros2_documentation/blob/galactic/source/Tutorials/Beginner-Client-Libraries/Pluginlib.rst)

- ``sensor_manager`` Use ``pluginlib::ClassLoader``to load ``cyberdog_tof`` and other modules.

```
//Refer to the ros sample for the loading process
pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
```

### Finite-state machine management
- Finite-state machine controls various peripheral modules, such as: controlling "low power consumption" and " active " state switching of modules such as ``cyberdog_tof``;

- ``sensor_manager`` inherits ``Cyberdog::machine::MachineActuator``, the client side can control the ``sensor_manager`` state switching through the service interface provided by ``Cyberdog::machine::MachineActuator``. Inside the final-state machine, the loaded modules are controlled in turn for corresponding states.
- Finite-state machine detailed introduction reference:   [Finite-state machine design](/en/cyberdog_machine_en.md)

### Topic&Service

- ``sensor_manager`` provides a ros message interface, and the client side can subscribe to sensor data topics to obtain sensor data.
- ``sensor_manager`` provides a ros service interface, and the client side can control the sensor state through the sensor control service.
- ROS interface:
  - ``Protocol:: msg:: GpsPayload``: GPS message
  - ``ScanMsg``: Radar message
  - ``sensor_msgs:::: Range``: Ultrasonic message
  - ``Protocol:: msg:: HeadTofPayload``: header Tof message
  - ``Protocol:: msg:: RearTofPayload``: tail Tof message
  - ``Protocol:: srv:: SensorOperation``: Sensor Control Service

### Module plugin

- [GPS Module](/en/cyberdog_gps_en.md)
- [TOF Module](/en/cyberdog_tof_en.md)
- [Radar Module](/en/cyberdog_lidar_en.md )
- [Ultrasound Module](/en/cyberdog_ultrasonic_en.md )

## Debug command

- Get sensor_manager find-state machine service:

  ```
  ros2 topic list | grep sensor_manager
  ```

- Final-state machine toggle (toggle to "Active" state):

  ```
  ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/sensor_managermachine_service  protocol/srv/ FsMachine  "{target_state: "Active"}"
  ```
