# sensor_manager 

## Overview

``sensor_manager`` is the management module of ``cyberdog_tof``, ``cyberdog_ultrasonic``, ``cyberdog_lidar`` and other functional modules; it provides the ability of final-state machine management, service callback and message release for each module.

## Software design

<center>

 ![avatar](./image/sensor_manager/sensor_manager.png)

</center>

## Functional design

- Finite-state machine controls various peripheral modules, such as: controlling ``low power`` consumption and ``active`` state switching of modules such as ``cyberdog_tof``;
- Provide a message channel for communication and control of each peripheral module for the ros2 platform client side; for example, the client can control the opening and closing of the ``cyberdog_tof`` through the ``protocol:: srv:: SensorOperation`` service, and the subscriber can subscribe to the ``sensor_msgs:: msg:: Range`` message to obtain ultrasonic sensor status information.
