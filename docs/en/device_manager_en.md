# device_manager 

## Overview

``device_manager`` is the management module of ``cyberdog_touch``, ``cyberdog_uwb``, ``cyberdog_bms`` and other functional modules; it provides the ability of final-state machine management, service callback and message release for each module.

## Software design

 <center>

 ![avatar](./image/device_manager/device_manager.png)

</center>

## Functional design

- Finite-state machine controls various peripheral modules, such as: control ``low power`` and ``active`` state switching of modules such as ``cyberdog_led``;
- Provide a message channel for communication and control with each peripheral module for the ros2 platform client side; for example, the client can control the light band change of the ``cyberdog_led``` through the ``protocol:: srv:: LedExecute`` service, and the subscriber subscribes to the ``protocol:: msg::B msStatus`` message to obtain battery status information
