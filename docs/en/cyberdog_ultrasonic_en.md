# Cyberdog_ultrasonic Design

##  Overview

``cyberdog_ultrasonic`` is a sensor plug-in associated with the ultrasonic ranging sensor. This plug-in provides the necessary API interface for controlling the sensor, and converts the collected ultrasonic data into a ros message format and feeds it back to the client side..

## Software Design

#### Software framework


<center>

 ![avatar](./image/cyberdog_ultrasonic/cyberdog_ultrasonic.png)

</center>


#### Data stream on


<center>

 ![avatar](./image/cyberdog_ultrasonic/cyberdog_ultrasonic_open_flow.png)

</center>


#### Data stream closed


<center>

 ![avatar](./image/cyberdog_ultrasonic/cyberdog_ultrasonic_close_flow.png)

</center>


## Functional design

- Flexible configuration of sensor number, message source, command id , etc. through configuration file
- Provide basic capability interfaces such as sensor enable, shutdown, and self-test

## Configuration files

- Source path: ``bridges/params/toml_config/sensors``
- Installed path：``/opt/ros2/cyberdog/share/params/toml_config/sensors``
- Configuration file:
  - ``utrasonic_config.toml``: used to configure the number of sensors and the actual configuration file
  - ``ultrasonic.toml``: used to configure the sensor of the head
- Main configuration description:
  - ``config_files``: the program gives the corresponding sensor entity according to the array member instance
  - ``Protocol``: communication protocol, default is ``CAN``.
  - ``can_interface``: message channel for CAN communication, configurable     ``can0``, ``can1``
  - ``Array``: data packet message reception configuration
    - ``array_name``: data packet name
    - ``can_package_num``: the number of CAN data frames in the data packet
    - ``can_id``: data packet, CAN data frame ``CAN ID``

  - ``cmd``: command packet message sending configuration
    - ``cmd_name``: instruction package name
    - ``can_id``: instruction package, CAN data frame ``CAN ID``
    - ``ctrl_len``: the data length of the instruction data frame in the CAN package
    - ``ctrl_data``: data default value of instruction data frame in CAN package

##  API interface
- ``Init (bool simulator)``: initialize configuration
  - ``Simulator = true``: configure to emulate mode
- ``Open () ``: turn on the sensor
- ``Start ()``: enable sensor
- ``Stop ()``: stop sensor
- ``Close ()``: turn off the sensor
- ``SelfCheck ()``: sensor self-check
- ``LowPowerOn ()``: enter low power mode
- ``LowPowerOff ()``: exit low power mode
- ``SetSinglePayloadCallback(std::function<void(std::shared_ptr<protocol::msg::Range> payload)> cb)``：set callback function for message.
