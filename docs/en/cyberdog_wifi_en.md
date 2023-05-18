# Cyberdog_wifi Design

## Overview

cyberdog_wifi is the ROS interface node for cyberdog to control the WiFi connection, and other ROS nodes can call it to connect to the specified router and hotspot.

## System Structure

1. cyberdog_wifi package: Generates cyberdog_wifi node，which is used to provide a router connected to a specified ssid and publish WiFi connection status information
   1. WiFi connection server: Connecting to specified SSID
   2. WiFi info publisher: Publishing wifi connection status info, including SSID, RSSI, local IP, etc

1. connector node: Acquiring WiFi status info, and activating request  
![structure](./image/cyberdog_wifi/cyberdog_wifi_en_structure.svg)
## Operation Process

### Connection Process
![connect](./image/cyberdog_wifi/cyberdog_wifi_request_en.svg)
### Info Publishing Process
![info](./image/cyberdog_wifi/cyberdog_wifi_info_en.svg)