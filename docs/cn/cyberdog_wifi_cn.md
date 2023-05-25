# WiFi功能设计文档

## 概述

cyberdog_wifi是cyberdog控制WiFi连接的ROS接口节点，其它ROS节点可通过调用它来连接指定的路由器和热点。

## 系统结构

1. cyberdog_wifi功能包：生成cyberdog_wifi节点，用于提供连接指定ssid的路由器、发布WiFi连接状态信息
    - 连接WiFi服务器：用于连接指定ssid的WiFi
    - WiFi信息发布器：用于在连接状态向外广播当前的连接信息，包括ssid、信号强度和本设备IP

1. connector节点：获取WiFi连接状态，发起连接请求  
![structure](./image/cyberdog_wifi/cyberdog_wifi_cn_structure.svg)  

## 运行流程

### 连接流程
![connect](./image/cyberdog_wifi/cyberdog_wifi_request_cn.svg)
### 连接信息发布流程
![info](./image/cyberdog_wifi/cyberdog_wifi_info_cn.svg)