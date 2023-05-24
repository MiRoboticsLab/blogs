# <center>connector 设计文档</center>

## 1. 概述
仿生机器人快速连接网络及目标设备（下文简称快连）场景主要有以下特性：
1. 仿生机器人处于WiFi环境中；
    * WiFi可以是第三方路由的WiFi；
    * WiFi也可以是APP所在设备外放的热点。
2. App所在设备已连接上目标WiFi；
3. App发起的使得仿生机器人连接目标WiFi；
4. 仿生机器人连接目标WiFi成功后再连接目标APP所在设备；
5. 最终实现快速和目标WiFi及设备建立连接。

## 2. 设计
### 2.1. 功能设计

快连功能主要有两个：
1. 配网功能：通过长按touch来实现开关；
2. 自动连接：通过系统自带的重连机制实现。

#### 2.2 模块设计

<center>

![](./image/connector/connector_module.svg)

</center>

如上图所示，仿生机器人快速连接网络及目标设备架构组成及各组成部分主要功能如下：
1. touch：提供机器人头部触摸信号。
2. led：提供展示机器人头部及身体上灯效功能。
3. audio：提供机器人语音播报功能。
4. camera：提供视野内图像数据。
5. QRreader：提供二维码识别功能。
6. WiFi：提供 linux 系统的目标WiFi连接功能。
7. App：提供 携带WiFi信息的 二维码生成功能。

## 3. 手动联网
### 3.1 联网及APP
```
ros2 service call /`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"`/connect protocol/srv/Connector "{wifi_name: 'wifi_name',wifi_password: 'wifi_password',provider_ip: 'app_ip'}"
```
### 3.2 仅联网
```
sudo nmcli dev wifi connect <wifi_name> password <wifi_password> ifname wlan0
```
### 3.3 其他操作
```
# 查看连接过的WiFi列表
nmcli connection | grep wifi
# 删除连接过的WiFi列表中指定WiFi连接
sudo nmcli connection delete 'ssid' 或者 'uuid'
```
