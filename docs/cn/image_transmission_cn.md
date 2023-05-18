# 图传功能设计文档

## 概述

image_transmission是一个用于传输摄像头图像到APP的功能包

## 系统结构

1. app端：发起WebRTC的信令请求，接收WebRTC的视频流，发送停止信号
2. grpc通信：转发app下发的信令和停止信号
3. image_transmission功能包：作为一个so库被cyberdog_camera节点调用，接收信令，通知cyberdog_camera开启发送原始数据，发送WebRTC视频流
4. cyberdog_camera功能包：接收image_transmission的通知，发送图像原始数据

![struct](./image/image_transmission/image_transmission_cn.svg)

## 运行流程

### 启动流程

1. app端发起信令请求，通过grpc接收后转成ros话题"img_trans_signal_in"发送给image_transmission
2. image_transmission收到信令请求后会调用"camera_service"（ros服务）通知cyberdog_camera启动发送图像原始数据
3. image_transmission发送"img_trans_signal_out"话题应答信令，grpc将信令转发给app端
4. 步骤1和3会进行多次
5. 当信令协商成功后image_transmission会将图像原始数据转化为WebRTC视频流发送给app端

### 停止流程

正常停止流程，按app上的停止按钮：

1. app端发送停止信号，grpc转成ros话题"img_trans_signal_in"发送给image_transmission
2. image_transmission收到停止信号后断开WebRTC连接，并发送停止完成信号，用"img_trans_signal_out"话题发送哥grpc，grpc转发给app端
3. image_transmission调用"camera_service"（ros服务）通知cyberdog_camera停止发送图像原始数据
4. app端停止显示

或者直接返回退出app：

1. app端的WebRTC连接会断开
2. image_transmission检测到WebRTC连接会断开会调用"camera_service"（ros服务）通知cyberdog_camera停止发送图像原始数据