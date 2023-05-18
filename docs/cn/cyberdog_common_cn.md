# cyberdog_common设计文档

## 版本

| 编写 | change                  | version | date       |
| ---- | ----------------------- | ------- | ---------- |
| 刘凯 | 梳理cyberdog_common功能 | 1.0     | 2023.05.22 |

## 概述

cyberdog_common对log、json、toml、lcm、fds、message queque、semaphore进行了封装，在其它业务可以引用。

## 设计

### 框架图

![](./image/cyberdog_common/framework.svg)

### 功能

- CyberdogLogger是对ros rcl logger进行了封装，简化丰富了日志输出接口宏定义

- CyberdogToml是对toml操作进行了封装，还包括了文件存储

- CyberdogJson是对json操作进行了封装，还包括了字符串转换、文件存储

- LcmClient、LcmServer是对lcm的收发进行了cs封装

- CyberdogFDS是fds功能封装，主要是从fds服务器进行拉取文件

- Semaphore是condition_variable信号量封装类

- MsgDeque是双向对列封装，消息阻塞式出队列