# cyberdog_parameter设计文档

## 版本

| 编写 | change                     | version | date       |
| ---- | -------------------------- | ------- | ---------- |
| 刘凯 | 梳理cyberdog_parameter功能 | 1.0     | 2023.05.22 |

## 概述

cyberdog_parameter是对toml参数操作封装的类，包括获取与设置。

## 功能

- 将toml参数配置生成so库。

- 通过运行时加载so库从内存获取具体的参数值。