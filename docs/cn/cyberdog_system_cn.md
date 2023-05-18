# cyberdog_system设计文档

## 版本

| 编写 | change                  | version | date       |
| ---- | ----------------------- | ------- | ---------- |
| 刘凯 | 梳理cyberdog_system功能 | 1.0     | 2023.05.22 |

## 概述

cyberdog_system是机器人全局状态码及错误码的封装。

## 功能

封装了CyberdogCode类，各功能包引用返回具体的错误码信息。

定义了各功能模块的模块代码及预至了系统默认错误码。