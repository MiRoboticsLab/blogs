# cyberdog_common设计文档

## 版本

| 编写 | change                  | version | date       |
| ---- | ----------------------- | ------- | ---------- |
| 刘凯 | 梳理cyberdog_common功能 | 1.0     | 2023.05.22 |

## 一、概述

cyberdog_common对log、json、toml、lcm、fds、message queque、semaphore进行了封装，在其它业务可以引用。

## 二、设计

### 2.1 框架图

![](./image/cyberdog_common/framework.svg)

### 2.2 功能说明

- CyberdogLogger是对ros rcl logger进行了封装，简化丰富了日志输出接口宏定义

- CyberdogToml是对toml操作进行了封装，还包括了文件存储

- CyberdogJson是对json操作进行了封装，还包括了字符串转换、文件存储

- LcmClient、LcmServer是对lcm的收发进行了cs封装

- CyberdogFDS是fds功能封装，主要是从fds服务器进行拉取文件

- Semaphore是condition_variable信号量封装类

- MsgDeque是双向对列封装，消息阻塞式出队列

### 2.3 接口说明

​	CyberdogLogger接口如下：（按从上到下紧急程度依次递增）

​       DEBUG（...）// 可用于打印调试信息

​       INFO（...）    // 可用于打印一般的提示信息

​       WARN（...）  // 可用于打印警告信息

​       ERROR（...） // 可用于打印错误信息     

​       FATAL（...）  // 可用于打印致命错误信息

  使用举例：INFO("hello world %s", name.c_str())



  CyberdogToml接口如下：

​    bool ParseFile()    // 从文件中读取数据，并翻译成toml数据结构

​    bool WriteFile()    // 将toml数据写入文件

​    bool Get()            //  从toml表格数据中，依据键k读取一个值

​    bool Set(toml::value & v, const std::string & k, const T & m)       //   为toml表格数据设置一个值,若该键已经存在，则会覆盖，包括不同类型,若该键不存在，则会添加一个新的键值对

​    bool Set(toml::value & v, const T & m)  //  为toml数组数据设置一个值,   该值会被追加在数组末尾

​    bool Set(toml::value & v, size_t k, const T & m)    //为toml数组数据设置一个值，依据角标序号,若序号已经存在，则会覆盖，包括不同类型, 若值序号不存在，则不会添加，且返回错误



 CyberdogJson接口如下：

​    bool Add()   // 为已创建好的Document添加变量，拥有多种不同数据类型的重载

​    bool Get()    //  从json::Value中读取一个值,拥有多种不同数据类型的重载

​    bool String2Document()     // 反序列化，将字符串转化为json数据结构

​    bool Document2String()     // 序列化，将一个Document转化成string，用于传输.

​    bool Value2Document()     // 将一个json::value 转换为 json::Document, Document具有完备的内存资源，如分配器等.进而可以不依赖其它资源进行数据结构再处理，如Add等操作.

​    bool ReadJsonFromFile()   //  从文件读取数据，并存储到json内存结构中

​    bool WriteJsonToFile()      //   将json数据写入文件, 默认使用pretty格式，即保持相对美观的缩进.



LcmClient、LcmServer接口如下：

   LcmClient(const std::string & name) // 创建一个lcm Client对象

   LcmServer()    //  创建一个lcm Server对象

   Request(const Req & request, Res & response, int mill_time) // 发起一个RPC调用



CyberdogFDS接口如下：

  GetObject()          //获取要下载的bucket

  GetObjectSize()  // 获取要下载内容的大小

  StopDownloading()  // 停止下载



Semaphore接口如下：

  WaitFor(int time)  // 信号等待函数，毫秒级

  Wait()   //信号等待函数，诸塞直到获取消息

  Give()   //换醒信号，唤醒一次

  GiveAll()  // 唤醒信号，唤醒所有等待



MsgDeque接口如下：

  bool EnQueue(const T & t)          // 入队函数，会在队首原址构造，如果有等待出队线程，会唤醒一次

  bool  EnQueueOne(const T & t)  //  入队函数，会在队首原址构造，如果有等待出队线程，会唤醒一次，会保持队列不超过一个的数据，用于click场景

  bool DeQueue(T & t)     //出队函数,   如果队列非空，则消耗掉一个数据，并将该数据引用返回, 如果队列为空，则进入条件等待，直到有数据入队,如果等待状态中发生析构，则会返回失败，此时引用参数不可用，其行为是未定义的

  void Reset()  //   重置队列，此时所有等待函数会得到失败的返回值并解锁