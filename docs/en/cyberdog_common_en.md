# cyberdog_common Design Document

## Version

| Author | change                             | version | date       |
| ------ | ---------------------------------- | ------- | ---------- |
| KaiLiu | Sort out cyberdog_common functions | 1.0     | 2023.05.22 |

## One、Overview

cyberdog_common encapsulates log, json, toml, lcm, fds, message queue, and semaphore, which can be referenced in other businesses.

## Two、Design

### 2.1 Frames

![](./image/cyberdog_common/framework.svg)

### 2.2 Function

- CyberdogLogger encapsulates ros rcl logger, simplifies and enriches the macro definition of log output interface

- CyberdogToml encapsulates toml operations and also includes file storage

- CyberdogJson encapsulates json operations, including string conversion and file storage

- LcmClient、LcmServer are cs packages for the sending and receiving of lcm

- CyberdogFDS is an encapsulation of fds functions, mainly to pull files from fds servers

- Semaphore is a condition_variable semaphore encapsulation class

- MsgDeque is a two-way column encapsulation, and the message is blocked out of the queue

### 2.3  Interface Description

The CyberdogLogger interface is as follows: (in increasing order of urgency from top to bottom)

- `DEBUG(...)` // Used for printing debug information
- `INFO(...)` // Used for printing general informational messages
- `WARN(...)` // Used for printing warning messages
- `ERROR(...)` // Used for printing error messages
- `FATAL(...)` // Used for printing fatal error messages

Example usage: `INFO("hello world %s", name.c_str())`



The CyberdogToml interface is as follows:

- `bool ParseFile()` // Reads data from a file and translates it into a toml data structure

- `bool WriteFile()` // Writes toml data into a file

- `bool Get()` // Reads a value from the toml table data based on the key `k`

- `bool Set(toml::value & v, const std::string & k, const T & m)` // Sets a value in the toml table data. If the key already exists, it will be overwritten, including different types. If the key doesn't exist, a new key-value pair will be added.

- `bool Set(toml::value & v, const T & m)` // Sets a value in the toml array data. The value will be appended at the end of the array.

- `bool Set(toml::value & v, size_t k, const T & m)` // Sets a value in the toml array data based on the index `k`. If the index already exists, it will be overwritten, including different types. If the index doesn't exist, an error will be returned.

  

The CyberdogJson interface is as follows:

- `bool Add()` // Adds a variable to an already created Document. Multiple overloaded versions for different data types.

- `bool Get()` // Reads a value from a json::Value. Multiple overloaded versions for different data types.

- `bool String2Document()` // Deserializes, converts a string into a json data structure.

- `bool Document2String()` // Serializes, converts a Document into a string for transmission.

- `bool Value2Document()` // Converts a json::value into a json::Document. The Document has complete memory resources such as allocators, allowing further data structure manipulations like Add operations without relying on other resources.

- `bool ReadJsonFromFile()` // Reads data from a file and stores it into a json memory structure.

- `bool WriteJsonToFile()` // Writes json data into a file using pretty formatting by default, maintaining relative indentation for readability.

  

The LcmClient and LcmServer interfaces are as follows:

- `LcmClient(const std::string & name)` // Creates an LCM client object.

- `LcmServer()` // Creates an LCM server object.

- `Request(const Req & request, Res & response, int mill_time)` // Initiates an RPC call.

  

The CyberdogFDS interface is as follows:

- `GetObject()` // Retrieves the bucket to download.

- `GetObjectSize()` // Retrieves the size of the content to download.

- `StopDownloading()` // Stops the download.

  

The Semaphore interface is as follows:

- `WaitFor(int time)` // Waits for a signal for the specified number of milliseconds.

- `Wait()` // Waits for a signal, blocks until a signal is received.

- `Give()` // Wakes up a signal, wakes up one waiting thread.

- `GiveAll()` // Wakes up all waiting threads.

  

The MsgDeque interface is as follows:

- `bool EnQueue(const T & t)` // Enqueues an element. If there are threads waiting for dequeue, it wakes up one thread.
- `bool EnQueueOne(const T & t)` // Enqueues an element. If there are threads waiting for dequeue, it wakes up one thread. It keeps the queue with at most one data, used in the click scenario.
- `bool DeQueue(T & t)` // Dequeues an element. If the queue is not empty, it consumes one data and returns a reference to that data. If the queue is empty, it enters a conditional wait until data is enqueued. If destruction occurs during the waiting state, it returns failure, and the reference parameter is not usable, with undefined behavior.
- `void Reset()` // Resets the queue. All waiting functions will receive a failure return value and unlock.