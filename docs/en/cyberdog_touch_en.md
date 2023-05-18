# Touch Design document
# Table of contents
## One. Overview
## Two. Design
- 2.1 Function Division
- 2.2 Module Architecture
- 2.3 Interface Design
- 2.4 Module design
# 1. Overview
The following aims to explain the robot dog touch interaction logic and function mapping.
# 2. Design
## 2.1 Function division
The main functions are as follows:
- Get touch trigger events (double click (0x03), long press (0x07), etc.);
- touch self-test;
## 2.2 Module architecture
The module architecture diagram is as follows:

<center>

 ![avatar](./image/cyberdog_touch/touch architecture diagram.png)

</center>

## 2.3 Interface design
The touch module data stream is as follows:

<center>

 ![avatar](./image/cyberdog_touch/Touch data stream.png)

</center>

The touch module message protocol is as follows:

- Topic name: "touch_status"
- topic message file directory: "Bridges/protocol/ros/MSG/TouchStatus MSG"
- The topic message structure is as follows:
int32 touch_state //touch semaphore, where a value not 0 indicates that touch is triggered
uint64 timestamp // timestamp
- touch_state signal quantity values are as follows:
    #define LPWG_SINGLETAP_DETECTED                      0x01
    #define LPWG_DOUBLETAP_DETECTED                      0x03
    #define LPWG_TOUCHANDHOLD_DETECTED                   0x07
    #define LPWG_CIRCLE_DETECTED                         0x08
    #define LPWG_TRIANGLE_DETECTED                       0x09
    #define LPWG_VEE_DETECTED                            0x0A
    #define LPWG_UNICODE_DETECTED                        0x0B
    #define LPWG_SWIPE_DETECTED                          0x0D
    #define LPWG_SWIPE_DETECTED_UP_CONTINU               0x0E
    #define LPWG_SWIPE_DETECTED_DOWN_CONTINU             0x0F
    #define LPWG_SWIPE_DETECTED_LEFT_CONTINU             0x10
    #define LPWG_SWIPE_DETECTED_RIGHT_CONTINU            0x11
## 2.4 Module design
It is mainly divided into touch_input_event_reader, touch_base, -touch_sensor_handler and touch_plugin modules.
- touch_input_event_reader reads touch events from events.
- touch_base mainly implements the definition of touch module initialization, self-check and other functions;
-touch_sensor_handler mainly implements device file opening, processing and mapping touch events based on epolling mechanism, etc.
- touch_plugin mainly implements touch module initialization, self-check, touch event signal topic release and other functions.
