# cyberdog_manager design document


## Overview
The cyberdog_manager serves as the top-level manager of the system, responsible for receiving commands to call upon robot capabilities and for managing the robot's state.
## Architecture
## Framework Design
![](./image/cyberdog_manager/cyberdog_manager_architecture_en.svg)
## Data Flow
![](./image/cyberdog_manager/cyberdog_manager_data_flow_en.svg)
## Main management module functions
### Battery Management  (BatteryCapacityInfo)
1. Manage and control all LED lighting effects related to power consumption
   - non-charging state
     - Battery ≤20%: eye lights red circular zoom, headlights and tail lights red meteor light effect
     - 20%<Battery<80%: eye light cyan circle zoom, headlight and tail light cyan meteor light effect
     -Battery ≥80%: eye lights blue circular zoom, headlights and tail lights blue meteor light effect

   - charging
     - Battery ≤20%: the red eye lights zoom in and out, and the red headlights and tail lights light up one by one
     - 20%<Battery<80: The blue eye lights zoom in and out, and the headlights and tail lights turn on the blue lights one by one
     -Battery ≥80%: the blue eye lights zoom in and out, and the blue headlights and tail lights light up one by one
2. Control all voice prompts related to battery

    - The power has dropped to 0%: "The power is 0, shutting down!"
    - The power level drops to ≤5%: "The power level is less than 5%, the battery is about to run out, please charge it as soon as possible!"
    - Battery down to ≤20%: "Battery is less than 20%, some functions are limited!"
    - Battery down to ≤30: "Battery is less than 30%, please charge as soon as possible!"

3. Control the state machine switching related to power

    - Battery down to 0%: state machine switches to TearDown
    - The battery power drops to ≤5%: the state machine switches to LowPower
    - The battery power drops to ≤20%: the state machine switches to Protect
### Power Consumption management (PowerConsumptionInfo)
1. Interface realization of shutdown, restart, low power entry/exit, motor up/down
2. Enter low power consumption according to the change of motion state: the cyberdog gets down for more than 30s to enter low power consumption (need to turn on the low power consumption switch)

3. Tips for entering/exiting low-power LED lighting effects

      - Enter low power consumption: tail light strip goes off

      - Exit low power consumption: tail light with sky blue slow breathing light effect
### State Machine Switching (MachineStateSwitchContext)
1. State machine configuration management entry
2. Switch the status of the whole machine according to the real-time power
3. Implementation of low power enable/disable interface
4. Associate with CyberdogMchine, switch the status of each module synchronously
### Touch management（TouchInfo）
1. Double-click the Touch: the voice will announce the current battery level
2. Double-tap Touch: exit low power mode
3. Long press Touch: open network distribution function
### Audio management（AudioInfo）
1. Audio initialization and self-test
2. Voice broadcast of machine self-inspection status
3. Implementation of the voice broadcast interface for the error status of each component
### LED management（LedInfo）
Realization of all LED lighting effects related to power consumption

### Account management （AccountInfo）
Interface implementation for adding, deleting, modifying and checking family member accounts

### Error context management（ErrorContxt）
Node Error Status Recording and Publishing

### Heart beat management  （HeartContext）
Cyberdog's heartbeat management includes the following modules:

audio, device, sensor, motion_manager, algorithm_manager

###  Query managemnet（QueryInfo）
Query and report device sn code, ota version, motor temperature, volume, power, motion, low power consumption and other data to the app.

### Ready management（ReadyInfo）
1. Publish self-test and state machine status
2. Publish APP connection status
3. After booting up, connect to the APP first, and realize the interface of controlling the cyberdog to stand

