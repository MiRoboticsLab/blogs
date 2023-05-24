# <center>cyberdog_interactive design document</center>

## 1. Overview

<center>

![](./image/cyberdog_lidar/cyberdog_lidar_scan.png)

</center>

As shown in the figure above, the bionic robot radar driver is mainly used in scenarios where real-time feedback of obstacle distance and distance detection value intensity information in the bionic robot's environment is required.

## 2. Design
### 2.1. Feature design
The workflow of the bionic robot radar driver is mainly as follows:
1. Analyze the radar configuration parameters and configure the radar software and hardware according to the parameters;
2. Initialize the radar;
3. If the initialization is successful, continue to the next step, otherwise exit;
4. Collect raw radar data;
5. If filtering is required, filter the raw radar data (smearing filter) and release it, otherwise release the original data directly;
6. If the program receives the termination signal, exit, otherwise, go to step 4.

### 2.2 Technology architecture

<center>

![](./image/cyberdog_lidar/cyberdog_lidar_module.svg)

</center>

As shown in the figure above, the composition of the bionic robot radar drive architecture and the main functions of each component are as follows:
1. Lidar SDK: Provides radar firmware data collection and analysis functions under Linux;
2. Utils: Provide common tools, such as logs;
3. Radar software and hardware configuration function module: provide radar hardware and software configuration functions;
4. Radar data acquisition function module: provide radar data acquisition and analysis functions;
5. Radar data filtering function module: provide radar data filtering (smearing filter) function;
6. Release radar data: Provide radar data release function.
