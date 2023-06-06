# cyberdog_mivins

## Module Introduction

cyberdog_mivins is used in functions such as visual mapping, visual localization, and visual following, providing real-time output of the pose TF service for quadruped robots. Developed based on open-source projects SVO and VINS-FUSION, it uses multi-sensor data fusion positioning, including cameras, IMUs, and quadruped odometry.

## Module Architecture

![cyberdog_mivins_architecture](./image/cyberdog_mivins/cyberdog_mivins_architecture.jpg)

The cyberdog_mivins localization algorithm includes modules for data input, front-end tracking, front-end mapping, back-end optimization, and output.

## Service Process

### mapping service process
![mivins_mapping_process](./image/cyberdog_mivins/mivins_mapping_process.jpg)

### localization service process

![mivins_localization_process](./image/cyberdog_mivins/mivins_localization_process.jpg)

### following localization function process
![mivins_following_process](./image/cyberdog_mivins/mivins_following_process.jpg)
