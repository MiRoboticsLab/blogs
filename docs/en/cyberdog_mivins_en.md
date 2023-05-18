# cyberdog_mivins

## Module Introduction

cyberdog_mivins is used in functions such as visual mapping, visual localization, and visual following, providing real-time output of the pose TF service for quadruped robots. Developed based on open-source projects SVO and VINS-FUSION, it uses multi-sensor data fusion positioning, including cameras, IMUs, and quadruped odometry.

## Module Architecture

![cyberdog_mivins_architecture](./image/cyberdog_mivins/cyberdog_mivins_architecture.jpg)

The cyberdog_mivins localization algorithm includes modules for data input, front-end tracking, front-end mapping, back-end optimization, and output.

## Service Process

### mapping service process
<img src=./image/cyberdog_mivins/mivins_mapping_process.jpg width=558 height=978 />

### localization service process

<img src=./image/cyberdog_mivins/mivins_localization_process.jpg width=578 height=978 />

### following localization function process
<img src=./image/cyberdog_mivins/mivins_following_process.jpg width=220 height=700 />
