# cyberdog_mivins

## Module Introduction

cyberdog_miloc is a visual mapping and localization module based on a trinocular camera system, used for visual mapping and navigation. During the mapping process, it utilizes robot poses output from Minins and images for sparse reconstruction. During the navigation process, it provides the pose located in the map。

## Module Architecture

![cyberdog_miloc_architecture](./image/cyberdog_miloc/cyberdog_miloc_architecture.jpg)

cyberdog_miloc includes modules of sparse reconstruction, re-localization and map manager。

## Service Process

### mapping service process
<img src=./image/cyberdog_miloc/miloc_mapping_process.jpg width=558 height=978 />

### re-localization service process

<img src=./image/cyberdog_miloc/miloc_relocalization_process.jpg width=578 height=978 />
