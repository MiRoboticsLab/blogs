# cyberdog_vision

## Module Introduction

This is a function package based on ROS2. Cyberdog_vision mainly used for AI algorithm scheduling management and AI algorithm inference. The included AI algorithms are face recognition with age and emotion, face registration, body detection, ReID, gesture recognition, keypoints detection, auto track.

## Module Architecture

![](./image/cyberdog_vision/vision_arch_en.png)



- Sensor layer: Hardware devices that provide source data
- Camera layer: Acquire streams based on hardware devices and perform business-related processing
- AI layger: Algorithm scheduling management and algorithm inference based on AI algorithm and business requirements
- Tracking layer: Estimate the target pose of pedestrians and anything else selected by user based on the results of AI algorithm

Remark: The red dotted box is the planning module or the module that changes due to the change of the plan.

## Workflow

![](./image/cyberdog_vision/vision_workflow_en.png)

- module input: RGB image
- module output: AI algorithm processing results
- status pub: selecting, tracking
