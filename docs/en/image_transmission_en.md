# Image_transmission Design

## Overview

image_transmission is a ROS package that transmits video stream to APP

## System Structure

1. APP: Makes WebRTC signalling request, receives video stream, sends stop signal

2. cyberdog_grpc: Transmits signalling messages and stop signal

3. image_transmission: Generates an shared library which is linked by cyberdog_camera node. The library receives signalling messages, notifys cyberdog_camera node to activate streaming raw video data, uses WebRTC peer connection to send vieo stream

4. cyberdog_camera: Receives notification from image_transmission，generates raw image data

![image_trans](./image/image_transmission/image_transmission_en.svg)

## Operation Process

### Activation Process

1. APP sends signlling message through gRPC. cyberdog_grpc node receives the messages and converts them to ROS message and publish with topic "img_trans_signal_in".

2. image_transmission library receives messages from "img_trans_signal_in" topic, calls "camera_service" service to notify cyberdog_camera to start producing raw image data.

3. image_transmission sends response signalling messages through topic "img_trans_signal_out". cyberdog_grpc converts them to gRPC messag and sends to APP.

4. Repeat 1 to 3 several times.

5. image_transmission starts sending video stream to APP through WebRTC.

### Termination Process

- Terminate video streaming from APP:

1. APP sends stop signal to cyberdog_grpc node through gRPC. cyberdog_grpc node  receives the messages and converts it to ROS message and publish with topic "img_trans_signal_in".

2. image_transmission library receives messages from "img_trans_signal_in" topic, destructs WebRTC peer connection and sends response to cyberdog_grpc. cyberdog_grpc converts them to gRPC messag and sends to APP.

3. image_transmission calls "camera_service" service to notify cyberdog_camera to stop producing raw image data.

4. APP stop display the video images.

- Or exit the APP directly:

1. APP disconnect WebRTC connection.

2. image_transmission detects that WebRTC connection state changing, calls "camera_service" service to notify cyberdog_camera to stop producing raw image data.