# Audio Design Document

## Version

| Author | change                                           | version | date       |
| ------ | ------------------------------------------------ | ------- | ---------- |
| KaiLiu | Sort out the overall framework of cyberdog_audio | 1.0     | 2023.05.21 |

## Overview

cyberdog_audio is voice service transfer station between nx and r329. For example, specific requests for playing offline or online voices can be forwarded through cyberdog_audio.

## Design

### Frames
![](./image/cyberdog_audio/framework.svg)
### Module Function

cyberdog_audio specific module design

1. Two-way service and topic encapsulation based on LCM communication

1. Realization of business functions
   
   > heartbeat maintenance

    > Authentication information synchronization

    > Offline/online voice playback

    > Realization of control vertical domain action

    > Volume setting and microphone enabling and disabling

    > Set nickname, record voiceprint and voiceprint recognition

    > Reporting of dog information