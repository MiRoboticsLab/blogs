# <center>cyberdog_interactive design document</center>

## 1. Overview
The interactive function of the bionic robot will have the built-in interactive function in a fixed scene as the basic ability of the robot, and provide programming examples for users, and hope that users (developers) can develop more fun and cool functions.

## 2. Design
### 2.1. Feature design

The action of touching the chin will only be triggered when the dog is in the "sit down" state, and the action of touching the chin is divided into the following three states:
1. Touch the chin back and forth:
   - When the obstacle touches the dog's chin back and forth, it will enter this state. In this state, the dog will trigger the following actions:
     - the dog barks once;
     - Twist the buttocks left and right;
2. Touch the left side of the chin:
   - When the obstacle is continuously touching the left side of the dog's chin, it will enter this state. In this state, the dog will trigger the following actions:
     - the dog barks once;
     - right butt twisting action;
3. Touch the right side of the chin
   - When the obstacle is continuously touching the right side of the dog's chin, it will enter this state. In this state, the dog will trigger the following actions:
     - the dog barks once;
     - Twist the buttocks to the left;

### 2.2 Technology architecture

<center>

![](./image/cyberdog_interactive/cyberdog_interactive_module.svg)

</center>

As shown in the figure above, this function can be divided into two modules, one is the main module, and only one interactive module for detecting chin touch is hung under the main module. The interactive module depends on the TOF module, motion module and voice module:
1. TOF module: Provide head TOF sensing data for sensing whether there is an obstacle near the dog's chin;
2. Sports module: provide basic sports ability;
3. And voice module: provide basic voice interaction capabilities.
