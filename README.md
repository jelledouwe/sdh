# sdh

This repo contains some ros packages for control of the SDH hand (ros noetic).

## Installation

First clone this repo in your catkin workspace:

```console
git clone git@github.com:jelledouwe/sdh.git
```

Then install the dependencies:

```console
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
```

Then build/make the package:

```console
  catkin_make
```

Finally, source you workspace.

## Launch Real and Sim controllers

The schunk_bringup package can be used to bring up the real and simulated hand.

Run for the real hand:

```console
  roslaunch schunk_bringup main.launch
```

And for the simulated hand:

```console
  roslaunch schunk_bringup main.launch sim:=true
```

This will launch a gazebo simulation.

## SDH interface

To ease controlling the hand, there is an SDH interface.

### Interface Real Example

```python
  import rospy
  import sdh_interface
  from sdh_interface import SDHInterface
  
  if __name__ == "__main__":
    rospy.init_node("example")
    
    sdhi = SDHInterface()
    rospy.sleep(10)
    sdhi.cmdOpen()
    rospy.sleep(10)
    sdhi.cmdGoToStartPos()
```

### Interface Sim Example

```python
  import rospy
  import sdh_interface
  from sdh_interface_sim import SDHInterfaceSim
  
  if __name__ == "__main__":
    rospy.init_node("example")
    
    sdhi = SDHInterfaceSim()
    rospy.sleep(10)
    sdhi.cmdOpen()
    rospy.sleep(10)
    sdhi.cmdGoToStartPos()
```

## Credits

Files adapted from [Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)](https://github.com/ipa320/schunk_modular_robotics).
Also adapted files from [@padmaja-kulkarni](https://github.com/padmaja-kulkarni).
