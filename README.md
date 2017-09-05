# README #

A pure pursuit path following controller, implemented in ROS.

## Dependencies ##

In addition to ROS packages included in the desktop-full variant, it requires The [ackermann_msgs ROS package](http://wiki.ros.org/ackermann_msgs) for publishing Ackermann steering commands, which can be installed from the official ROS repo:
```
sudo apt install ros-kinetic-ackermann-msgs
```


## Usage ##

To se a demo of the controller in action, use the [lattice_navigation_demos package](https://github.com/larics/lattice_navigation_demos).

## Limitations ##

The controller currently does not handle backwards driving correctly.
