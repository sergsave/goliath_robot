Goliath robot
=============
This repo contain source for ROS powered hexapod robot named Goliath.

## Description

Robot has 6 legs, each 3DOF. 
A robot can see (soon :) ) using a web camera.
I want that Goliath to detect and chase my cat with help of computer vision, it will be fun ;).

## Packages

**_goliath_bringup_**

Contain launcher and config files.

**_goliath_description_**

Contain STL 3d models of robot parts and XML description.

**_goliath_motion_**

Cpp Source Code! Convert command from teleop controller (legs, body, gait control) to
signals for motors driver.

**_goliath_msgs_**

My custom messages for ROS topics.

**_goliath_simulation_**

Cpp Source Code! Auxiliary stuff for simulation and visualization, such as servo-motors emulator.

**_goliath_teleop_**

Cpp Source Code! Sends commands based on keystrokes on the keyboard.

## Media

Robot 3d model screen

[![ScreenShot](https://i.paste.pics/3d7bf86f18e3a6d02eab9c7f837d6c7d.png)]

Video with visualization of a robot.

[![ScreenShot](http://img.youtube.com/vi/lGk0WovRYYo/0.jpg)]
(https://www.youtube.com/watch?v=tgWmmHb4pjg)
