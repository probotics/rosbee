# rosbee
This repository contains source code for the Rosbee mobile platform.

Information about setting up Rosbee can be found in the [wiki](../../wiki).

## rosbee_embedded:
- rosbee_propeller:
  - Source code for the Parallax Embedded board

## rosbee_ros:
- rosbee_bringup:
  Contains launchfiles for starting rosbee in a minimal configuration, a configuration with kinect, and a launchfile for needed for joystick control.
- rosbee_description:
  Contains a URDF description of the rosbee robot.
- rosbee_node:
  The main node needed for communication with the hardware platform.
- rosbee_navigation:
  Contains launch files and configurations for running gmapping and move_base with Rosbee. 
