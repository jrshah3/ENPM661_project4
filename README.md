# Project 4 - MoveIt Motion Planning on the Panda Robotic Arm

## Student Information
- **Name:** Jigar Shah
- **UID:** 121355690
- **Course:** ENPM661 - Spring 2026

## Project Overview
This project demonstrates MoveIt-based motion planning for the Franka Panda robotic arm in a simulated environment using ROS2 Humble.

The robot performs the following sequence:
1. Move to a home position
2. Move to goal position 1
3. Close the gripper
4. Move to goal position 2
5. Open the gripper
6. Return to the home position

The planning scene also includes obstacle avoidance for the simulated environment.

## Repository Structure
```text
project4_ws/
├── src/
│   ├── package_121355690/
│   ├── project4_moveit_config/
│   └── panda_description/
├── README.md
└── .gitignore
