# Project 4 - MoveIt Motion Planning on the Panda Robotic Arm

## Student Information
- **Name:** Jigar Shah
- **UID:** 121355690
- **Course:** ENPM661 - Spring 2026

---

## Project Overview
This project demonstrates MoveIt-based motion planning for the Franka Panda robotic arm in a simulated RViz environment using ROS 2 Humble. The robot performs a pick-and-place sequence with obstacle avoidance using the MoveIt Motion Planning Framework.

The robot executes the following sequence:
1. Move to home position
2. Open gripper
3. Move to goal position 1 (pick location)
4. Close gripper (simulate picking)
5. Move to goal position 2 (place location)
6. Open gripper (simulate placing)
7. Return to home position

---

## Repository Structure
```
project4_ws/
├── src/
│   ├── package_121355690/        # Custom motion planning package
│   │   ├── src/
│   │   │   └── package_121355690.cpp
│   │   ├── launch/
│   │   │   └── pick_place_demo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── project4_moveit_config/   # MoveIt configuration for Panda
│   └── panda_description/        # Panda URDF/robot description
└── README.md
```

---

## Dependencies

### System Requirements
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Colcon build tools

### Install Required Packages
```bash
sudo apt-get update
sudo apt install ros-humble-moveit
sudo apt-get install ros-humble*controller*
sudo apt-get install ros-humble*joint*state*
sudo apt install python3-colcon-common-extensions
```

### Fix OMPL Library (required on some systems)
```bash
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libompl.so.17 /usr/lib/x86_64-linux-gnu/libompl.so.18
sudo ldconfig
```

---

## Workspace Setup

Clone this repository into the `src` folder of a ROS 2 workspace:

```bash
mkdir -p ~/project4_ws/src
cd ~/project4_ws/src
git clone <your-github-repo-url>
```

Build the workspace:

```bash
cd ~/project4_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

**Use two separate terminals.**

### Terminal 1 — Launch MoveIt + RViz backend
```bash
cd ~/project4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project4_moveit_config demo.launch.py
```

Wait until you see:
```
[move_group] You can start planning now!
```

### Terminal 2 — Launch the pick and place sequence
```bash
cd ~/project4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch package_121355690 pick_place_demo.launch.py
```

---

## Motion Sequence

| Step | Action |
|------|--------|
| 1 | Move arm to home position |
| 2 | Open gripper |
| 3 | Move arm to goal 1 (pick location) |
| 4 | Close gripper (simulate pick) |
| 5 | Move arm to goal 2 (place location) |
| 6 | Open gripper (simulate place) |
| 7 | Return arm to home position |

---

## Planning Scene / Obstacles

The planning scene includes the following collision objects:
- **Table** — surface beneath the robot
- **Path Blocker 1** — pillar on the right side
- **Path Blocker 2** — pillar on the left side
- **Center Wall** — wall between pick and place locations

These force the planner to compute non-trivial collision-free trajectories.

---

## Notes
- The MoveIt Setup Assistant was used to generate `project4_moveit_config`
- Planning is done using the OMPL (RRTConnect) planner
- The simulation runs in RViz with fake hardware controllers
- The gripper ghost (white) in RViz shows the planned gripper state

---

## Submission Contents
- `package_121355690.zip` — zipped ROS 2 package
- `Jigar_121355690.pdf` — containing:
  - Pick and place simulation video - https://drive.google.com/file/d/13T09eE9X1HwhEq2wj54pZ7nBcdwHlKMD/view?usp=sharing
  - MoveIt Setup Assistant video - https://drive.google.com/file/d/1eNGcd39ujokAB_EskpH_A9aZg1efbVNp/view?usp=drive_link
  - Drive link - https://drive.google.com/drive/folders/1V-wLLaBsrlRsCjMs5MMfh60_EBk0e_Xt?usp=drive_link
  - GitHub repository link - https://github.com/jrshah3/ENPM661_project4/
