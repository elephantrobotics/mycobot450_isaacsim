# mycobot450_isaacsim

## Overview

Isaac Sim simulation and ROS 2 integration for **myCobot Pro 450** and **myGripper F100**.

## Development Environment

- Ubuntu 24.04
- NVIDIA Isaac Sim 6.2.0 (installed with uv)
- ROS 2 Jazzy
- `pymycobot` >= 4.0.6 (hardware sync)

## Project Structure

```text
mycobot450_isaacsim/
├── USD/
│   ├── mycobot_pro_450/      # Arm assets
│   ├── mygripper_f100/       # Gripper assets
│   └── sim_450_f100/         # Assembled scene
│       └── config/           # Motion planning configuration
├── standalone_examples/      # Simulation examples
├── robots_description/       # URDF packages
└── pro450_isaacsim/          # ROS 2 hardware sync nodes
```


## Run Examples

From the repository root, use the Python interpreter in your uv environment containing Isaac Sim. Run one example at a time.

```bash
export SIM_PYTHON=/path/to/.venv/bin/python

# Gripper open/close
"$SIM_PYTHON" standalone_examples/tutorials/manipulation/tutorial_9_gripper_control.py

# Arm trajectory
"$SIM_PYTHON" standalone_examples/tutorials/manipulation/tutorial_9_arm_trajectory.py

# Target following (add --with-obstacle for obstacle avoidance)
"$SIM_PYTHON" standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py

# Pick and place with cuMotion or PINK
"$SIM_PYTHON" standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py
"$SIM_PYTHON" standalone_examples/tutorials/manipulation/tutorial_9_pick_place_pink.py
```
