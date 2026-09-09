# mycobot450_isaacsim

Isaac Sim / ROS 2 project for **myCobot Pro 450** and **myGripper F100**.

**In development. Target runtime is NVIDIA Isaac Sim 6.0.1.** The older Isaac Sim 4.5.0 / ROS 2 Humble / MoveIt workflow no longer applies.

## Environment

- Isaac Sim **6.0.1**
- Ubuntu 24.04
- ROS 2 Jazzy
- `pymycobot` >= 4.0.6 (hardware sync only)

## Layout

```text
USD/mycobot_pro_450/          # arm scene
USD/mygripper_f100/           # gripper scene
USD/sim_450_f100/             # assembled scene 450_f100.usda
USD/sim_450_f100/config/      # cuMotion: URDF / XRDF / rmp_flow.yaml
standalone_examples/          # Tutorial 9 counterparts for Pro 450 + F100
robots_description/           # URDF description packages
pro450_isaacsim/              # ROS 2 sync nodes
```

## Usage

1. Start Isaac Sim 6.0.1 and open `USD/sim_450_f100/450_f100.usda` (or the standalone arm / gripper USD).
2. Click **Play**.
3. For hardware sync, put this repo in a ROS 2 workspace, `colcon build`, then run **one** of:

```bash
ros2 run pro450_isaacsim slider_control
ros2 run pro450_isaacsim follow_display
```

Do not run both nodes at the same time. Isaac exchanges joint data with ROS on `/isaac_joint_states` and `/isaac_joint_commands`.

## Tutorial 9 (Isaac Sim 6.0.1)

Counterparts of the official pick-and-place standalone examples, using this repo's USD and `USD/sim_450_f100/config`.

The pip `isaacsim` command only launches `.kit` experiences. These scripts start `SimulationApp` themselves, so run them with the **venv Python** (not `isaacsim`):

```bash
python standalone_examples/tutorials/manipulation/tutorial_9_gripper_control.py
python standalone_examples/tutorials/manipulation/tutorial_9_arm_trajectory.py
python standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py
python standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py --with-obstacle
python standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py
python standalone_examples/tutorials/manipulation/tutorial_9_pick_place_pink.py
```

F100 drive joint is `joint2_left_joint` (0 rad closed, -58 deg open). Tool frame is `tcp`.
