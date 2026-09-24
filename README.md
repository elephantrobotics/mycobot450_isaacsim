# mycobot450_isaacsim

Isaac Sim / ROS 2 project for **myCobot Pro 450** and **myGripper F100**.

**In development. Target runtime is NVIDIA Isaac Sim 6.1.0** (`6.1.0-dev`). The Isaac Sim 4.5.0 / ROS 2 Humble / MoveIt workflow no longer applies. Use `6.0.1-dev` if you still need Isaac Sim 6.0.1.

## Environment

- Isaac Sim **6.1.0** (pip / uv extra, typically from an [Isaac Lab](https://github.com/isaac-sim/IsaacLab) `release/3.0.0` checkout)
- Ubuntu 24.04
- ROS 2 Jazzy
- `pymycobot` >= 4.0.6 (hardware sync only)

This repo does not vendor Isaac Sim. System `python3` will fail with `No module named 'isaacsim'`. Use the Python that has the Sim 6.1 package installed.

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

## Launch Isaac Sim 6.1

Set `ISAACLAB` to your Isaac Lab clone (uv project with `--extra isaacsim` already synced):

```bash
export ISAACLAB=/path/to/IsaacLab
```

Open the Kit app (no `source activate` required):

```bash
cd "$ISAACLAB"
uv run --extra isaacsim isaaclab -s
```

Then open `USD/sim_450_f100/450_f100.usda` (or the standalone arm / gripper USD) and click **Play**.

Equivalent:

```bash
uv run --directory "$ISAACLAB" --extra isaacsim isaacsim
```

If you use a named venv instead of Lab's `.venv`, activate that environment first, then run `isaacsim` or `isaaclab -s` from it.

## ROS 2 hardware sync

Put this repo in a ROS 2 workspace, `colcon build`, then run **one** of:

```bash
ros2 run pro450_isaacsim slider_control
ros2 run pro450_isaacsim follow_display
```

Do not run both nodes at the same time. Isaac exchanges joint data with ROS on `/isaac_joint_states` and `/isaac_joint_commands`.

## Tutorial 9 (Isaac Sim 6.1)

Counterparts of the official pick-and-place standalone examples, using this repo's USD and `USD/sim_450_f100/config`.

The pip `isaacsim` command only launches `.kit` experiences. These scripts start `SimulationApp` themselves, so run them with the **Isaac Sim 6.1 interpreter** (Lab `.venv` or `uv run`), not system `python3` and not the `isaacsim` CLI:

```bash
cd /path/to/mycobot450_isaacsim

"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_gripper_control.py
"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_arm_trajectory.py
"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py
"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_follow_target.py --with-obstacle
"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py
"$ISAACLAB/.venv/bin/python" standalone_examples/tutorials/manipulation/tutorial_9_pick_place_pink.py
```

Keep the current working directory in this repo so USD relative paths resolve.

Example on a machine where Isaac Lab lives at `~/IsaacLab` (uv `--extra isaacsim` already installed):

```bash
cd ~/mycobot450_isaacsim
~/IsaacLab/.venv/bin/python standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py
```

The same interpreter works for the other Tutorial 9 scripts. With uv only:

```bash
uv run --directory "$ISAACLAB" --extra isaacsim python \
  "$(pwd)/standalone_examples/tutorials/manipulation/tutorial_9_pick_place_cumotion.py"
```

F100 drive joint is `joint2_left_joint` (0 rad closed, -58 deg open). Tool frame is `tcp`.
