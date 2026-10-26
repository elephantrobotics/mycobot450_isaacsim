# mycobot450_isaacsim

Isaac Sim / ROS 2 project for **myCobot Pro 450** and **myGripper F100**.

**Target runtime is NVIDIA Isaac Sim 6.2.0** (`6.2.0` branch), based on `6.1.0`. **Runtime validation is pending**; the branch name does not certify compatibility. Use `6.1.0` or `6.0.1` for the corresponding earlier target. The Isaac Sim 4.5.0 / ROS 2 Humble / MoveIt workflow no longer applies.

## Environment

- Isaac Sim **6.2.0** (Python packages `6.2.0.0`)
- If launching through [Isaac Lab](https://github.com/isaac-sim/IsaacLab), use a compatible checkout and environment; NVIDIA lists `v3.0.0-GA` for Sim 6.2 in its [compatibility matrix](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/compatibility.html).
- Ubuntu 24.04
- ROS 2 Jazzy
- `pymycobot` >= 4.0.6 (hardware sync only)

This repo does not vendor or pin the installed Isaac Sim runtime. Use the Python environment containing Sim 6.2; switching Git branches does not switch the installed simulator version.

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

## Launch Isaac Sim 6.2

Set `ISAACLAB` to your Isaac Lab clone (uv project with the Sim 6.2 extra already installed). Before launching, verify the actual environment:

```bash
export ISAACLAB=/path/to/IsaacLab
"$ISAACLAB/.venv/bin/python" - <<'PYVER'
import sys
from importlib.metadata import version
print("Python:", sys.executable)
for package in ("isaacsim", "isaacsim-core", "isaacsim-robot-motion", "isaacsim-asset", "isaacsim-ros2"):
    print(package, version(package))
PYVER
```

Each listed Sim package should report `6.2.0.0`. A missing package or a different version needs environment setup first. `uv run` can synchronize dependencies to the Lab project lockfile, so verify that checkout targets 6.2 before using it.

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

## Tutorial 9 (Isaac Sim 6.2)

Counterparts of the official pick-and-place standalone examples, using this repo's USD and `USD/sim_450_f100/config`.

The pip `isaacsim` command only launches `.kit` experiences. These scripts start `SimulationApp` themselves, so run them with the **Isaac Sim 6.2 interpreter** (Lab `.venv` or `uv run`), not system `python3` and not the `isaacsim` CLI:

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


## 6.2.0 手动验证清单

本分支从 `6.1.0` 继承模型、控制代码和规划配置，以下项目初始均为**待验证**。先沿用组合场景的 **PhysX** 配置；切换 Newton 另行验证。版本变化参考 [NVIDIA release notes](https://docs.isaacsim.omniverse.nvidia.com/latest/overview/release_notes.html)。

### 1. 场景与关节

- [ ] 按上面的命令确认实际安装版本为 `6.2.0.0`。
- [ ] 打开 `USD/sim_450_f100/450_f100.usda`：机械臂、夹爪、材质及背景完整，无缺失引用或扩展加载错误。
- [ ] 点击 Play，观察静置状态；Stop 后再次 Play，检查关节无持续抖动、散架或异常跳变。重新打开场景确认初始状态可恢复。
- [ ] 分别打开独立机械臂和夹爪 USD，确认也能加载和运行。

组合场景仍引用 6.0 在线背景资源，加载失败时记录具体 URL；不要仅把 URL 中的版本数字替换为 6.2。独立 F100 的 link 上有未赋值的 `isaac:nameOverride`，组合场景部分关节已有 `isaac:NameOverride`；以实际发布的关节名称为准，不做全局替换。

### 2. 独立示例

在仓库根目录运行上面 Tutorial 9 中的命令，**一次只运行一个，关闭窗口后再运行下一个**。使用 GUI 完成验证；脚本通常自动开始仿真，若处于暂停状态则点击 Play。

| 完成 | 命令对应脚本 | 操作与通过标准 |
| --- | --- | --- |
| [ ] | `tutorial_9_gripper_control.py` | 观察至少 3 次开闭；左右手指联动，无明显穿透、卡死或持续振荡。主动关节为 `joint2_left_joint`，0 rad 闭合、约 -1.0123 rad（-58°）张开。 |
| [ ] | `tutorial_9_arm_trajectory.py` | 观察至少 2 个完整轨迹周期；六轴平滑运动，无索引错误；测试 Stop/Play 后继续运行。 |
| [ ] | `tutorial_9_follow_target.py` | 拖动橙色 `/World/TargetCube` 到多个可达位置；末端持续跟随，夹爪不被机械臂控制指令意外驱动。 |
| [ ] | `tutorial_9_follow_target.py --with-obstacle` | 在障碍物附近移动可达目标，检查机械臂能合理避障，无明显穿透；局部规划器遇到不可达目标停滞不等同于版本不兼容。 |
| [ ] | `tutorial_9_pick_place_cumotion.py` | 至少完成一次接近、下降、夹紧、抬升、搬运、放下、松开；方块实际被夹起并放下，无明显滑落。 |
| [ ] | `tutorial_9_pick_place_pink.py` | 同样完成完整抓取搬运；同时检查终端没有 IK、关节映射或控制器初始化异常。 |

对跟随和抓取示例补测 Stop/Play，检查重置后可继续运行；若失败，记录具体操作顺序。抓取时保留现有指尖摩擦与凸分解配置，先观察效果再决定是否调参。

`--test` 只执行有限帧，不能证明完整抓取成功。现有脚本会捕获异常并打印 traceback，**退出码为 0 也不能单独判定通过**；需同时检查终端和实际动作。

### 3. ROS 2 与实机同步

先完成纯仿真验证。在打开组合场景并 Play 后，另开已加载 ROS 2 Jazzy 环境的终端：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /isaac_joint_states --once
ros2 topic hz /isaac_joint_states
ros2 topic info /isaac_joint_commands --verbose
```

- [ ] 状态话题持续发布，命令话题有 Isaac Sim 订阅者。
- [ ] 状态消息包含 `joint1` 至 `joint6` 和 `joint2_left_joint`；名称与代码一致，位置为弧度，数值有限。允许存在额外从动关节。

实机验证使用上面 ROS 2 hardware sync 的两个节点，先在 ROS 工作区执行 `colcon build` 并 `source install/setup.bash`。核对设备 IP、夹爪 ID 和运动空间后，一次只启动一个节点；`slider_control` 会驱动真实机械臂及夹爪，`follow_display` 会启用实机拖动模式。

- [ ] `slider_control`：先以小幅仿真动作验证，实机六轴及夹爪方向、开合量正确，无跳变。
- [ ] 停止前一个节点后运行 `follow_display`：拖动实机，仿真六轴和夹爪状态正确跟随。

若需要覆盖默认地址，可通过 `--ros-args -p ip:=YOUR_ROBOT_IP -p gripper_id:=YOUR_GRIPPER_ID` 传参。

### 4. 问题反馈

遇到问题时提供：失败项目、完整启动命令、Git 提交（`git rev-parse --short HEAD`）、上述版本输出、终端首个异常及 traceback，以及实际动作与预期差异；必要时附截图或短视频。日志可保存在已忽略的 `log/` 目录，不纳入提交。未执行的项目保留待验证状态。
