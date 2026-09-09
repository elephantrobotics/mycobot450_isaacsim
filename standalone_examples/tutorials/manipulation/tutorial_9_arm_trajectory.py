# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Adapted from Isaac Sim 6.0.1 Tutorial 9 Part 2 for myCobot Pro 450 + myGripper F100.

"""Plan and execute a joint-space arm trajectory on Pro 450."""

import argparse

from _common import ARTICULATION_PATH, ROBOT_PRIM_PATH, ROBOT_USD, is_arm_joint
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

if args.headless:
    from isaacsim.core.experimental.utils.app import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view

# Pro 450 analog of the UR10e "home / reach-out" demo: F100 +X points down.
# Do not copy UR10 numbers; 450 zero pose has the flange pointing sideways.
# _HOME: np.ndarray = np.array([0.00, -0.90, -0.65, 0.00, 1.5708, 0.00], dtype=np.float32)
_HOME: np.ndarray = np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], dtype=np.float32)
_REACH: np.ndarray = np.array([0.50, -0.55, -0.90, 0.00, 0.00, 0.00], dtype=np.float32)


async def setup_scene() -> Articulation:
    """Load Pro 450 + F100 and return the articulation."""
    stage_utils.add_reference_to_stage(usd_path=ROBOT_USD, path=ROBOT_PRIM_PATH)
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    await omni.kit.app.get_app().next_update_async()
    set_camera_view(eye=[0.9, 0.9, 0.8], target=[0.2, 0.0, 0.25], camera_prim_path="/OmniverseKit_Persp")
    robot = Articulation(ARTICULATION_PATH)
    await omni.kit.app.get_app().next_update_async()
    return robot


def get_estimated_state(robot: Articulation, joint_space: list[str]) -> mg.RobotState:
    """Read current joint state for the trajectory follower."""
    return mg.RobotState(
        joints=mg.JointState.from_name(
            robot_joint_space=joint_space,
            positions=(joint_space, robot.get_dof_positions()),
            velocities=(joint_space, robot.get_dof_velocities()),
            efforts=(joint_space, robot.get_dof_efforts()),
        )
    )


def apply_desired_state(robot: Articulation, desired_state: mg.RobotState) -> None:
    """Apply position / velocity / effort commands from the follower."""
    if desired_state.joints is None:
        return
    joint_state = desired_state.joints
    if joint_state.positions is not None:
        robot.set_dof_position_targets(joint_state.positions, dof_indices=joint_state.position_indices)
    if joint_state.velocities is not None:
        robot.set_dof_velocity_targets(joint_state.velocities, dof_indices=joint_state.velocity_indices)
    if joint_state.efforts is not None:
        robot.set_dof_efforts(joint_state.efforts, dof_indices=joint_state.effort_indices)


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    """Run the Pro 450 arm trajectory tutorial."""
    SimulationManager.setup_simulation(dt=1.0 / 60.0)

    robot = app.run_coroutine(setup_scene())
    app.update()

    if args.headless:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
        while app.is_running() and not app_utils.is_playing():
            app.update()
    else:
        app_utils.play()
        app.update()

    robot_joint_space = robot.dof_names
    arm_joints = [n for n in robot_joint_space if is_arm_joint(n)]
    print(f"Arm joints ({len(arm_joints)}): {arm_joints}")

    initial_positions = np.zeros(robot.num_dofs, dtype=np.float32)
    for i, joint_name in enumerate(arm_joints):
        initial_positions[robot_joint_space.index(joint_name)] = _HOME[i]
    robot.set_dof_positions(wp.from_numpy(initial_positions, dtype=wp.float32))
    app.update()

    waypoints = np.array(
        [
            _HOME,  # hover, gripper down
            _REACH,  # yaw and reach-out, gripper down
            _HOME,  # back to home
        ]
    )
    max_velocities = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    max_accelerations = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])

    trajectory = mg.Path(waypoints).to_minimal_time_joint_trajectory(
        max_velocities=max_velocities,
        max_accelerations=max_accelerations,
        robot_joint_space=robot_joint_space,
        active_joints=arm_joints,
    )
    print(f"Trajectory duration: {trajectory.duration:.2f} s")

    follower = mg.TrajectoryFollower()
    follower.set_trajectory(trajectory)

    dt = SimulationManager.get_physics_dt()
    max_steps = int((trajectory.duration + 1.0) / dt)
    frame_count = 0

    # Replay until the window is closed. TrajectoryFollower.clear()s the path
    # when time exceeds duration, so set_trajectory must run every cycle.
    while app.is_running():
        app.update()
        if not (app_utils.is_playing() and SimulationManager.is_simulating()):
            continue
        follower.set_trajectory(trajectory)
        simulation_time = 0.0
        if not follower.reset(get_estimated_state(robot, robot_joint_space), None, simulation_time):
            raise RuntimeError("Failed to reset TrajectoryFollower")
        for _ in range(max_steps):
            app.update()
            if not (app_utils.is_playing() and SimulationManager.is_simulating()):
                break
            simulation_time += dt
            desired_state = follower.forward(get_estimated_state(robot, robot_joint_space), None, simulation_time)
            if desired_state is None:
                print("Trajectory complete.")
                break
            apply_desired_state(robot, desired_state)
            frame_count += 1
            if args.test and frame_count >= 100:
                return


if __name__ == "__main__":
    try:
        main(args, simulation_app)
    except Exception:
        import traceback

        traceback.print_exc()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()
