# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Adapted from Isaac Sim 6.0.1 Tutorial 9 Part 1 for myCobot Pro 450 + myGripper F100.

"""Control the F100 gripper through the Pro 450 articulation.

Cycles joint2_left_joint between open (-58 deg) and closed (0 rad).
Mimic joints follow the drive joint.
"""

import argparse

from _common import ARTICULATION_PATH, GRIPPER_CLOSED, GRIPPER_JOINT, GRIPPER_OPEN, ROBOT_PRIM_PATH, ROBOT_USD
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
import omni.kit.app
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view

_HOLD_STEPS: int = 120


async def setup_scene() -> Articulation:
    """Load Pro 450 + F100 and return the articulation."""
    stage_utils.add_reference_to_stage(usd_path=ROBOT_USD, path=ROBOT_PRIM_PATH)
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    await omni.kit.app.get_app().next_update_async()
    set_camera_view(eye=[0.9, 0.9, 0.7], target=[0.15, 0.0, 0.25], camera_prim_path="/OmniverseKit_Persp")
    robot = Articulation(ARTICULATION_PATH)
    await omni.kit.app.get_app().next_update_async()
    return robot


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    """Run the F100 gripper control tutorial."""
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

    print(f"Robot DOFs: {robot.dof_names}")
    finger_idx = robot.dof_names.index(GRIPPER_JOINT)
    frame_count = 0

    while app.is_running():
        for target_pos, label in [(GRIPPER_CLOSED, "closing"), (GRIPPER_OPEN, "opening")]:
            print(f"Gripper {label}...")
            for _ in range(_HOLD_STEPS):
                robot.set_dof_position_targets(target_pos, dof_indices=finger_idx)
                app.update()
                frame_count += 1
                if args.test and frame_count >= _HOLD_STEPS * 2:
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
