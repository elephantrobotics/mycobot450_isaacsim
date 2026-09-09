# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Adapted from Isaac Sim 6.0.1 Tutorial 9 Part 4 (PINK) for myCobot Pro 450 + myGripper F100.

"""Pick-and-place with PINK differential IK on Pro 450 + F100."""

import argparse

from _common import (
    ARM_HOME,
    ARM_JOINTS,
    ARTICULATION_PATH,
    EE_LINK_NAME,
    GRIPPER_CLOSED,
    GRIPPER_DOWN_QUAT,
    GRIPPER_JOINT,
    GRIPPER_OPEN,
    ROBOT_PRIM_PATH,
    ROBOT_USD,
    TCP_TO_PAD,
    TOOL_FRAME,
    URDF_PATH,
    arm_joint_indices,
    arm_q_from_dof,
    default_dof_positions,
    ee_at_grasp_pose,
    extract_arm_command,
    fk_tcp_position,
    gripper_is_holding,
    gripper_target_for_pick_phase,
    resolve_gripper_dof_index,
    should_advance_pick_phase,
)
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--urdf", type=str, default=URDF_PATH)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

if args.headless and not args.test:
    from isaacsim.core.experimental.utils.app import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.robot_motion.pink", True)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.pink import PinkIKController, PinkRobot, load_pink_robot


class Pro450PickPlace:
    """Eight-phase pick-and-place for Pro 450 + F100 using PINK IK."""

    _CUBE_PRIM_PATH = "/World/cube"
    _EE_LINK_NAME = EE_LINK_NAME
    _GRIPPER_JOINT = GRIPPER_JOINT
    _TOOL_FRAME = TOOL_FRAME

    _OPEN_POS: float = GRIPPER_OPEN
    _CLOSED_POS: float = GRIPPER_CLOSED

    _ABOVE_HEIGHT: float = 0.16
    _NEAR_HEIGHT: float = 0.04
    _TOOL_OFFSET: dict[str, float] = {TOOL_FRAME: TCP_TO_PAD}
    _EE_THRESHOLD: float = 0.05
    _GRIPPER_THRESHOLD: float = 0.15
    _MIN_STEPS: int = 60
    _MIN_GRIPPER_STEPS: int = 90
    _WARMUP_FRAMES: int = 120
    _PHYSICS_DT: float = 1.0 / 60.0
    _MOTION_RESET_EVENTS: tuple[int, ...] = (0, 1, 3, 4, 5, 7)

    _POSITION_COST: float = 0.5
    _ORIENTATION_COST: float = 1.0
    _POSTURE_COST: float = 1e-3

    _HOME_ARM: np.ndarray = np.array(ARM_HOME, dtype=np.float32)
    _DOWN_ORI: np.ndarray = np.array(GRIPPER_DOWN_QUAT, dtype=np.float32)

    _PHASE_LABELS: tuple[str, ...] = (
        "Pre-grasp: moving above cube",
        "Approach: descending to cube",
        "Grasp: closing gripper",
        "Lift: raising arm",
        "Transport: moving to target",
        "Lower: descending to place",
        "Release: opening gripper",
        "Retract: lifting arm away",
    )

    def __init__(
        self,
        urdf_path: str | None = None,
        cube_position: np.ndarray | None = None,
        target_position: np.ndarray | None = None,
        events_dt: list[int] | None = None,
    ) -> None:
        self._urdf_path = urdf_path if urdf_path is not None else URDF_PATH
        self.cube_position = cube_position if cube_position is not None else np.array([0.28, 0.0, 0.025])
        self.target_position = target_position if target_position is not None else np.array([0.22, 0.16, 0.05])
        self.events_dt = events_dt or [250, 300, 180, 80, 150, 250, 180, 100]

        self._event: int = 0
        self._step: int = 0
        self._t: float = 0.0
        self._warmup_remaining: int = self._WARMUP_FRAMES

        self._articulation: Articulation | None = None
        self._ee_prim: GeomPrim | None = None
        self._finger_idx: int | None = None
        self._pink_robot: PinkRobot | None = None
        self._controller: PinkIKController | None = None
        self._held_arm: tuple[list[float], list[int]] | None = None
        self._logged_apply: bool = False

    def _home_targets(self) -> np.ndarray:
        return np.array(default_dof_positions(self._articulation.dof_names, self._OPEN_POS), dtype=np.float32)

    async def setup_scene(self) -> None:
        """Build the scene and initialize PINK IK."""
        stage_utils.add_reference_to_stage(usd_path=ROBOT_USD, path=ROBOT_PRIM_PATH)
        GroundPlane("/World/GroundPlane")
        DomeLight("/World/DomeLight").set_intensities(1000)

        cube_obj = Cube(
            paths=self._CUBE_PRIM_PATH, positions=[self.cube_position], sizes=1.0, scales=[0.04, 0.04, 0.04]
        )
        RigidPrim(paths=cube_obj.paths)
        GeomPrim(paths=cube_obj.paths, apply_collision_apis=True)

        await omni.kit.app.get_app().next_update_async()
        set_camera_view(eye=[0.9, 0.9, 0.7], target=[0.2, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

        self._articulation = Articulation(ARTICULATION_PATH)
        await omni.kit.app.get_app().next_update_async()

        self._articulation.set_default_state(
            dof_positions=default_dof_positions(self._articulation.dof_names, self._OPEN_POS)
        )

        self._pink_robot = load_pink_robot(urdf_path=self._urdf_path)
        self._pink_robot.controlled_joint_names = [
            name for name in self._pink_robot.controlled_joint_names if name in ARM_JOINTS
        ]
        if self._TOOL_FRAME not in self._TOOL_OFFSET:
            raise ValueError(
                f"Tool frame '{self._TOOL_FRAME}' has no entry in _TOOL_OFFSET. "
                f"Add it: {list(self._TOOL_OFFSET.keys())}"
            )
        self._init_pink_q0()

        self._controller = PinkIKController(
            pink_robot=self._pink_robot,
            robot_joint_space=self._articulation.dof_names,
            robot_site_space=[self._TOOL_FRAME],
            tool_frame=self._TOOL_FRAME,
            position_cost=self._POSITION_COST,
            orientation_cost=self._ORIENTATION_COST,
            posture_cost=self._POSTURE_COST,
            solver="osqp",
            dt=self._PHYSICS_DT,
        )

    def initialize_after_play(self) -> None:
        """Resolve EE link and gripper DOF after physics starts."""
        link_names = self._articulation.link_names
        if self._EE_LINK_NAME in link_names:
            self._ee_prim = GeomPrim(paths=self._articulation.link_paths[0][link_names.index(self._EE_LINK_NAME)])
        else:
            print(f"WARNING: '{self._EE_LINK_NAME}' not found. Available: {link_names}")

        dof_names = self._articulation.dof_names
        self._finger_idx = resolve_gripper_dof_index(dof_names)
        if self._finger_idx is None:
            print(f"WARNING: gripper DOF not found. Available: {dof_names}")
        else:
            print(f" Gripper DOF '{dof_names[self._finger_idx]}' index={self._finger_idx}")

        self._articulation.reset_to_default_state()

    def _init_pink_q0(self) -> None:
        """Neutral posture: ARM_HOME so TCP +Z points down."""
        import pinocchio as pin

        q0 = pin.neutral(self._pink_robot.model)
        for i, name in enumerate(("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")):
            if self._pink_robot.model.existJointName(name):
                jid = self._pink_robot.model.getJointId(name)
                q0[self._pink_robot.model.joints[jid].idx_q] = float(self._HOME_ARM[i])
        self._pink_robot.q0 = q0

    def _phase_ee_target(self) -> np.ndarray:
        c, p = self.cube_position, self.target_position
        offset = self._TOOL_OFFSET[self._TOOL_FRAME]
        hi = self._ABOVE_HEIGHT + offset
        lo = self._NEAR_HEIGHT + offset
        targets = {
            0: [c[0], c[1], c[2] + hi],
            1: [c[0], c[1], c[2] + lo],
            2: [c[0], c[1], c[2] + lo],
            3: [c[0], c[1], c[2] + hi],
            4: [p[0], p[1], p[2] + hi],
            5: [p[0], p[1], p[2] + lo],
            6: [p[0], p[1], p[2] + lo],
            7: [p[0], p[1], p[2] + hi],
        }
        return np.array(targets[self._event], dtype=np.float32)

    def _make_setpoint(self, position: np.ndarray) -> mg.RobotState:
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[self._TOOL_FRAME],
                positions=([self._TOOL_FRAME], wp.array([position.tolist()], dtype=wp.float32)),
                orientations=([self._TOOL_FRAME], wp.array([self._DOWN_ORI.tolist()], dtype=wp.float32)),
            )
        )

    def _estimated_state(self) -> mg.RobotState:
        names = self._articulation.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self._articulation.get_dof_positions()),
                velocities=(names, self._articulation.get_dof_velocities()),
            )
        )

    def _set_gripper(self, pos: float) -> None:
        if self._finger_idx is not None:
            self._articulation.set_dof_position_targets(
                wp.array([pos], dtype=wp.float32), dof_indices=[self._finger_idx]
            )

    def _apply_arm_targets(self, desired: mg.RobotState) -> None:
        """Apply IK targets the same way as follow_target; then snapshot the arm."""
        joints = desired.joints
        if joints is None or joints.positions is None:
            return
        self._articulation.set_dof_position_targets(
            positions=joints.positions,
            dof_indices=joints.position_indices,
        )
        arm_pos, arm_idx = extract_arm_command(joints, self._articulation.dof_names)
        if arm_idx:
            self._held_arm = (arm_pos, arm_idx)
        if not self._logged_apply:
            self._logged_apply = True
            names = list(joints.position_names or [])
            print(f"  planner joints={names} arm_idx={arm_idx}")

    def _hold_arm(self) -> None:
        if self._held_arm is not None:
            pos, idx = self._held_arm
            self._articulation.set_dof_position_targets(wp.array(pos, dtype=wp.float32), dof_indices=idx)
            return
        names = self._articulation.dof_names
        current = self._articulation.get_dof_positions().numpy().reshape(-1)
        idx = arm_joint_indices(names)
        pos = [float(current[i]) for i in idx]
        self._articulation.set_dof_position_targets(wp.array(pos, dtype=wp.float32), dof_indices=idx)
        self._held_arm = (pos, idx)

    def _ee_near_target(self) -> bool:
        ee = self._ee_xyz()
        return ee is not None and bool(np.linalg.norm(ee - self._phase_ee_target()) < self._EE_THRESHOLD)

    def _gripper_pos(self) -> float | None:
        if self._finger_idx is None:
            return None
        return float(self._articulation.get_dof_positions().numpy().flatten()[self._finger_idx])

    def _gripper_at(self, target: float) -> bool:
        pos = self._gripper_pos()
        return pos is not None and abs(pos - target) < self._GRIPPER_THRESHOLD

    def _ee_xyz(self) -> np.ndarray | None:
        names = self._articulation.dof_names
        q = arm_q_from_dof(names, self._articulation.get_dof_positions().numpy())
        return fk_tcp_position(q)

    def _at_grasp_height(self) -> bool:
        ee = self._ee_xyz()
        return ee is not None and ee_at_grasp_pose(ee, self._phase_ee_target())

    def _over_cube(self) -> bool:
        ee = self._ee_xyz()
        if ee is None:
            return False
        target = self._phase_ee_target()
        return float(np.hypot(ee[0] - target[0], ee[1] - target[1])) < 0.05

    def _phase_status(self) -> str:
        ee = self._ee_xyz()
        grip = self._gripper_pos()
        target = self._phase_ee_target()
        q = arm_q_from_dof(self._articulation.dof_names, self._articulation.get_dof_positions().numpy())
        ee_s = "none" if ee is None else f"[{ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f}]"
        g_s = "none" if grip is None else f"{grip:.3f}"
        q_s = ",".join(f"{v:.2f}" for v in q)
        return f"ee={ee_s} target_z={target[2]:.3f} grip={g_s} q=[{q_s}]"

    def _reset_motion(self) -> None:
        if not self._controller.reset(
            self._estimated_state(), self._make_setpoint(self._phase_ee_target()), t=0.0
        ):
            raise RuntimeError("PinkIKController reset failed.")
        self._t = 0.0

    def _should_advance(self) -> bool:
        return should_advance_pick_phase(
            event=self._event,
            step=self._step,
            timeout=self.events_dt[self._event],
            min_motion_steps=self._MIN_STEPS,
            min_gripper_steps=self._MIN_GRIPPER_STEPS,
            ee_near=self._ee_near_target(),
            at_grasp_height=self._at_grasp_height(),
            over_cube=self._over_cube(),
            gripper_closed=gripper_is_holding(self._gripper_pos()),
            gripper_open=self._gripper_at(self._OPEN_POS),
        )

    def forward(self) -> bool:
        """Advance one simulation frame."""
        if self.is_done():
            return False

        if self._warmup_remaining > 0:
            targets = self._home_targets()
            n_dofs = len(targets)
            self._articulation.set_dof_position_targets(
                wp.array(targets, dtype=wp.float32), dof_indices=list(range(n_dofs))
            )
            self._warmup_remaining -= 1
            if np.abs(self._articulation.get_dof_positions().numpy().flatten()[:6] - self._HOME_ARM).max() < 0.1:
                self._warmup_remaining = 0
            return True

        if self._step == 0:
            if self._event in (2, 6):
                self._held_arm = None
            print(f" Phase {self._event}: {self._PHASE_LABELS[self._event]}  {self._phase_status()}")
            if self._event in self._MOTION_RESET_EVENTS:
                self._reset_motion()

        if self._event in (1, 5) and self._step > 0 and self._step % 120 == 0:
            print(f"  still descending  {self._phase_status()}")

        if self._event in (2, 6):
            self._hold_arm()
            if self._step > 0 and self._step % 60 == 0:
                print(f"  waiting gripper  {self._phase_status()}")
        else:
            desired = self._controller.forward(
                self._estimated_state(), self._make_setpoint(self._phase_ee_target()), self._t
            )
            if desired is not None and desired.joints.positions is not None:
                self._apply_arm_targets(desired)
        self._set_gripper(gripper_target_for_pick_phase(self._event))

        self._t += self._PHYSICS_DT
        self._step += 1
        if self._should_advance():
            print(f" Phase {self._event} done  {self._phase_status()}")
            self._event += 1
            self._step = 0

        return True

    def is_done(self) -> bool:
        return self._event >= len(self.events_dt)

    def reset(self) -> None:
        self._event = 0
        self._step = 0
        self._t = 0.0
        self._warmup_remaining = self._WARMUP_FRAMES
        self._held_arm = None
        self._logged_apply = False


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    """Run Pro 450 PINK pick-and-place."""
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device=args.device)

    scenario = Pro450PickPlace(urdf_path=args.urdf)
    app.run_coroutine(scenario.setup_scene())
    app.update()

    if args.headless and not args.test:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
    else:
        app_utils.play()
        app.update()
        scenario.initialize_after_play()

    needs_reset = True
    initialized = not args.headless
    frame_count = 0
    while app.is_running():
        app.update()
        if app_utils.is_playing() and SimulationManager.is_simulating():
            if not initialized:
                scenario.initialize_after_play()
                initialized = True
            if needs_reset:
                scenario.reset()
                needs_reset = False
            scenario.forward()
            frame_count += 1
            if args.test and frame_count >= 100:
                break
        elif not app_utils.is_playing():
            needs_reset = True


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
