"""Shared Pro 450 + F100 paths and joint names for Tutorial 9 counterparts."""

from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_USD = str(REPO_ROOT / "USD" / "sim_450_f100" / "450_f100.usda")
XRDF_DIR = str(REPO_ROOT / "USD" / "sim_450_f100" / "config")
URDF_FILENAME = "450_f100.urdf"
XRDF_FILENAME = "robot.xrdf"
URDF_PATH = str(REPO_ROOT / "USD" / "sim_450_f100" / "config" / URDF_FILENAME)

ROBOT_PRIM_PATH = "/World/mycobot_pro450"
ARTICULATION_PATH = "/World/mycobot_pro450/Geometry"

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT = "joint2_left_joint"
# USD/URDF/XRDF: 0 rad = closed, -58 deg = fully open.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = -1.012291
# A grasped cube stalls the fingers short of 0 (logs: ~-0.315). Closer to 0 than this
# means the F100 has closed onto something, or shut empty.
GRIPPER_HOLDING = -0.55
# Planning frame in URDF/XRDF. +Z = F100 finger approach. Not a USD link.
# Same role as UR10e tool0: cuMotion moves this frame, not Link6 / the flange.
TOOL_FRAME = "tcp"
# USD link used to read pose (tcp has no USD prim). Same origin as tcp today.
EE_LINK_NAME = "f100_base_link"
# tcp origin is f100_base. Pads are ~this far along tcp +Z (world -Z when grasping).
# Official UR10e+2F-140 uses NEAR_HEIGHT=0.185 on tool0 for the same reason.
TCP_TO_PAD = 0.11
# wxyz. Aligns TCP +Z with world -Z (approach the table from above).
GRIPPER_DOWN_QUAT = (0.0, 0.0, 1.0, 0.0)
# Seed pose: TCP +Z along world -Z. j6 = 45 deg matches GRIPPER_DOWN_QUAT yaw.
ARM_HOME = (0.00, -0.90, -0.65, 0.00, 0.00, 0.7854)


def is_arm_joint(name: str) -> bool:
    return name in ARM_JOINTS


def arm_joint_indices(dof_names: list[str]) -> list[int]:
    return [i for i, name in enumerate(dof_names) if name in ARM_JOINTS]


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def _origin_T(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def _rotz_T(q: float) -> np.ndarray:
    t = np.eye(4)
    c, s = np.cos(q), np.sin(q)
    t[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return t


# From USD/sim_450_f100/config/450_f100.urdf (export). Used when GeomPrim USD pose is stale.
_ARM_ORIGINS = (
    ((0.0, 0.0, 0.20999999), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.00000001), (0.0, 1.57079633, -1.57079633)),
    ((-0.1792196, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((-0.17378039, 0.0, 0.08549054), (0.0, 0.0, -1.57079627)),
    ((-0.00000001, -0.09750101, 0.0), (-1.57079448, -1.57079263, -3.1415909)),
    ((0.0, 0.063173, 0.00020984), (-1.57080002, 0.0, 0.0)),
)
_ASSEMBLER_ORIGIN = ((0.0, 0.0, 0.0), (0.79496343, -1.57079269, -0.00956537))
_TCP_ORIGIN = ((0.0, 0.0, 0.0), (0.0, 1.57079632679, 0.0))


def fk_tcp_position(arm_q) -> np.ndarray:
    """World position of the planning TCP. Robot base at identity."""
    t = np.eye(4)
    for origin, qi in zip(_ARM_ORIGINS, arm_q):
        t = t @ _origin_T(*origin) @ _rotz_T(float(qi))
    t = t @ _origin_T(*_ASSEMBLER_ORIGIN) @ _origin_T(*_TCP_ORIGIN)
    return t[:3, 3].astype(np.float32)


def arm_q_from_dof(dof_names: list[str], positions) -> np.ndarray:
    q = np.zeros(6, dtype=np.float32)
    index = {name: i for i, name in enumerate(dof_names)}
    pos = np.asarray(positions, dtype=np.float32).reshape(-1)
    for i, name in enumerate(ARM_JOINTS):
        if name in index:
            q[i] = pos[index[name]]
    return q


def _as_1d(value) -> np.ndarray:
    if value is None:
        return np.zeros(0)
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy()).reshape(-1)
    return np.asarray(value).reshape(-1)


def extract_arm_command(desired_joints, dof_names: list[str]) -> tuple[list[float], list[int]]:
    """Map a planner JointState onto the 6 arm DOFs of the articulation."""
    if desired_joints is None or desired_joints.positions is None:
        return [], []
    values = _as_1d(desired_joints.positions)
    names = list(desired_joints.position_names or [])
    art_index = {name: i for i, name in enumerate(dof_names)}
    arm_pos: list[float] = []
    arm_idx: list[int] = []
    if names:
        for name, value in zip(names, values):
            if name in ARM_JOINTS and name in art_index:
                arm_pos.append(float(value))
                arm_idx.append(art_index[name])
        if arm_pos:
            return arm_pos, arm_idx
    raw_idx = desired_joints.position_indices
    indices = list(range(len(values))) if raw_idx is None else _as_1d(raw_idx).astype(int).tolist()
    for value, index in zip(values, indices):
        if 0 <= index < len(dof_names) and dof_names[index] in ARM_JOINTS:
            arm_pos.append(float(value))
            arm_idx.append(index)
    return arm_pos, arm_idx


def gripper_target_for_pick_phase(event: int) -> float:
    """Keep the F100 closed while lifting / carrying / lowering (phases 2–5)."""
    return GRIPPER_CLOSED if event in (2, 3, 4, 5) else GRIPPER_OPEN


def gripper_is_holding(pos: float | None, holding_limit: float = GRIPPER_HOLDING) -> bool:
    """True when the drive joint has closed far enough to pinch or shut."""
    return pos is not None and float(pos) > holding_limit


def resolve_gripper_dof_index(dof_names: list[str]) -> int | None:
    if GRIPPER_JOINT in dof_names:
        return dof_names.index(GRIPPER_JOINT)
    for name in dof_names:
        if "joint2_left" in name:
            return dof_names.index(name)
    return None


def ee_at_grasp_pose(ee_xyz, target_xyz, xy_tol: float = 0.04, z_tol: float = 0.035) -> bool:
    """True only when XY is over the cube and Z has actually descended."""
    xy = float(((ee_xyz[0] - target_xyz[0]) ** 2 + (ee_xyz[1] - target_xyz[1]) ** 2) ** 0.5)
    return xy < xy_tol and abs(float(ee_xyz[2] - target_xyz[2])) < z_tol


def should_advance_pick_phase(
    *,
    event: int,
    step: int,
    timeout: int,
    min_motion_steps: int,
    min_gripper_steps: int,
    ee_near: bool,
    at_grasp_height: bool,
    over_cube: bool,
    gripper_closed: bool,
    gripper_open: bool,
) -> bool:
    """Hard gates: do not lift until the gripper is closed at grasp height."""
    if event == 2:
        return step >= min_gripper_steps and gripper_closed
    if event == 6:
        return step >= min_gripper_steps and gripper_open
    if event in (1, 5):
        if step >= min_motion_steps and at_grasp_height:
            return True
        if step >= timeout and over_cube:
            return True
        return step >= timeout * 2
    if ee_near and step >= min_motion_steps:
        return True
    return event in (0, 3, 4, 7) and step >= timeout


def default_dof_positions(dof_names: list[str], gripper_pos: float = GRIPPER_OPEN) -> list[float]:
    """Arm at ARM_HOME, F100 drive joint at gripper_pos, other DOFs at 0."""
    positions = [0.0] * len(dof_names)
    index = {name: i for i, name in enumerate(dof_names)}
    for i, name in enumerate(ARM_JOINTS):
        if name in index:
            positions[index[name]] = ARM_HOME[i]
    if GRIPPER_JOINT in index:
        positions[index[GRIPPER_JOINT]] = gripper_pos
    return positions
