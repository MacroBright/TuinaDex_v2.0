"""Canonical joint names and dimension contracts for TuinaDex teleoperation.

Directly aligns with TuinaDex v2.0 invariants:
- Hand: Canonical 16-DOF LEAP Hand ordering (0.0 rad = OPEN_POSE, positive = flexion).
- Arm: Canonical 6-DOF reBot-DevArm ordering (joint1 through joint6).
"""
from __future__ import annotations

# Exact match with TuinaDex_v2.0/src/tuinadex_core/tuinadex_core/names.py
CANONICAL_HAND_JOINT_NAMES = [
    "index_yaw",
    "index_mcp",
    "index_pip",
    "index_dip",
    "middle_yaw",
    "middle_mcp",
    "middle_pip",
    "middle_dip",
    "ring_yaw",
    "ring_mcp",
    "ring_pip",
    "ring_dip",
    "thumb_yaw",
    "thumb_cmc",
    "thumb_mcp",
    "thumb_ip",
]

# reBot-DevArm 6-DOF canonical joints
CANONICAL_ARM_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

# Unified 22-DOF composite robot (Arm 6 + Hand 16)
CANONICAL_COMPOSITE_JOINT_NAMES = (
    CANONICAL_ARM_JOINT_NAMES + CANONICAL_HAND_JOINT_NAMES
)

ARM_NUM_DOFS = len(CANONICAL_ARM_JOINT_NAMES)       # 6
HAND_NUM_DOFS = len(CANONICAL_HAND_JOINT_NAMES)     # 16
COMPOSITE_NUM_DOFS = len(CANONICAL_COMPOSITE_JOINT_NAMES)  # 22

