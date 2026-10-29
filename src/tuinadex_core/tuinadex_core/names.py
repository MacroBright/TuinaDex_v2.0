"""Canonical joint-space vocabulary shared by runtime, simulation, and data tools."""

# 16-DOF LEAP Hand 规范关节名称
CANONICAL_JOINT_NAMES = [
    "index_yaw",  "index_mcp",  "index_pip",  "index_dip",
    "middle_yaw", "middle_mcp", "middle_pip", "middle_dip",
    "ring_yaw",   "ring_mcp",   "ring_pip",   "ring_dip",
    "thumb_yaw",  "thumb_cmc",  "thumb_mcp",  "thumb_ip",
]

# 6-DOF reBot B601-RS 机械臂规范关节名称
CANONICAL_ARM_JOINT_NAMES = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]

