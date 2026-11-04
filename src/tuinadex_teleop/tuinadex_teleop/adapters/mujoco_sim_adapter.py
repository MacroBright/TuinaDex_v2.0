"""mujoco_sim_adapter.py — Production MuJoCo simulation adapter for reBot-DevArm + LEAP Hand.

Implements BaseRobotAdapter contract for:
1. 22-DOF unified position control (Arm 6-DOF + Hand 16-DOF).
2. Contact force inspection (fingertips vs patient dummy / tuina bed).
3. Both Headless CI mode and Interactive 3D Viewer mode via mujoco.viewer.
"""
from __future__ import annotations

from pathlib import Path
import time
from typing import Any, Dict, List, Optional
import numpy as np

try:
    import mujoco
    import mujoco.viewer as mj_viewer
    _HAS_MUJOCO = True
except ImportError:
    _HAS_MUJOCO = False

try:
    from tuinadex_teleop.core.names import (
        ARM_NUM_DOFS,
        CANONICAL_ARM_JOINT_NAMES,
        CANONICAL_COMPOSITE_JOINT_NAMES,
        CANONICAL_HAND_JOINT_NAMES,
        HAND_NUM_DOFS,
    )
    from tuinadex_teleop.core.types import RobotObservation, TeleopCommand
except ImportError:
    from tuina_teleop.core.names import (
        ARM_NUM_DOFS,
        CANONICAL_ARM_JOINT_NAMES,
        CANONICAL_COMPOSITE_JOINT_NAMES,
        CANONICAL_HAND_JOINT_NAMES,
        HAND_NUM_DOFS,
    )
    from tuina_teleop.core.types import RobotObservation, TeleopCommand
from .base import BaseRobotAdapter


# Default ready arm posture: elevated tuina ready stance with parallel palm hovering above bed (Palm Z=0.981m)
DEFAULT_READY_ARM_Q = np.array([0.0, 1.400, 1.230, -1.400, 0.0, 0.0], dtype=np.float64)


class MujocoSimAdapter(BaseRobotAdapter):
    """High-fidelity MuJoCo physics adapter for the composite tuina robot."""

    def __init__(
        self,
        model_path: Optional[str | Path] = None,
        initial_arm_q: Optional[np.ndarray] = None,
        initial_hand_q: Optional[np.ndarray] = None,
        headless: bool = True,
        timestep: float = 0.002,
    ):
        if not _HAS_MUJOCO:
            raise RuntimeError("mujoco is not installed or available in this Python environment.")

        # Resolve model path
        if model_path is None:
            # Default to bundled scene relative to this package
            root = Path(__file__).resolve().parent.parent.parent
            self.model_path = root / "configs" / "mjcf" / "tuina_sim_scene.xml"
        else:
            self.model_path = Path(model_path).resolve()

        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found at {self.model_path}")

        self.headless = headless
        self.timestep = timestep
        self._initial_arm_q = (
            initial_arm_q.copy() if initial_arm_q is not None else DEFAULT_READY_ARM_Q.copy()
        )
        self._initial_hand_q = (
            initial_hand_q.copy() if initial_hand_q is not None else np.zeros(HAND_NUM_DOFS, dtype=np.float64)
        )

        self._connected = False
        self.model: Optional[Any] = None
        self.data: Optional[Any] = None
        self._viewer: Optional[Any] = None

        # Address caches
        self._arm_qpos_indices: List[int] = []
        self._arm_dof_indices: List[int] = []
        self._arm_act_ids: List[int] = []
        self._arm_ctrl_limits: np.ndarray = np.zeros((ARM_NUM_DOFS, 2))

        self._hand_qpos_indices: List[int] = []
        self._hand_dof_indices: List[int] = []
        self._hand_act_ids: List[int] = []
        self._hand_ctrl_limits: np.ndarray = np.zeros((HAND_NUM_DOFS, 2))

        # Geom and body IDs for contact telemetry
        self._fingertip_body_ids: Dict[str, int] = {}
        self._dummy_body_ids: List[int] = []

    def connect(self) -> bool:
        """Load MuJoCo model, setup indices, initialize viewer if requested."""
        if self._connected:
            return True

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        if self.timestep > 0:
            self.model.opt.timestep = self.timestep
        self.data = mujoco.MjData(self.model)

        # Cache joint addresses for 6 arm joints
        self._arm_qpos_indices = []
        self._arm_dof_indices = []
        self._arm_act_ids = []
        for i, name in enumerate(CANONICAL_ARM_JOINT_NAMES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model {self.model_path}")
            self._arm_qpos_indices.append(int(self.model.jnt_qposadr[jid]))
            self._arm_dof_indices.append(int(self.model.jnt_dofadr[jid]))

            act_name = f"act_{name}"
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid < 0:
                raise ValueError(f"Actuator '{act_name}' not found in MuJoCo model")
            self._arm_act_ids.append(aid)
            self._arm_ctrl_limits[i] = self.model.actuator_ctrlrange[aid]

        # Cache joint addresses for 16 hand joints
        self._hand_qpos_indices = []
        self._hand_dof_indices = []
        self._hand_act_ids = []
        for i, name in enumerate(CANONICAL_HAND_JOINT_NAMES):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model {self.model_path}")
            self._hand_qpos_indices.append(int(self.model.jnt_qposadr[jid]))
            self._hand_dof_indices.append(int(self.model.jnt_dofadr[jid]))

            act_name = f"act_{name}"
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid < 0:
                raise ValueError(f"Actuator '{act_name}' not found in MuJoCo model")
            self._hand_act_ids.append(aid)
            self._hand_ctrl_limits[i] = self.model.actuator_ctrlrange[aid]

        # Cache contact bodies
        for tip_name in ("fingertip", "fingertip_2", "fingertip_3", "thumb_fingertip"):
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, tip_name)
            if bid >= 0:
                self._fingertip_body_ids[tip_name] = bid

        for dummy_part in ("patient_dummy", "dummy_torso", "dummy_head", "dummy_legs"):
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, dummy_part)
            if bid >= 0:
                self._dummy_body_ids.append(bid)

        # Set initial positions
        self.reset()

        # Launch interactive viewer if requested
        if not self.headless and _HAS_MUJOCO:
            try:
                self._viewer = mj_viewer.launch_passive(self.model, self.data)
            except Exception as e:
                print(f"[WARN] Could not launch passive MuJoCo viewer: {e}. Falling back to headless.")
                self._viewer = None

        self._connected = True
        return True

    def reset(self) -> None:
        """Reset simulation state to default initial configuration."""
        if self.data is None or self.model is None:
            return

        # Zero velocities and forces
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        self.data.act[:] = 0.0

        # Set arm posture
        for i, qpos_idx in enumerate(self._arm_qpos_indices):
            self.data.qpos[qpos_idx] = self._initial_arm_q[i]
            aid = self._arm_act_ids[i]
            self.data.ctrl[aid] = self._initial_arm_q[i]

        # Set hand posture
        for i, qpos_idx in enumerate(self._hand_qpos_indices):
            self.data.qpos[qpos_idx] = self._initial_hand_q[i]
            aid = self._hand_act_ids[i]
            self.data.ctrl[aid] = self._initial_hand_q[i]

        mujoco.mj_forward(self.model, self.data)

    def disconnect(self) -> None:
        """Safely close viewer and release resources."""
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def viewer(self) -> Optional[Any]:
        return self._viewer

    def send_command(self, cmd: TeleopCommand) -> None:
        """Send target 22-DOF positions to MuJoCo actuators."""
        if not self._connected or self.data is None:
            raise RuntimeError("Cannot send command: MujocoSimAdapter is not connected.")

        # Clip and assign arm targets
        target_arm = np.clip(
            cmd.arm_command.target_q,
            self._arm_ctrl_limits[:, 0],
            self._arm_ctrl_limits[:, 1],
        )
        for i, aid in enumerate(self._arm_act_ids):
            self.data.ctrl[aid] = target_arm[i]

        # Clip and assign hand targets
        target_hand = np.clip(
            cmd.hand_command.target_q,
            self._hand_ctrl_limits[:, 0],
            self._hand_ctrl_limits[:, 1],
        )
        for i, aid in enumerate(self._hand_act_ids):
            self.data.ctrl[aid] = target_hand[i]

    def step(self, n_steps: int = 1) -> None:
        """Advance physical simulation by n_steps."""
        if not self._connected or self.model is None or self.data is None:
            raise RuntimeError("Cannot step: MujocoSimAdapter is not connected.")

        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)

        # Safety check: Detect divergence / NaN
        if np.isnan(self.data.qpos).any() or np.isinf(self.data.qpos).any():
            self.reset()
            raise RuntimeError("MuJoCo simulation diverged: NaN or Inf detected in qpos! Reset to safe state.")

        # Sync viewer if running
        if self._viewer is not None and self._viewer.is_running():
            self._viewer.sync()

    def read_observation(self) -> RobotObservation:
        """Read 22-DOF joint positions, velocities, and motor efforts."""
        if not self._connected or self.data is None:
            raise RuntimeError("Cannot read observation: MujocoSimAdapter is not connected.")

        arm_q = np.array([self.data.qpos[idx] for idx in self._arm_qpos_indices], dtype=np.float64)
        arm_dq = np.array([self.data.qvel[idx] for idx in self._arm_dof_indices], dtype=np.float64)
        arm_effort = np.array([self.data.qfrc_actuator[idx] for idx in self._arm_dof_indices], dtype=np.float64)

        hand_q = np.array([self.data.qpos[idx] for idx in self._hand_qpos_indices], dtype=np.float64)
        hand_current = np.array([self.data.qfrc_actuator[idx] for idx in self._hand_dof_indices], dtype=np.float64)

        return RobotObservation(
            timestamp=float(self.data.time),
            arm_actual_q=arm_q,
            arm_actual_dq=arm_dq,
            arm_effort=arm_effort,
            hand_actual_q=hand_q,
            hand_current=hand_current,
            is_connected=self._connected,
        )

    def get_contact_forces(self) -> Dict[str, float]:
        """Compute normal contact forces (in Newtons) between fingertips and patient dummy/bed.
        
        Returns:
            Dictionary mapping finger name to normal force (N).
        """
        forces: Dict[str, float] = {
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "thumb": 0.0,
            "total": 0.0,
        }
        if not self._connected or self.data is None or self.model is None:
            return forces

        name_map = {
            "fingertip": "index",
            "fingertip_2": "middle",
            "fingertip_3": "ring",
            "thumb_fingertip": "thumb",
        }

        # Inspect active contacts
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            g1 = con.geom1
            g2 = con.geom2
            b1 = self.model.geom_bodyid[g1]
            b2 = self.model.geom_bodyid[g2]

            for tip_name, tip_bid in self._fingertip_body_ids.items():
                if b1 == tip_bid or b2 == tip_bid:
                    # Contact with dummy or bed
                    c_force = np.zeros(6, dtype=np.float64)
                    mujoco.mj_contactForce(self.model, self.data, i, c_force)
                    normal_f = abs(float(c_force[0]))
                    finger_label = name_map.get(tip_name, "other")
                    if finger_label in forces:
                        forces[finger_label] += normal_f
                        forces["total"] += normal_f

        return forces
