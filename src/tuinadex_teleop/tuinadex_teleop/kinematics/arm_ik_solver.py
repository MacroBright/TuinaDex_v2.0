"""arm_ik_solver.py — 6-DOF reBot-DevArm Inverse Kinematics & Safety Gate.

Features:
1. Pinocchio-based reduced 6-DOF kinematic tree from URDF (gripper joints locked).
2. Dual-solver architecture:
   - Primary: Pink QP-IK with ProxQP/OSQP solver and joint limit / posture tasks.
   - Secondary / Fallback: Pinocchio Damped Least-Squares (DLS) differential IK with null-space posture regularization.
3. Multiple orientation modes:
   - "fixed_downward": End-effector tool points directly downward (Tuina pressing).
   - "track_wrist": Tracks human wrist 3D rotation matrix.
   - "position_only": 3-DOF translation tracking with free orientation.
4. Multi-tier safety gates:
   - Cartesian workspace clamping (safety floor Z >= Z_min).
   - Joint limit soft buffers.
   - Joint velocity saturation (|dq| <= dq_max).
   - Joint-space 1€ adaptive filtering for jitter-free trajectory.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import os
from pathlib import Path
import sys
import time
from typing import Optional, Tuple, Union

import numpy as np

# Defense-in-depth: Remove incompatible Python version packages (e.g. ROS Lyrical Python 3.14 on Ubuntu 24.04)
if sys.version_info[:2] == (3, 10):
    sys.path = [p for p in sys.path if not ("python3.1" in p and "python3.10" not in p)]

# Dynamically locate cmeel Pinocchio dynamic library path in active virtualenv/conda
_py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
_site_pkgs = os.path.join(sys.prefix, "lib", _py_ver, "site-packages")
_cmeel_lib = os.path.join(_site_pkgs, "cmeel.prefix", "lib")
_cmeel_pkg = os.path.join(_cmeel_lib, _py_ver, "site-packages")

if not os.path.isdir(_cmeel_lib):
    _fallback_prefix = os.path.expanduser("~/win_office/conda/envs/tuinadex-control")
    _site_pkgs = os.path.join(_fallback_prefix, "lib", _py_ver, "site-packages")
    _cmeel_lib = os.path.join(_site_pkgs, "cmeel.prefix", "lib")
    _cmeel_pkg = os.path.join(_cmeel_lib, _py_ver, "site-packages")

if os.path.isdir(_cmeel_lib) and _cmeel_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = f"{_cmeel_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}"
if os.path.isdir(_cmeel_pkg) and _cmeel_pkg not in sys.path:
    sys.path.insert(0, _cmeel_pkg)
if os.path.isdir(_site_pkgs) and _site_pkgs not in sys.path:
    sys.path.insert(0, _site_pkgs)

import pinocchio as pin

_HAS_PINK = False
try:
    import pink
    from pink.tasks import FrameTask, PostureTask
    import qpsolvers
    _HAS_PINK = True
except Exception:
    _HAS_PINK = False

from ..core.filters import OneEuroFilter
from ..core.names import ARM_NUM_DOFS, CANONICAL_ARM_JOINT_NAMES
from ..core.types import ClutchState, TeleopArmCommand


class ArmOrientationMode(Enum):
    """End-effector orientation tracking modes."""
    FIXED_DOWNWARD = auto()   # Tool axis (+Z in link6) points straight down (-Z in base)
    TRACK_IMU = auto()        # 6-axis IMU wrist attitude tracking (Visual XYZ + IMU RPY)
    TRACK_WRIST = auto()      # Track 3D rotation from human wrist
    POSITION_ONLY = auto()    # 3D position tracking with posture regularization


@dataclass
class ArmIKMetrics:
    """Diagnostic telemetry from the 6-DOF Arm IK solver."""
    computation_time_ms: float = 0.0
    position_error_m: float = 0.0
    orientation_error_rad: float = 0.0
    is_converged: bool = False
    iterations: int = 0
    solver_mode: str = "pinocchio_dls"


# Default downward rotation (pi around X: maps +Z_tool to -Z_base)
R_DOWNWARD_DEFAULT = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)


# Canonical downward posture (Tuina ready stance with parallel palm hovering safely above bed, Palm Z=0.981m)
DEFAULT_READY_ARM_Q = np.array([0.0, 1.400, 1.230, -1.400, 0.0, 0.0], dtype=np.float64)


class ArmIKSolver:
    """High-performance 6-DOF Inverse Kinematics solver with multi-tier safety."""

    def __init__(
        self,
        urdf_path: Optional[str] = None,
        ee_frame_name: str = "link6",
        home_q: Optional[np.ndarray] = None,
        prefer_pink: bool = True,
        joint_safety_margin_rad: float = 0.03,
        max_joint_velocity_rad_s: float = 6.0,
        filter_min_cutoff: float = 2.0,
        filter_beta: float = 0.1,
        workspace_x: Tuple[float, float] = (-0.45, 0.45),
        workspace_y: Tuple[float, float] = (-0.45, 0.70),
        workspace_z: Tuple[float, float] = (0.02, 0.50),
    ):
        self.ee_frame_name = ee_frame_name
        self.joint_safety_margin_rad = joint_safety_margin_rad
        self.max_joint_velocity_rad_s = max_joint_velocity_rad_s
        self.workspace_x = workspace_x
        self.workspace_y = workspace_y
        self.workspace_z = workspace_z

        # 1. Resolve URDF path
        project_root = Path(__file__).resolve().parents[2]
        resolved_urdf = (
            Path(urdf_path).resolve()
            if urdf_path
            else (project_root / "configs" / "urdf" / "rebot_arm_rs.urdf").resolve()
        )
        if not resolved_urdf.exists():
            candidate = Path(
                "/home/bright/win_office/ubantu_files/project/reRot_DevArm/reBot-DevArm/Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf"
            ).resolve()
            if candidate.exists():
                resolved_urdf = candidate

        self.urdf_path = str(resolved_urdf)

        # 2. Build reduced 6-DOF Pinocchio model
        full_model = pin.buildModelFromUrdf(self.urdf_path)
        q_ref = pin.neutral(full_model)

        joints_to_lock = []
        for jname in ["gripper_joint1", "gripper_joint2"]:
            if full_model.existJointName(jname):
                joints_to_lock.append(full_model.getJointId(jname))

        if joints_to_lock:
            self.model = pin.buildReducedModel(full_model, joints_to_lock, q_ref)
        else:
            self.model = full_model

        self.data = self.model.createData()

        if self.model.nq != ARM_NUM_DOFS:
            raise ValueError(f"Expected 6 DOFs for arm, but model has nq={self.model.nq}")

        # Resolve end-effector frame ID
        if self.model.existFrame(self.ee_frame_name):
            self.ee_id = self.model.getFrameId(self.ee_frame_name)
        else:
            self.ee_id = self.model.nframes - 1
            self.ee_frame_name = self.model.frames[self.ee_id].name

        # 3. Soft joint limits with safety margin
        self.q_min = self.model.lowerPositionLimit.copy() + self.joint_safety_margin_rad
        self.q_max = self.model.upperPositionLimit.copy() - self.joint_safety_margin_rad

        # 4. Home and default configuration (Tuina-ready downward posture hovering above bed)
        if home_q is not None:
            self.home_q = np.array(home_q, dtype=np.float64)
        else:
            self.home_q = DEFAULT_READY_ARM_Q.copy()

        self.current_q = self.home_q.copy()
        self.current_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)

        # 5. Output 1€ Filter
        self._filter = OneEuroFilter(
            n_dim=ARM_NUM_DOFS,
            freq_hz=30.0,
            min_cutoff=filter_min_cutoff,
            beta=filter_beta,
        )

        # 6. Pink QP solver configuration if available
        self._pink_available = False
        self._qp_solver_name = "proxqp"
        if prefer_pink and _HAS_PINK:
            avail = qpsolvers.available_solvers
            if "proxqp" in avail:
                self._qp_solver_name = "proxqp"
                self._pink_available = True
            elif "osqp" in avail:
                self._qp_solver_name = "osqp"
                self._pink_available = True

        if self._pink_available:
            self._configuration = pink.Configuration(
                self.model, self.model.createData(), self.current_q.copy()
            )
            self._ee_task = FrameTask(
                self.ee_frame_name,
                position_cost=10.0,
                orientation_cost=1.0,
            )
            self._posture_task = PostureTask(cost=1e-3)
            self._tasks = [self._ee_task, self._posture_task]

        self._last_metrics = ArmIKMetrics(
            solver_mode="pink_" + self._qp_solver_name if self._pink_available else "pinocchio_dls"
        )

    @property
    def solver_mode(self) -> str:
        return self._last_metrics.solver_mode

    def clamp_workspace(self, target_pos: np.ndarray) -> np.ndarray:
        """Clamp Cartesian target coordinates inside safe workspace envelope."""
        pos = np.array(target_pos, dtype=np.float64, copy=True)
        pos[0] = np.clip(pos[0], self.workspace_x[0], self.workspace_x[1])
        pos[1] = np.clip(pos[1], self.workspace_y[0], self.workspace_y[1])
        pos[2] = np.clip(pos[2], self.workspace_z[0], self.workspace_z[1])
        return pos

    def forward_kinematics(self, q: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
        """Compute end-effector position and rotation matrix via FK.
        
        Returns:
            (pos_xyz (3,), rot_mat (3, 3))
        """
        q_in = self.current_q if q is None else np.asarray(q, dtype=np.float64)
        pin.forwardKinematics(self.model, self.data, q_in)
        pin.updateFramePlacements(self.model, self.data)
        pos = self.data.oMf[self.ee_id].translation.copy()
        rot = self.data.oMf[self.ee_id].rotation.copy()
        return pos, rot

    def compute_manipulability(self, q: Optional[np.ndarray] = None) -> Tuple[float, float, float]:
        """Compute kinematic manipulability metrics (Yoshikawa, min singular value, condition number).
        
        Returns:
            (w_yoshikawa, min_singular_value, condition_number)
        """
        q_in = self.current_q if q is None else np.asarray(q, dtype=np.float64)
        pin.computeJointJacobians(self.model, self.data, q_in)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.getFrameJacobian(
            self.model, self.data, self.ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv = float(sv[-1])
        max_sv = float(sv[0])
        cond_num = float(max_sv / max(1e-6, min_sv))
        w = float(np.prod(sv))
        return w, min_sv, cond_num

    def check_joint_limits(
        self, q: Optional[np.ndarray] = None, margin_rad: float = 0.05
    ) -> Tuple[bool, float, int]:
        """Check if any arm joint is within safety margin of physical limits.
        
        Returns:
            (is_near_limit, min_dist_rad, worst_joint_idx)
        """
        q_in = self.current_q if q is None else np.asarray(q, dtype=np.float64)
        dist_min = q_in - self.model.lowerPositionLimit
        dist_max = self.model.upperPositionLimit - q_in
        all_dists = np.minimum(dist_min, dist_max)
        worst_idx = int(np.argmin(all_dists))
        min_dist = float(all_dists[worst_idx])
        is_near = min_dist < margin_rad
        return is_near, min_dist, worst_idx

    def reset(self, q: Optional[np.ndarray] = None) -> None:
        """Reset internal configuration and temporal filter buffers."""
        reset_q = self.home_q if q is None else np.asarray(q, dtype=np.float64)
        self.current_q = reset_q.copy()
        self.current_dq = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        self._filter.reset()
        if self._pink_available:
            self._configuration.update(self.current_q.copy())

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: Optional[np.ndarray] = None,
        orientation_mode: ArmOrientationMode = ArmOrientationMode.FIXED_DOWNWARD,
        dt: float = 0.033,
        clutch_state: ClutchState = ClutchState.ENGAGED,
        timestamp: Optional[float] = None,
    ) -> TeleopArmCommand:
        """Solve 6-DOF IK for commanded target Cartesian pose.
        
        Args:
            target_pos: (3,) target translation [x, y, z] in robot base frame (meters)
            target_rot: (3, 3) optional target rotation matrix
            orientation_mode: ArmOrientationMode (FIXED_DOWNWARD, TRACK_WRIST, POSITION_ONLY)
            dt: Time step in seconds (default 0.033s = 30Hz)
            clutch_state: Current clutch state (freeze if DISENGAGED or ESTOP)
            timestamp: Monotonic timestamp
            
        Returns:
            TeleopArmCommand matching TuinaDex contract.
        """
        t_now = time.monotonic() if timestamp is None else timestamp
        t_start = time.perf_counter()

        # If clutch is disengaged or ESTOP, hold current configuration
        if clutch_state in (ClutchState.DISENGAGED, ClutchState.ESTOP):
            pos, rot = self.forward_kinematics(self.current_q)
            target_pose = np.eye(4, dtype=np.float64)
            target_pose[:3, :3] = rot
            target_pose[:3, 3] = pos
            return TeleopArmCommand(
                timestamp=t_now,
                target_pose=target_pose,
                target_q=self.current_q.copy(),
                target_dq=np.zeros(ARM_NUM_DOFS, dtype=np.float64),
                clutch_state=clutch_state,
            )

        # 1. Enforce Cartesian workspace safety boundaries
        safe_pos = self.clamp_workspace(target_pos)

        # 2. Determine target orientation matrix
        if orientation_mode == ArmOrientationMode.FIXED_DOWNWARD or target_rot is None:
            safe_rot = R_DOWNWARD_DEFAULT.copy()
        elif orientation_mode == ArmOrientationMode.POSITION_ONLY:
            _, cur_rot = self.forward_kinematics(self.current_q)
            safe_rot = cur_rot
        else:
            safe_rot = np.asarray(target_rot, dtype=np.float64)

        # Construct target SE(3) transform
        target_se3 = pin.SE3(safe_rot, safe_pos)

        # 3. Solve IK via Pink or Pinocchio DLS
        q_solved = None
        dq_solved = None
        converged = False
        iters = 1

        if self._pink_available:
            try:
                q_solved, dq_solved, converged, iters = self._solve_pink(
                    target_se3, orientation_mode, dt
                )
            except Exception:
                # Robust fallback to Pinocchio DLS
                q_solved, dq_solved, converged, iters = self._solve_dls(
                    target_se3, orientation_mode, dt
                )
        else:
            q_solved, dq_solved, converged, iters = self._solve_dls(
                target_se3, orientation_mode, dt
            )

        # 4. Joint limit clamping and velocity saturation
        delta_q = q_solved - self.current_q
        max_delta = self.max_joint_velocity_rad_s * dt
        delta_q_clamped = np.clip(delta_q, -max_delta, max_delta)
        q_saturated = self.current_q + delta_q_clamped

        # Clamp within soft joint limits
        q_clamped = np.clip(q_saturated, self.q_min, self.q_max)

        # 5. Smooth output via 1€ filter
        q_smoothed = self._filter.filter(q_clamped, timestamp=t_now)
        dq_filtered = (q_smoothed - self.current_q) / max(1e-4, dt)

        # Update state
        self.current_q = q_smoothed.copy()
        self.current_dq = dq_filtered.copy()

        # Telemetry calculation
        pos_actual, rot_actual = self.forward_kinematics(self.current_q)
        pos_err = float(np.linalg.norm(safe_pos - pos_actual))
        rot_err = float(np.linalg.norm(pin.log3(rot_actual.T @ safe_rot)))
        elapsed_ms = (time.perf_counter() - t_start) * 1000.0

        self._last_metrics = ArmIKMetrics(
            computation_time_ms=elapsed_ms,
            position_error_m=pos_err,
            orientation_error_rad=rot_err,
            is_converged=converged,
            iterations=iters,
            solver_mode="pink_" + self._qp_solver_name if self._pink_available else "pinocchio_dls",
        )

        # Construct target SE(3) matrix
        target_pose_mat = np.eye(4, dtype=np.float64)
        target_pose_mat[:3, :3] = safe_rot
        target_pose_mat[:3, 3] = safe_pos

        return TeleopArmCommand(
            timestamp=t_now,
            target_pose=target_pose_mat,
            target_q=self.current_q.copy(),
            target_dq=self.current_dq.copy(),
            clutch_state=clutch_state,
        )

    def _solve_pink(
        self,
        target_se3: pin.SE3,
        orientation_mode: ArmOrientationMode,
        dt: float,
        max_sub_iters: int = 6,
        pos_tol: float = 5e-4,
    ) -> Tuple[np.ndarray, np.ndarray, bool, int]:
        """Solve IK step using Pink QP-IK with iterative convergence."""
        # Configure task weights based on orientation mode
        if orientation_mode == ArmOrientationMode.POSITION_ONLY:
            self._ee_task.set_position_cost(10.0)
            self._ee_task.set_orientation_cost(0.0)
        elif orientation_mode in (ArmOrientationMode.TRACK_WRIST, ArmOrientationMode.TRACK_IMU):
            self._ee_task.set_position_cost(10.0)
            self._ee_task.set_orientation_cost(4.0)
        else:
            self._ee_task.set_position_cost(10.0)
            self._ee_task.set_orientation_cost(1.5)

        self._ee_task.set_target(target_se3)
        self._posture_task.set_target(self.home_q)

        q_curr = self.current_q.copy()
        velocity = np.zeros(ARM_NUM_DOFS, dtype=np.float64)
        converged = False

        for it in range(max_sub_iters):
            self._configuration.update(q_curr)
            err = self._ee_task.compute_error(self._configuration)
            pos_err = np.linalg.norm(err[:3])
            ori_err = np.linalg.norm(err[3:])
            if pos_err < pos_tol and (orientation_mode == ArmOrientationMode.POSITION_ONLY or ori_err < 0.02):
                converged = True
                break

            velocity = pink.solve_ik(
                self._configuration,
                self._tasks,
                dt,
                solver=self._qp_solver_name,
            )
            q_curr = pin.integrate(self.model, q_curr, velocity * dt)

        return q_curr, velocity, converged, it + 1

    def _solve_dls(
        self,
        target_se3: pin.SE3,
        orientation_mode: ArmOrientationMode,
        dt: float,
        max_iters: int = 8,
        damping: float = 1e-3,
        eps: float = 1e-4,
    ) -> Tuple[np.ndarray, np.ndarray, bool, int]:
        """Solve IK step using Damped Least-Squares (DLS) with null-space posture."""
        q_cur = self.current_q.copy()
        converged = False

        weight_ori = 0.0 if orientation_mode == ArmOrientationMode.POSITION_ONLY else (
            1.2 if orientation_mode in (ArmOrientationMode.TRACK_WRIST, ArmOrientationMode.TRACK_IMU) else 0.8
        )

        for it in range(max_iters):
            pin.forwardKinematics(self.model, self.data, q_cur)
            pin.updateFramePlacements(self.model, self.data)

            dMf = self.data.oMf[self.ee_id].inverse() * target_se3
            err_spatial = pin.log(dMf).vector

            # Scale orientation error component
            err_scaled = err_spatial.copy()
            err_scaled[3:] *= weight_ori

            norm_err = float(np.linalg.norm(err_scaled))
            if norm_err < eps:
                converged = True
                break

            # Frame Jacobian in LOCAL frame
            J = pin.computeFrameJacobian(
                self.model, self.data, q_cur, self.ee_id, pin.ReferenceFrame.LOCAL
            )

            # DLS inverse: J_pinv = J^T (J J^T + lambda^2 I)^(-1)
            JJT = J @ J.T + damping * np.eye(6)
            J_pinv = J.T @ np.linalg.inv(JJT)

            # Primary motion
            dq_primary = J_pinv @ err_scaled

            # Secondary null-space posture objective
            null_proj = np.eye(ARM_NUM_DOFS) - J_pinv @ J
            dq_posture = 0.08 * (self.home_q - q_cur)
            dq_step = dq_primary + null_proj @ dq_posture

            q_cur = pin.integrate(self.model, q_cur, dq_step * 0.75)

        dq_result = (q_cur - self.current_q) / max(1e-4, dt)
        return q_cur, dq_result, converged, it + 1

    @property
    def metrics(self) -> ArmIKMetrics:
        """Return diagnostic metrics from the last IK solve."""
        return self._last_metrics

    @property
    def last_metrics(self) -> ArmIKMetrics:
        """Alias for metrics."""
        return self._last_metrics

    def get_metrics(self) -> ArmIKMetrics:
        """Return the latest diagnostic telemetry."""
        return self._last_metrics
