"""Unit tests for scripts/calibrate_imu.py interactive calibration routine."""
import importlib.util
from pathlib import Path
import tempfile
import numpy as np
import pytest
import yaml

from tuinadex_teleop.perception.imu_stream import parse_axis_mapping

# Import calibrate_imu module dynamically from scripts
_SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
_CALIB_SCRIPT = _SCRIPTS_DIR / "calibrate_imu.py"

spec = importlib.util.spec_from_file_location("calibrate_imu", _CALIB_SCRIPT)
calibrate_imu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibrate_imu)


def test_average_quaternions_identity():
    """Verify quaternion averaging produces normalized result."""
    q_id = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    quats = [q_id, q_id, q_id]
    avg = calibrate_imu.average_quaternions(quats)
    np.testing.assert_allclose(avg, q_id, atol=1e-6)


def test_run_calibration_mock_auto_flow():
    """Verify run_calibration in mock auto mode produces valid SO(3) mapping and config files."""
    with tempfile.TemporaryDirectory() as tmpdir:
        session_yaml = Path(tmpdir) / "session_calib.yaml"
        imu_yaml = Path(tmpdir) / "imu_calib.yaml"

        # Prepopulate dummy session yaml with table/hand keys
        session_yaml.write_text("table:\n  is_valid: true\nhand:\n  is_valid: true\n")

        mapping_str, neutral_quat = calibrate_imu.run_calibration(
            mock=True,
            auto=True,
            session_yaml_path=session_yaml,
            imu_yaml_path=imu_yaml,
        )

        assert isinstance(mapping_str, str)
        R = parse_axis_mapping(mapping_str)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-4), "Mapping must be proper SO(3) rotation"

        assert len(neutral_quat) == 4
        assert np.isclose(np.linalg.norm(neutral_quat), 1.0, atol=1e-4)

        # Verify files were written and structure preserved
        assert session_yaml.exists()
        assert imu_yaml.exists()

        with open(session_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            assert "table" in data, "Existing table section must be preserved"
            assert "hand" in data, "Existing hand section must be preserved"
            assert "imu" in data
            assert data["imu"]["is_valid"] is True
            assert data["imu"]["imu_axis_mapping"] == mapping_str
            assert len(data["imu"]["neutral_quat_wxyz"]) == 4

        with open(imu_yaml, "r", encoding="utf-8") as f:
            imu_data = yaml.safe_load(f)
            assert imu_data["is_valid"] is True
            assert imu_data["imu_axis_mapping"] == mapping_str
