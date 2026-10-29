#!/usr/bin/env python3
"""Unit tests for rebot.config module."""

import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile

from rebot.config import (
    DEFAULT_CONFIG_PATH,
    DEFAULT_MOTORS,
    DEFAULT_MOTORS_7DOF,
    GRIPPER_JOINT_ID,
    GRIPPER_MOTOR_ID,
    ControllerConfig,
    MotorConfig,
    ReturnZeroConfig,
    TemperatureThresholds,
    load_config,
)


class TestConfig(unittest.TestCase):
    def test_default_config_path_exists(self):
        self.assertTrue(
            DEFAULT_CONFIG_PATH.is_file(),
            f"Default config not found at {DEFAULT_CONFIG_PATH}",
        )

    def test_load_default_6dof_config(self):
        cfg = load_config()
        self.assertEqual(cfg.channel, "can0")
        self.assertEqual(cfg.host_id, 0xFD)
        self.assertEqual(cfg.control_hz, 200.0)
        self.assertEqual(cfg.telemetry_hz, 2.0)
        self.assertEqual(cfg.default_speed_deg_s, 30.0)
        self.assertEqual(len(cfg.motors), 6)

        # J1-J3 RS06, J4-J6 RS00
        for i in range(3):
            self.assertEqual(cfg.motors[i].model, "rs-06")
            self.assertEqual(cfg.motors[i].motor_id, i + 1)
        for i in range(3, 6):
            self.assertEqual(cfg.motors[i].model, "rs-00")
            self.assertEqual(cfg.motors[i].motor_id, i + 1)

    def test_load_7dof_config(self):
        config_7dof_path = DEFAULT_CONFIG_PATH.parent / "rebotarm_rs_7dof.yaml"
        if config_7dof_path.is_file():
            cfg7 = load_config(config_7dof_path)
            self.assertEqual(len(cfg7.motors), 7)
            self.assertEqual(cfg7.motors[6].motor_id, 7)
            self.assertEqual(cfg7.motors[6].model, "rs-00")

    def test_temperature_thresholds_validation(self):
        # Valid: alarm < return_zero < disconnect
        t = TemperatureThresholds(alarm_c=80.0, return_zero_c=125.0, disconnect_c=140.0)
        self.assertEqual(t.alarm_c, 80.0)
        self.assertEqual(t.return_zero_c, 125.0)
        self.assertEqual(t.disconnect_c, 140.0)

        # Invalid order: alarm >= return_zero
        with self.assertRaises(ValueError):
            TemperatureThresholds(alarm_c=130.0, return_zero_c=125.0, disconnect_c=140.0)

        # Invalid order: return_zero >= disconnect
        with self.assertRaises(ValueError):
            TemperatureThresholds(alarm_c=80.0, return_zero_c=145.0, disconnect_c=140.0)

    def test_controller_config_validation(self):
        with self.assertRaises(ValueError):
            ControllerConfig(control_hz=0.0)

        with self.assertRaises(ValueError):
            ControllerConfig(telemetry_hz=-1.0)

        with self.assertRaises(ValueError):
            ControllerConfig(default_speed_deg_s=0.0)

        with self.assertRaises(ValueError):
            ControllerConfig(motors=())

    def test_yaml_unknown_key_rejection(self):
        invalid_yaml = """
temperatures:
  alarm_c: 80.0
  return_zero_c: 125.0
  disconnect_c: 140.0
  unknown_typo_key: 123.0
"""
        with NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            f.write(invalid_yaml)
            tmp_path = f.name

        try:
            with self.assertRaises(ValueError):
                ControllerConfig.from_yaml(tmp_path)
        finally:
            Path(tmp_path).unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
