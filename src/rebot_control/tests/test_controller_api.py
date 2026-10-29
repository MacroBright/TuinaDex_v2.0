#!/usr/bin/env python3
"""Unit tests for ReBotRSMITController API without CAN hardware."""

import math
import unittest

from rebot import ReBotRSMITController, load_config
from rebot.config import DEFAULT_CONFIG_PATH


class TestControllerAPI(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.arm = ReBotRSMITController(self.config)

    def test_initial_state_6dof(self):
        self.assertEqual(self.arm.motor_count, 6)
        self.assertEqual(self.arm.channel, "can0")
        self.assertFalse(self.arm.started)
        self.assertEqual(self.arm.get_target_angles(), [0.0] * 6)
        self.assertEqual(self.arm.get_command_angles(), [0.0] * 6)

    def test_set_joint_angles_6dof(self):
        targets = [10.0, 20.0, -15.0, 30.0, 0.0, 45.0]
        self.arm.set_joint_angles(targets)
        result = self.arm.get_target_angles()
        for expected, actual in zip(targets, result):
            self.assertAlmostEqual(expected, actual, places=4)

        # Mismatched length (expects 6, passed 7 or 3)
        with self.assertRaises(ValueError):
            self.arm.set_joint_angles([10.0, 20.0, 30.0])

        with self.assertRaises(ValueError):
            self.arm.set_joint_angles(targets + [180.0])

        # Non-finite angle (nan / inf)
        with self.assertRaises(ValueError):
            self.arm.set_joint_angles([float("nan")] + [0.0] * 5)

    def test_set_joint_angle_single(self):
        # Valid joint_id 1 to 6
        self.arm.set_joint_angle(1, 25.0)
        self.arm.set_joint_angle(6, 45.0)
        targets = self.arm.get_target_angles()
        self.assertAlmostEqual(targets[0], 25.0, places=4)
        self.assertAlmostEqual(targets[5], 45.0, places=4)

        # Out-of-bounds joint_id
        with self.assertRaises(ValueError):
            self.arm.set_joint_angle(0, 10.0)

        with self.assertRaises(ValueError):
            self.arm.set_joint_angle(7, 10.0)

        # Non-finite
        with self.assertRaises(ValueError):
            self.arm.set_joint_angle(2, float("inf"))

    def test_set_max_speeds(self):
        speeds = [15.0, 20.0, 20.0, 30.0, 30.0, 30.0]
        self.arm.set_max_speeds(speeds)
        for expected, actual_rad in zip(speeds, self.arm.max_speeds_rad_s):
            self.assertAlmostEqual(math.radians(expected), actual_rad, places=5)

        # Mismatched length
        with self.assertRaises(ValueError):
            self.arm.set_max_speeds([20.0] * 7)

        # Non-positive speed
        with self.assertRaises(ValueError):
            self.arm.set_max_speeds([20.0, 0.0, 20.0, 20.0, 20.0, 20.0])

        with self.assertRaises(ValueError):
            self.arm.set_max_speeds([-5.0] * 6)


if __name__ == "__main__":
    unittest.main()
