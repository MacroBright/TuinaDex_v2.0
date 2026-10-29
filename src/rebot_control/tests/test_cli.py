#!/usr/bin/env python3
"""Unit tests for rebot.cli module."""

import unittest
from unittest.mock import MagicMock

from rebot.cli import ReBotShell, main
from rebot.config import load_config
from rebot.controller import ReBotRSMITController


class TestCLI(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.arm = ReBotRSMITController(self.config)
        self.shell = ReBotShell(self.arm, default_speed=15.0)

    def test_shell_set_joint(self):
        # Valid joint
        self.shell.do_set("1 25.0")
        self.assertAlmostEqual(self.arm.get_target_angles()[0], 25.0, places=3)

        self.shell.do_set("6 -15.5")
        self.assertAlmostEqual(self.arm.get_target_angles()[5], -15.5, places=3)

        # Invalid joint ID (> 6)
        self.shell.do_set("7 10.0")
        # J1 should remain 25, J6 remain -15.5
        self.assertAlmostEqual(self.arm.get_target_angles()[0], 25.0, places=3)

    def test_shell_step_joint(self):
        self.arm.set_joint_angle(2, 10.0)
        self.shell.do_step("2 5.0")
        self.assertAlmostEqual(self.arm.get_target_angles()[1], 15.0, places=3)

        self.shell.do_step("2 -8.0")
        self.assertAlmostEqual(self.arm.get_target_angles()[1], 7.0, places=3)

    def test_shell_all_joints(self):
        expected = [10.0, 15.0, 20.0, -10.0, -20.0, 30.0]
        self.shell.do_all("10 15 20 -10 -20 30")
        targets = self.arm.get_target_angles()
        for exp, act in zip(expected, targets):
            self.assertAlmostEqual(exp, act, places=4)

        # Wrong count should not update
        self.shell.do_all("10 20 30")
        for exp, act in zip(expected, self.arm.get_target_angles()):
            self.assertAlmostEqual(exp, act, places=4)

    def test_shell_zero(self):
        self.arm.set_joint_angles([10, 20, 30, 40, 50, 60])
        self.shell.do_zero("")
        self.assertEqual(self.arm.get_target_angles(), [0.0] * 6)

    def test_shell_speed(self):
        self.shell.do_speed("30.0")
        for s in self.arm.max_speeds_rad_s:
            self.assertAlmostEqual(s, 30.0 * 3.141592653589793 / 180.0, places=4)

        self.shell.do_speed("1 45.0")
        self.assertAlmostEqual(self.arm.max_speeds_rad_s[0], 45.0 * 3.141592653589793 / 180.0, places=4)

    def test_shell_disable_and_enable(self):
        self.shell.do_disable("")
        self.assertTrue(self.arm.control_paused)

        self.shell.do_enable("")
        self.assertFalse(self.arm.control_paused)

    def test_shell_watch_invalid_joint(self):
        # Invalid joint ID
        self.assertFalse(self.shell.do_watch("7"))
        self.assertFalse(self.shell.do_watch("abc"))


if __name__ == "__main__":
    unittest.main()
