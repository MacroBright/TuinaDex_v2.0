#!/usr/bin/env python3
"""Unit tests for smoothstep and trajectory math."""

import math
import unittest

from rebot import ReBotRSMITController


class TestTrajectoryMath(unittest.TestCase):
    def test_smoothstep_bounds(self):
        # Alpha <= 0 -> 0
        self.assertEqual(ReBotRSMITController._smoothstep(-0.5), 0.0)
        self.assertEqual(ReBotRSMITController._smoothstep(0.0), 0.0)

        # Alpha >= 1 -> 1
        self.assertEqual(ReBotRSMITController._smoothstep(1.0), 1.0)
        self.assertEqual(ReBotRSMITController._smoothstep(1.5), 1.0)

        # Midpoint
        self.assertAlmostEqual(ReBotRSMITController._smoothstep(0.5), 0.5)

    def test_smoothstep_monotonicity(self):
        steps = 100
        values = [
            ReBotRSMITController._smoothstep(i / steps) for i in range(steps + 1)
        ]
        for a, b in zip(values[:-1], values[1:]):
            self.assertLessEqual(a, b)


if __name__ == "__main__":
    unittest.main()

