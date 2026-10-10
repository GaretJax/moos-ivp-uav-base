#!/usr/bin/env python3
"""Contract checks for simulation-only runtime speed control."""

import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]


class SitlSpeedupTest(unittest.TestCase):
    def test_request_is_simulation_only_and_bounded(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index('else if (key == "SITL_SPEEDUP")')
        stop = source.index('else if (key == "DEAD_MAN_POST_INTERRUPT")', start)
        handler = source[start:stop]
        self.assertIn("if (!m_is_simulation)", handler)
        self.assertIn('"SIMULATION_ONLY"', handler)
        self.assertIn("speedup < 1.0", handler)
        self.assertIn("speedup > 100.0", handler)
        self.assertIn('"REQUIRE_1_TO_100"', handler)

    def test_success_is_reported_only_after_parameter_write(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index("if (m_sitl_speedup_request)")
        stop = source.index("pollBrakeAndHold();", start)
        dispatch = source[start:stop]
        self.assertIn('setFloatParameter(\n        "SIM_SPEEDUP"', dispatch)
        self.assertIn('accepted ? "ACCEPTED" : "FAILED"', dispatch)
        self.assertIn('accepted ? "PARAMETER_SET" : "PARAMETER_SET_FAILED"', dispatch)
        self.assertIn('if (accepted)\n      Notify(m_uav_prefix + "_SITL_SPEEDUP"', dispatch)
        self.assertIn("m_sitl_speedup_request.reset()", dispatch)


if __name__ == "__main__":
    unittest.main()
