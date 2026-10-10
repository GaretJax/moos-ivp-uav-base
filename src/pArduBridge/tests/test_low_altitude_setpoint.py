#!/usr/bin/env python3
"""Regression contract for low-altitude Copter Helm setpoint admission."""

import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]


class LowAltitudeSetpointTest(unittest.TestCase):
    def test_native_landed_state_replaces_altitude_threshold(self):
        source = (BASE / "src/pArduBridge/UAV_Model.cpp").read_text()
        start = source.index("bool UAV_Model::commandCopterHelmSetpoint(")
        stop = source.index("uint64_t UAV_Model::reportCommandResult(", start)
        method = source[start:stop]

        self.assertNotIn("!m_in_air", method)
        self.assertIn("hasLandedStateTelemetry()", method)
        self.assertIn("std::isfinite(landed_age_s)", method)
        self.assertIn("landed_age_s >= 0.0", method)
        self.assertIn("landed_age_s <= LANDED_STATE_TELEMETRY_MAX_AGE_S", method)
        self.assertIn("Telemetry::LandedState::InAir", method)
        self.assertIn("if (!isGuidedMode())", method)

    def test_ground_landing_and_uncertain_states_are_not_admitted(self):
        source = (BASE / "src/pArduBridge/UAV_Model.cpp").read_text()
        start = source.index("bool UAV_Model::commandCopterHelmSetpoint(")
        stop = source.index("uint64_t UAV_Model::reportCommandResult(", start)
        method = source[start:stop]

        # The allowlist is exactly native InAir; OnGround, TakingOff, Landing,
        # Unknown, missing, stale, negative, and non-finite evidence all fail it.
        self.assertEqual(method.count("Telemetry::LandedState::InAir"), 1)
        self.assertIn("if (!isCopter() || !airborne)", method)
        self.assertIn("fresh airborne Copter telemetry", method)


if __name__ == "__main__":
    unittest.main()
