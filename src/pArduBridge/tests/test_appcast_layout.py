#!/usr/bin/env python3
"""Contract checks for the structured pArduBridge appcast layout."""

import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]


class AppcastLayoutTest(unittest.TestCase):
    def test_report_uses_aligned_fields_and_named_sections(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index("bool ArduBridge::buildReport()")
        stop = source.index("// Procedure: sendSetPointsToUAV()", start)
        report = source[start:stop]
        self.assertIn("constexpr int label_width = 31", report)
        self.assertIn("std::setw(label_width)", report)
        for section in (
            "Configuration Settings",
            "UAV State",
            "Command Policies",
            "GPS Details",
            "Helm Control",
        ):
            self.assertIn(f'section("{section}")', report)
        self.assertIn('"State | Measurement | Helm | Target"', report)
        self.assertIn('"Bridge Helm Authority"', report)
        self.assertIn("not pHelmIvP PARK/DRIVE", report)


if __name__ == "__main__":
    unittest.main()
