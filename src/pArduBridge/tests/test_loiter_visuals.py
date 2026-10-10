#!/usr/bin/env python3
"""Contract checks for display-only loiter marker suppression."""

import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]


class LoiterVisualsTest(unittest.TestCase):
    def test_configuration_defaults_on_and_only_gates_visualization(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        self.assertIn("m_show_loiter_visuals{true}", source)
        self.assertIn('param == "show_loiter_visuals"', source)
        self.assertIn("setBooleanOnString(m_show_loiter_visuals, value)", source)

        start = source.index("void ArduBridge::visualizeLoiterLocation(")
        stop = source.index("void ArduBridge::visualizeCourseWaypoint(", start)
        method = source[start:stop]
        self.assertIn("visualize = visualize && m_show_loiter_visuals", method)
        self.assertIn("marker.set_active(visualize)", method)
        self.assertNotIn("commandLoiter", method)
        self.assertNotIn("goToHelmMode", method)


if __name__ == "__main__":
    unittest.main()
