#!/usr/bin/env python3
"""Exercise ENTER_GUIDED cancellation and verify its preemption wiring."""

import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]
SOURCE = BASE / "src/pArduBridge/ArduBridge.cpp"

PRELUDE = r"""
#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct Result {
  std::string command, status, detail;
  uint64_t id;
};
struct Model {
  std::vector<Result> results;
  void reportCommandResult(const std::string &command, const std::string &status,
                           const std::string &detail, uint64_t id) {
    results.push_back({command, status, detail, id});
  }
};
class ArduBridge {
public:
  Model m_uav_model;
  bool m_enter_guided_pending = false;
  std::optional<int> m_enter_guided_retained_recovery_mode;
  double m_last_enter_guided_attempt = 5;
  uint64_t m_enter_guided_command_id = 17;
  void cancelEnterGuided(const std::string &detail);
};
"""

MAIN = r"""
int main() {
  ArduBridge bridge;
  bridge.cancelEnterGuided("NO_PENDING");
  assert(bridge.m_uav_model.results.empty());
  assert(bridge.m_last_enter_guided_attempt == 5);

  bridge.m_enter_guided_pending = true;
  bridge.m_enter_guided_retained_recovery_mode = 9;
  bridge.cancelEnterGuided("SUPERSEDED_BY_RTL");
  assert(!bridge.m_enter_guided_retained_recovery_mode);
  assert(!bridge.m_enter_guided_pending);
  assert(bridge.m_last_enter_guided_attempt == -1);
  assert(bridge.m_uav_model.results.size() == 1);
  const auto &result = bridge.m_uav_model.results.front();
  assert(result.command == "ENTER_GUIDED");
  assert(result.status == "CANCELED");
  assert(result.detail == "SUPERSEDED_BY_RTL");
  assert(result.id == 17);

  bridge.cancelEnterGuided("DUPLICATE");
  assert(bridge.m_uav_model.results.size() == 1);
}
"""


class EnterGuidedPreemptionTest(unittest.TestCase):
    def test_cancellation_reports_once_and_releases_retry(self):
        source = SOURCE.read_text()
        start = source.index("void ArduBridge::cancelEnterGuided(")
        stop = source.index("void ArduBridge::handleYawAlignmentRequest(", start)
        with tempfile.TemporaryDirectory(prefix="enter-guided-preemption-") as temp:
            cpp = Path(temp) / "test.cpp"
            cpp.write_text(PRELUDE + source[start:stop] + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run(
                [os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                 "-Werror", str(cpp), "-o", str(binary)],
                check=True,
                timeout=30,
            )
            subprocess.run([str(binary)], check=True, timeout=5)

    def test_newer_intents_and_native_fc_modes_preempt(self):
        source = SOURCE.read_text()
        mail_start = source.index("bool ArduBridge::OnNewMail(")
        mail_stop = source.index("bool ArduBridge::OnConnectToServer(", mail_start)
        mail = source[mail_start:mail_stop]
        reasons = set(re.findall(r'cancelEnterGuided\("([A-Z0-9_]+)"', mail))
        self.assertTrue({
            "SUPERSEDED_BY_HELM_STATUS_OFF",
            "SUPERSEDED_BY_MANUAL_OVERRIDE",
            "SUPERSEDED_BY_AUTOPILOT_MODE",
            "SUPERSEDED_BY_ARM_REQUEST",
            "SUPERSEDED_BY_AUTOLAND",
            "SUPERSEDED_BY_RTL",
            "SUPERSEDED_BY_DEADMAN",
        }.issubset(reasons))
        for command in (
            "FLY_WAYPOINT", "DO_TAKEOFF", "RETURN_TO_LAUNCH", "RETURN",
            "LOITER", "HOLD_POSITION", "LOITER_FC", "PRECISION_LOITER",
            "SURVEY", "DO_VORONOI", "AUTOLAND",
        ):
            self.assertIn(f'"{command}"', mail)

        iterate_start = source.index("bool ArduBridge::Iterate(")
        iterate_stop = source.index("bool ArduBridge::OnStartUp(", iterate_start)
        iterate = source[iterate_start:iterate_stop]
        self.assertIn("Telemetry::FlightMode::Land", iterate)
        self.assertIn("Telemetry::FlightMode::ReturnToLaunch", iterate)
        self.assertIn('cancelEnterGuided("FC_LAND_OR_RTL_PREEMPTED")', iterate)

    def test_confirmed_guided_mode_is_published_for_readiness_checks(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        self.assertIn(
            'postBool(m_uav_prefix + "_GUIDED_MODE_ACTIVE", m_uav_model.isGuidedMode())',
            source,
        )


if __name__ == "__main__":
    unittest.main()
