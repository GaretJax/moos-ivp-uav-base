#!/usr/bin/env python3
"""Compile the production yaw command against an FC-ACK/telemetry test double."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]
PRELUDE = r"""
#include <cassert>
#include <cmath>
#include <memory>
#include <initializer_list>
namespace mavsdk {
struct Telemetry { enum class LandedState { Unknown, OnGround, InAir }; };
struct MavlinkPassthrough {
  struct CommandLong { int command{}, target_sysid{}, target_compid{}; float param1{}, param2{}, param3{}, param4{}; };
  enum class Result { Success, Failure };
  Result result = Result::Success;
  int sends = 0;
  CommandLong last{};
  Result send_command_long(CommandLong command) { ++sends; last = command; return result; }
};
}
constexpr int MAV_CMD_CONDITION_YAW = 115, MAV_COMP_ID_AUTOPILOT1 = 1;
struct System { int get_system_id() { return 17; } };
struct UAV_Model {
  static constexpr double LANDED_STATE_TELEMETRY_MAX_AGE_S = 2;
  std::unique_ptr<mavsdk::MavlinkPassthrough> m_mavPass_ptr{new mavsdk::MavlinkPassthrough};
  std::unique_ptr<System> m_system_ptr{new System};
  bool copter = true, connected = true, guided = true, armed = true, received = true;
  mavsdk::Telemetry::LandedState landed = mavsdk::Telemetry::LandedState::InAir;
  double age = 0.1, m_target_course = 50;
  bool m_yaw_alignment_hold = false;
  bool isCopter() { return copter; }
  bool isFlightControllerConnected() { return connected; }
  bool isGuidedMode() { return guided; }
  bool isArmed() { return armed; }
  bool hasLandedStateTelemetry() { return received; }
  auto getLandedState() { return landed; }
  double getLandedStateTelemetryAge() { return age; }
  bool commandCopterYawAlignment(double, double);
};
"""
MAIN = r"""
int main() {
  UAV_Model model;
  assert(model.commandCopterYawAlignment(280, 30));
  auto &transport = *model.m_mavPass_ptr;
  assert(transport.sends == 1 && transport.last.command == 115);
  assert(transport.last.target_sysid == 17 && transport.last.target_compid == 1);
  assert(transport.last.param1 == 280 && transport.last.param2 == 30);
  assert(transport.last.param3 == 0 && transport.last.param4 == 0);
  assert(model.m_target_course == 280); // subsequent Guided hold keeps the accepted goal
  assert(model.m_yaw_alignment_hold); // refresh must leave the latched XYZ/FIXED-yaw hold alone
  transport.result = mavsdk::MavlinkPassthrough::Result::Failure;
  assert(!model.commandCopterYawAlignment(90, 30));
  assert(model.m_target_course == 280); // rejection never substitutes a desired heading
  transport.result = mavsdk::MavlinkPassthrough::Result::Success;
  for (bool *flag : {&model.copter, &model.connected, &model.guided, &model.armed, &model.received}) {
    *flag = false;
    const int sends = transport.sends;
    assert(!model.commandCopterYawAlignment(90, 30));
    assert(transport.sends == sends);
    *flag = true;
  }
  for (auto state : {mavsdk::Telemetry::LandedState::OnGround, mavsdk::Telemetry::LandedState::Unknown}) {
    model.landed = state;
    assert(!model.commandCopterYawAlignment(90, 30));
  }
  model.landed = mavsdk::Telemetry::LandedState::InAir;
  for (double age : {3.0, -1.0, double(NAN), double(INFINITY)}) {
    model.age = age;
    assert(!model.commandCopterYawAlignment(90, 30));
  }
  model.age = 0.1;
  for (double heading : {-1.0, 360.0, double(INFINITY), double(NAN)})
    assert(!model.commandCopterYawAlignment(heading, 30));
  for (double speed : {-1.0, 0.0, 91.0, double(INFINITY), double(NAN)})
    assert(!model.commandCopterYawAlignment(90, speed));
}
"""


class YawCommandTest(unittest.TestCase):
    def test_native_admission_fc_ack_and_hold_heading(self):
        source = (BASE / "src/pArduBridge/UAV_Model.cpp").read_text()
        start = source.index("bool UAV_Model::commandCopterYawAlignment(")
        stop = source.index("bool UAV_Model::refreshCopterGuidedHold(", start)
        with tempfile.TemporaryDirectory(prefix="yaw-command-") as temp:
            cpp = Path(temp) / "test.cpp"
            cpp.write_text(PRELUDE + source[start:stop] + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                            "-Werror", str(cpp), "-o", str(binary)], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
