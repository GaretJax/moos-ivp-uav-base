"""Compile the production park handler, ground proof and armed callback unchanged.

Controlled native telemetry doubles only; no MOOS/FC or live flight commands.
"""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]

PRELUDE = r"""
#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

namespace mavsdk {
struct Telemetry {
  enum class LandedState { Unknown, OnGround, InAir, TakingOff, Landing };
};
}
using LandedState = mavsdk::Telemetry::LandedState;
double clockNow() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ArmedSample { bool armed = false; double received_s = -1; };
struct Snapshot {
  ArmedSample sample;
  ArmedSample get() const { return sample; }
  void set(ArmedSample value) { sample = value; }
};
struct TelemetryStub {
  std::function<void(bool)> callback;
  void subscribe_armed(std::function<void(bool)> value) { callback = value; }
};
class UAV_Model {
public:
  static constexpr double LANDED_STATE_TELEMETRY_MAX_AGE_S = 2;
  Snapshot mts_armed_snapshot;
  bool connected = true, m_is_armed = false, available = true;
  double age = 0.1;
  LandedState state = LandedState::OnGround;
  std::shared_ptr<TelemetryStub> m_telemetry_ptr = std::make_shared<TelemetryStub>();
  bool isFlightControllerConnected() const { return connected; }
  bool isArmed() const { return m_is_armed; }
  bool hasLandedStateTelemetry() const { return available; }
  double getLandedStateTelemetryAge() const { return age; }
  LandedState getLandedState() const { return state; }
  bool hasFreshGroundedDisarmedTelemetry() const;
  void subscribeArmedTest();
  void receiveArmed(bool armed) { m_telemetry_ptr->callback(armed); }
};
struct Message {
  std::string value = "true", source = "pFSMRuntime";
  std::string GetString() const { return value; }
  std::string GetSource() const { return source; }
};
struct Logger { static void info(const std::string&) {} };
enum class AutopilotHelmMode { HELM_PARKED };
class ArduBridge {
public:
  UAV_Model m_uav_model;
  bool m_force_native_rtl = false, m_do_return_to_launch = false;
  unsigned parks = 0, events = 0;
  std::string GetAppName() const { return "pArduBridge"; }
  void reportEvent(const std::string&) { ++events; }
  void cancelEnterGuided(const std::string&) {}
  void goToHelmMode(AutopilotHelmMode, bool) { ++parks; }
  void handle(Message msg);
};
"""

MAIN = r"""
int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string test = argv[1];
  ArduBridge b;
  b.m_uav_model.subscribeArmedTest();
  b.m_uav_model.receiveArmed(false);
  if (test == "grounded") {
    assert(b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
    b.handle({});
    b.handle({}); // repeated FSM parking is idempotent, not another RTL
    assert(b.parks == 2 && b.events == 2 && !b.m_do_return_to_launch);
  } else if (test == "armed") {
    b.m_uav_model.receiveArmed(true);
    assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
    b.handle({});
    assert(b.parks == 1 && b.m_do_return_to_launch);
  } else if (test == "states") {
    for (auto state : {LandedState::Unknown, LandedState::InAir,
                       LandedState::TakingOff, LandedState::Landing}) {
      b.m_uav_model.state = state;
      b.m_do_return_to_launch = false;
      assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
      b.handle({});
      assert(b.m_do_return_to_launch);
    }
    assert(b.parks == 4);
  } else if (test == "invalid_landed") {
    for (double age : {-1.0, 2.01, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
      b.m_uav_model.age = age;
      b.m_do_return_to_launch = false;
      b.handle({});
      assert(b.m_do_return_to_launch);
    }
    b.m_uav_model.age = 0.1;
    b.m_uav_model.available = false;
    b.m_do_return_to_launch = false;
    b.handle({});
    assert(b.m_do_return_to_launch);
    b.m_uav_model.available = true;
    b.m_uav_model.connected = false;
    assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
  } else if (test == "invalid_armed") {
    for (double stamp : {-1.0, clockNow() - 2.01, clockNow() + 1,
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
      b.m_uav_model.mts_armed_snapshot.sample.received_s = stamp;
      b.m_do_return_to_launch = false;
      b.handle({});
      assert(b.m_do_return_to_launch);
    }
    b.m_uav_model.receiveArmed(false);
    b.m_uav_model.m_is_armed = true; // concurrent newer arm indication wins
    assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
  } else if (test == "pending_rtl") {
    b.m_do_return_to_launch = true;
    b.m_force_native_rtl = true; // an independent pending emergency request
    b.handle({});
    assert(b.parks == 1 && b.m_do_return_to_launch && b.m_force_native_rtl);
  } else if (test == "self_and_false") {
    b.m_uav_model.state = LandedState::InAir;
    b.handle({"true", "pArduBridge"});
    assert(b.parks == 1 && !b.m_do_return_to_launch);
    b.handle({"false", "pFSMRuntime"});
    assert(b.parks == 1 && !b.m_do_return_to_launch);
  } else if (test == "callback") {
    b.m_uav_model.mts_armed_snapshot.sample.received_s = -1;
    assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
    b.m_uav_model.receiveArmed(false);
    assert(b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
    b.m_uav_model.mts_armed_snapshot.sample.received_s = clockNow() - 10;
    b.m_uav_model.receiveArmed(false); // unchanged heartbeat still renews proof
    assert(b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
    b.m_uav_model.receiveArmed(true);
    assert(!b.m_uav_model.hasFreshGroundedDisarmedTelemetry());
  } else return 1;
}
"""


class GroundedParkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="grounded-park-")
        cls.addClassCleanup(cls.directory.cleanup)
        model = (BASE / "UAV_Model.cpp").read_text()
        start = model.index("bool UAV_Model::hasFreshGroundedDisarmedTelemetry()")
        stop = model.index("double UAV_Model::getLandedStateTelemetryAge()", start)
        proof = model[start:stop]
        start = model.index("  m_telemetry_ptr->subscribe_armed(")
        stop = model.index("  m_telemetry_ptr->subscribe_health_all_ok(", start)
        callback = (
            "void UAV_Model::subscribeArmedTest() {\n" + model[start:stop] + "}\n"
        )
        bridge = (BASE / "ArduBridge.cpp").read_text()
        start = bridge.index('    else if (key == "MOOS_MANUAL_OVERRIDE")')
        stop = bridge.index('    else if (key == "AUTOPILOT_MODE")', start)
        handler = (
            "void ArduBridge::handle(Message msg) {\n"
            'const std::string key = "MOOS_MANUAL_OVERRIDE";\n'
            + bridge[start:stop].replace("else if", "if", 1)
            + "}\n"
        )
        source = Path(cls.directory.name) / "test.cpp"
        source.write_text(PRELUDE + proof + callback + handler + MAIN)
        cls.binary = source.with_suffix("")
        subprocess.run(
            [
                os.environ.get("CXX", "c++"),
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                str(source),
                "-o",
                str(cls.binary),
            ],
            check=True,
            timeout=30,
        )

    def test_production_park_path(self):
        for case in (
            "grounded",
            "armed",
            "states",
            "invalid_landed",
            "invalid_armed",
            "pending_rtl",
            "self_and_false",
            "callback",
        ):
            with self.subTest(case=case):
                subprocess.run([str(self.binary), case], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
