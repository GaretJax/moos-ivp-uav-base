"""Compile actual Guided admission/preemption and native ground proof, without an FC."""

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
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>
namespace mavsdk {
struct Telemetry {
  enum class FlightMode { Unknown, Offboard, Land, ReturnToLaunch, Hold };
  enum class LandedState { Unknown, OnGround, InAir, TakingOff, Landing };
};
}
using Mode = mavsdk::Telemetry::FlightMode;
using State = mavsdk::Telemetry::LandedState;
double clockNow() {
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
double moos_now = 20;
double MOOSTime() { return moos_now; }
std::string doubleToString(double x) { return std::to_string(x); }
struct Armed { bool armed = false; double received_s = clockNow(); };
struct Snapshot { Armed sample; Armed get() const { return sample; } };
struct Result { std::string command, status, detail; uint64_t id; };
class UAV_Model {
public:
  static constexpr double LANDED_STATE_TELEMETRY_MAX_AGE_S = 2;
  Snapshot mts_armed_snapshot;
  bool connected = true, armed = false, landed_available = true;
  double landed_age = 0.1;
  State landed = State::OnGround;
  Mode mode = Mode::Land;
  int attempts = 0;
  std::vector<Result> results;
  bool isFlightControllerConnected() const { return connected; }
  bool isArmed() const { return armed; }
  bool hasLandedStateTelemetry() const { return landed_available; }
  double getLandedStateTelemetryAge() const { return landed_age; }
  State getLandedState() const { return landed; }
  Mode getFlightMode() const { return mode; }
  bool isGuidedMode() const { return mode == Mode::Offboard; }
  bool hasFreshGroundedDisarmedTelemetry() const;
  bool commandEnterGuided() { ++attempts; return true; } // ACK != native mode
  uint64_t reportCommandResult(const std::string& command, const std::string& status,
                              const std::string& detail, uint64_t id=0) {
    if (!id) id=results.size()+1;
    results.push_back({command,status,detail,id}); return id;
  }
};
struct Message {
  double stamp = MOOSTime(); bool string = true;
  double GetTime() const { return stamp; }
  bool IsString() const { return string; }
};
class ArduBridge {
public:
  UAV_Model m_uav_model;
  bool m_enter_guided_pending = false;
  double m_enter_guided_started = 0, m_enter_guided_timeout = 30;
  double m_last_enter_guided_attempt = -1, m_curr_time = MOOSTime();
  double m_bridge_started = 5;
  uint64_t m_enter_guided_command_id = 0;
  bool m_do_autoland = false, m_do_return_to_launch = false;
  std::optional<Mode> m_enter_guided_retained_recovery_mode;
  void cancelEnterGuided(const std::string&);
  void admit(Message msg={});
  void poll();
};
"""
MAIN = r"""
void canceled(ArduBridge& b) {
  assert(!b.m_enter_guided_pending);
  assert(b.m_uav_model.results.back().status == "CANCELED");
  assert(b.m_uav_model.results.back().detail == "FC_LAND_OR_RTL_PREEMPTED");
  assert(!b.m_enter_guided_retained_recovery_mode);
}
int main() {
  for (Mode mode : {Mode::Land, Mode::ReturnToLaunch}) {
    ArduBridge b; b.m_uav_model.mode = mode;
    for (int task=0; task<2; ++task) {
      b.admit(); b.poll();
      assert(b.m_enter_guided_pending && b.m_uav_model.attempts == task+1);
      assert(!b.m_uav_model.isGuidedMode()); // never manufacture confirmation
      b.m_uav_model.mode = Mode::Offboard; b.poll();
      assert(!b.m_enter_guided_pending && !b.m_enter_guided_retained_recovery_mode);
      assert(b.m_uav_model.results.back().status == "ACCEPTED");
      b.m_uav_model.mode = mode;
    }
  }
  // Every uncertain/native airborne case is fail-closed, including arm-sample
  // disagreement, missing samples, invalid ages, disconnects and freshness loss.
  for (int fault=0; fault<15; ++fault) {
    for (int stage=0; stage<2; ++stage) {
      ArduBridge b;
      if (stage) b.admit();
      auto& u=b.m_uav_model;
      if (fault==0) u.connected=false;
      if (fault==1) u.armed=true;
      if (fault==2) u.mts_armed_snapshot.sample.armed=true;
      if (fault==3) u.landed_available=false;
      if (fault==4) u.landed=State::Unknown;
      if (fault==5) u.landed=State::InAir;
      if (fault==6) u.landed=State::TakingOff;
      if (fault==7) u.landed=State::Landing;
      if (fault==8) u.landed_age=2.01;
      if (fault==9) u.landed_age=-1;
      if (fault==10) u.landed_age=std::numeric_limits<double>::quiet_NaN();
      if (fault==11) u.landed_age=std::numeric_limits<double>::infinity();
      if (fault==12) u.mts_armed_snapshot.sample.received_s=-1;
      if (fault==13) u.mts_armed_snapshot.sample.received_s=clockNow()-3;
      if (fault==14) u.mts_armed_snapshot.sample.received_s=clockNow()+1;
      if (!stage) b.admit();
      b.poll(); canceled(b); assert(u.attempts==0);
    }
  }
  // New native recovery mode after the request is preemption, even on ground.
  for (Mode mode : {Mode::Land,Mode::ReturnToLaunch}) {
    ArduBridge b; b.m_uav_model.mode=Mode::Hold;
    b.admit(); b.poll(); assert(b.m_uav_model.attempts==1);
    b.m_uav_model.mode=mode; b.poll(); canceled(b);
    assert(b.m_uav_model.attempts==1);
  }
  { ArduBridge b; b.admit(); b.m_uav_model.mode=Mode::ReturnToLaunch;
    b.poll(); canceled(b); assert(b.m_uav_model.attempts==0); }
  { ArduBridge b; b.admit(); b.m_uav_model.mode=Mode::Hold; b.poll();
    assert(!b.m_enter_guided_retained_recovery_mode);
    b.m_uav_model.mode=Mode::Land; b.poll(); canceled(b); }
  for (int intent=0; intent<2; ++intent) {
    ArduBridge b; b.admit();
    if (intent==0) b.m_do_autoland=true; else b.m_do_return_to_launch=true;
    b.poll(); canceled(b); assert(b.m_uav_model.attempts==0);
  }
  for (const auto& detail : {"SUPERSEDED_BY_DEADMAN","SUPERSEDED_BY_RTL",
                            "SUPERSEDED_BY_AUTOLAND","SUPERSEDED_BY_MANUAL_OVERRIDE"}) {
    ArduBridge b; b.admit(); b.cancelEnterGuided(detail); b.poll();
    assert(!b.m_enter_guided_pending && !b.m_enter_guided_retained_recovery_mode);
    assert(b.m_uav_model.attempts==0);
  }
  for (Message msg : {Message{4,true},Message{17,true},Message{23,true},Message{20,false}}) {
    ArduBridge b; b.admit(msg); b.poll(); canceled(b);
    assert(b.m_uav_model.attempts==0);
  }
  { ArduBridge b; b.admit(); b.m_curr_time+=31; b.poll();
    assert(!b.m_enter_guided_pending && !b.m_enter_guided_retained_recovery_mode);
    assert(b.m_uav_model.results.back().status=="FAILED");
    assert(b.m_uav_model.results.back().detail=="TIMEOUT");
    assert(b.m_uav_model.attempts==0); }
}
"""


class GuidedGroundReadmissionTest(unittest.TestCase):
    def test_production_admission_preemption_and_native_proof(self):
        source = (BASE / "ArduBridge.cpp").read_text()
        begin = source.index('      else if (command == "ENTER_GUIDED")')
        end = source.index('      else if (command == "DO_TAKEOFF")', begin)
        admission = ('void ArduBridge::admit(Message msg) {\n'
                     'bool handled=false; std::string command="ENTER_GUIDED";\n'
                     'if (false) {}\n' + source[begin:end] + '(void)handled; (void)msg;\n}\n')
        begin = source.index('  // LAND/RTL may be selected directly at the FC')
        end = source.index('  /// Takeoff', begin)
        poll = 'void ArduBridge::poll() {\n' + source[begin:end] + '}\n'
        begin = source.index('void ArduBridge::cancelEnterGuided(')
        end = source.index('void ArduBridge::handleYawAlignmentRequest(', begin)
        cancel = source[begin:end]
        model = (BASE / "UAV_Model.cpp").read_text()
        begin = model.index('bool UAV_Model::hasFreshGroundedDisarmedTelemetry()')
        end = model.index('double UAV_Model::getLandedStateTelemetryAge()', begin)
        with tempfile.TemporaryDirectory(prefix="guided-ground-readmission-") as temp:
            cpp = Path(temp) / "test.cpp"
            cpp.write_text(PRELUDE + model[begin:end] + admission + poll + cancel + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                            "-Werror", str(cpp), "-o", str(binary)], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
