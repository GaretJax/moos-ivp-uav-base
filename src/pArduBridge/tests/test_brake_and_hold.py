"""Compile the production brake arbiter against native telemetry/transport doubles.

No MOOS/FC connection. Exercises packet ordering, admission, distinct fresh
native evidence, failure, preemption and suppression of late Helm commands.
"""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
PRELUDE = r"""
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
namespace mavsdk {
struct Telemetry {
  struct Position { double latitude_deg{}, longitude_deg{}; float absolute_altitude_m{}, relative_altitude_m{}; };
  struct VelocityNed { float north_m_s{}, east_m_s{}, down_m_s{}; };
  enum class LandedState { Unknown, OnGround, InAir, TakingOff, Landing };
  enum class FlightMode { Guided, Land, ReturnToLaunch, Loiter };
};
}
template <typename T> struct ThreadSafeVariable {
  T value; T get() const { return value; } void set(T v) { value = v; }
};
struct XYPoint { double lat{}, lon{}; XYPoint() = default; XYPoint(double a, double b): lat(a),lon(b) {} };
struct mavlink_global_position_int_t {
  uint32_t time_boot_ms{}; int32_t lat{},lon{},alt{},relative_alt{}; int16_t vx{},vy{},vz{};
};
struct mavlink_message_t { int sysid=17, compid=1; mavlink_global_position_int_t raw; };
void mavlink_msg_global_position_int_decode(const mavlink_message_t* m, mavlink_global_position_int_t* raw) { *raw=m->raw; }
constexpr int MAVLINK_MSG_ID_GLOBAL_POSITION_INT=33, MAV_COMP_ID_AUTOPILOT1=1;
struct System { int get_system_id() const { return 17; } };
struct Transport {
  std::function<void(const mavlink_message_t&)> receive;
  void subscribe_message(int, std::function<void(const mavlink_message_t&)> cb) { receive=cb; }
};
double clockNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
class UAV_Model {
public:
"""
STUB = r"""
  static constexpr double LANDED_STATE_TELEMETRY_MAX_AGE_S=2;
  struct ArmedTelemetry { bool armed=true; double received_s=clockNow(); };
  ThreadSafeVariable<ArmedTelemetry> mts_armed_snapshot{ArmedTelemetry()};
  ThreadSafeVariable<MotionTelemetry> mts_motion_snapshot{MotionTelemetry()};
  std::recursive_mutex mutable m_brake_mutex;
  std::mutex m_queue_mutex;
  std::queue<int> m_command_queue;
  std::condition_variable m_thread_cv;
  std::atomic<bool> m_command_executing{false};
  BrakeStatus m_brake_status;
  std::atomic<bool> m_brake_running{false};
  double m_brake_started_s=-1, m_brake_altitude_agl_m=0, m_motion_epoch_s=-1;
  std::atomic<double> m_last_landed_state_update_s{clockNow()};
  uint32_t m_brake_last_boot_ms=0;
  std::string brakeAdmissionReason(double) const;
  bool serviceBrakeAndHold(double);
  bool commandCopterHelmSetpoint(double,double,double);
  bool copter=true, connected=true, guided=true, armed=true, received=true;
  bool velocity_ok=true, position_ok=true;
  double landed_age=0.1;
  mavsdk::Telemetry::LandedState landed=mavsdk::Telemetry::LandedState::InAir;
  mavsdk::Telemetry::FlightMode mode=mavsdk::Telemetry::FlightMode::Guided;
  ThreadSafeVariable<XYPoint> mts_current_loiter_coord{};
  double m_last_sent_altitudeAGL=0, m_target_altitudeAGL=0;
  std::unique_ptr<System> m_system_ptr{new System};
  std::unique_ptr<Transport> m_mavPass_ptr{new Transport};
  struct Outcome { std::string command, status, reason; uint64_t id; };
  std::vector<Outcome> outcomes;
  std::vector<char> packets;
  mavsdk::Telemetry::Position hold{};
  uint64_t next_id=1;
  uint64_t reportCommandResult(const std::string& c,const std::string& s,const std::string& r,uint64_t id=0) {
    if (!id) id=next_id++;
    outcomes.push_back({c,s,r,id}); return id;
  }
  bool isCopter() const { return copter; }
  bool isFlightControllerConnected() const { return connected; }
  bool isGuidedMode() const { return guided; }
  bool isArmed() const { return armed; }
  bool hasLandedStateTelemetry() const { return received; }
  double getLandedStateTelemetryAge() const { return landed_age; }
  auto getLandedState() const { return landed; }
  auto getFlightMode() const { return mode; }
  bool sendCopterVelocitySetpoint(double course,double speed,double alt) {
    (void)course; assert(alt==8); packets.push_back(speed==0?'Z':'V'); return velocity_ok;
  }
  bool sendCopterPositionSetpoint(const mavsdk::Telemetry::Position& p,double yaw,bool ignore_yaw) {
    assert(yaw==0 && ignore_yaw); packets.push_back('P'); hold=p; return position_ok;
  }
  void subscribeMotion();
  void seed(double north=2.6) {
    MotionTelemetry m; m.boot_ms=100; m.received_s=clockNow();
    m.position={42,-71,9,8}; m.velocity={float(north),0,0}; mts_motion_snapshot.set(m);
    mts_armed_snapshot.set({true,clockNow()});
  }
  void sample(double speed, bool advance=true) {
    std::lock_guard<std::recursive_mutex> lock(m_brake_mutex);
    auto m=mts_motion_snapshot.get();
    if(advance) { m.boot_ms+=100; m.received_s=clockNow(); m.position.latitude_deg+=0.00001; }
    m.velocity.north_m_s=speed; mts_motion_snapshot.set(m);
  }
  void tick(double now=clockNow()) {
    std::lock_guard<std::recursive_mutex> lock(m_brake_mutex);
    serviceBrakeAndHold(now);
  }
};
"""
MAIN = r"""
int main(int argc,char** argv) {
  assert(argc==2); const std::string test=argv[1]; UAV_Model m; m.seed();
  using Phase=UAV_Model::BrakePhase;
  if(test=="ordering") {
    const auto entry=m.mts_motion_snapshot.get().position;
    assert(m.requestBrakeAndHold()); auto id=m.m_brake_status.command_id;
    assert(!m.requestBrakeAndHold()); // duplicate never resets intent/deadline
    assert(!m.commandCopterHelmSetpoint(30,3,8)); // stale stream blocked immediately
    m.tick(); assert(m.packets.back()=='Z'); assert(m.m_brake_status.stopped_samples==0);
    m.sample(0.1); m.tick(); assert(m.m_brake_status.stopped_samples==1);
    for(int i=0;i<8;i++) m.tick(); // one retained sample cannot establish stopping
    assert(m.m_brake_status.stopped_samples==1);
    m.sample(0.3); m.tick(); assert(m.m_brake_status.stopped_samples==0);
    for(int i=0;i<3;i++) { m.sample(0.1); m.tick(); }
    assert(m.m_brake_status.phase==Phase::HoldTargetSent);
    assert(m.hold.latitude_deg>entry.latitude_deg); // after braking, not request anchor
    assert(m.hold.longitude_deg==entry.longitude_deg && m.hold.absolute_altitude_m==9);
    assert(m.packets.back()=='P'); auto count=m.packets.size();
    m.tick(); assert(!m.commandCopterHelmSetpoint(30,3,8)); assert(m.packets.size()==count);
    assert(m.outcomes.back().id==id && m.outcomes.back().status=="COMPLETED");
  } else if(test=="admission") {
    for(bool* flag:{&m.copter,&m.connected,&m.guided,&m.armed,&m.received}) {
      *flag=false; assert(!m.requestBrakeAndHold()); *flag=true;
    }
    for(auto state:{mavsdk::Telemetry::LandedState::Unknown,mavsdk::Telemetry::LandedState::OnGround,
                    mavsdk::Telemetry::LandedState::TakingOff,mavsdk::Telemetry::LandedState::Landing}) {
      m.landed=state; assert(!m.requestBrakeAndHold());
    }
    m.landed=mavsdk::Telemetry::LandedState::InAir;
    for(double age:{-1.,2.1,double(NAN),double(INFINITY)}) {
      m.landed_age=age; assert(!m.requestBrakeAndHold());
    }
    m.landed_age=.1;
    for(double stamp:{-1.,clockNow()-3,clockNow()+2,double(NAN),double(INFINITY)}) {
      m.mts_armed_snapshot.set({true,stamp}); assert(!m.requestBrakeAndHold());
    }
    m.seed(); m.mts_armed_snapshot.set({false,clockNow()}); assert(!m.requestBrakeAndHold());
    m.seed(); m.m_command_queue.push(1); assert(!m.requestBrakeAndHold()); m.m_command_queue.pop();
    m.m_command_executing=true; assert(!m.requestBrakeAndHold());
    assert(m.packets.empty());
  } else if(test=="motion") {
    for(double stamp:{-1.,clockNow()-1,clockNow()+1,double(NAN),double(INFINITY)}) {
      m.seed(); auto s=m.mts_motion_snapshot.get(); s.received_s=stamp; m.mts_motion_snapshot.set(s);
      assert(!m.requestBrakeAndHold());
    }
    for(double bad:{double(NAN),double(INFINITY)}) {
      for(int field=0;field<7;field++) {
        m.seed(); auto s=m.mts_motion_snapshot.get();
        if(field==0)s.position.latitude_deg=bad;
        if(field==1)s.position.longitude_deg=bad;
        if(field==2)s.position.absolute_altitude_m=bad;
        if(field==3)s.position.relative_altitude_m=bad;
        if(field==4)s.velocity.north_m_s=bad;
        if(field==5)s.velocity.east_m_s=bad;
        if(field==6)s.velocity.down_m_s=bad;
        m.mts_motion_snapshot.set(s); assert(!m.requestBrakeAndHold());
      }
    }
    m.seed(); assert(m.requestBrakeAndHold()); auto s=m.mts_motion_snapshot.get();
    s.received_s=clockNow()-1; m.mts_motion_snapshot.set(s); m.tick();
    assert(m.m_brake_status.phase==Phase::Failed && m.packets.empty());
    assert(!m.commandCopterHelmSetpoint(0,3,8));
  } else if(test=="active_guards") {
    for(int field=0;field<8;field++) {
      UAV_Model active; active.seed(); assert(active.requestBrakeAndHold());
      if(field==0)active.connected=false;
      if(field==1)active.guided=false; // never reclaim a native non-Guided mode
      if(field==2)active.armed=false;
      if(field==3)active.received=false;
      if(field==4)active.landed=mavsdk::Telemetry::LandedState::OnGround;
      if(field==5)active.landed_age=3;
      if(field==6)active.mts_armed_snapshot.set({true,clockNow()-3});
      if(field==7)active.mts_armed_snapshot.set({false,clockNow()});
      active.tick();
      assert(active.m_brake_status.phase==Phase::Failed && active.packets.empty());
      assert(!active.commandCopterHelmSetpoint(0,3,8));
    }
  } else if(test=="timeout") {
    assert(m.requestBrakeAndHold());
    double now=m.m_brake_started_s+11; auto s=m.mts_motion_snapshot.get();
    s.received_s=now; m.mts_motion_snapshot.set(s); m.mts_armed_snapshot.set({true,now});
    m.tick(now); assert(m.m_brake_status.phase==Phase::Failed);
    assert(m.m_brake_status.reason=="BRAKING_TIMEOUT" && m.packets.empty());
    assert(!m.commandCopterHelmSetpoint(0,3,8));
  } else if(test=="failures") {
    assert(m.requestBrakeAndHold()); m.velocity_ok=false; m.tick();
    assert(m.m_brake_status.phase==Phase::Failed);
    m.cancelBrakeAndHold("new_guidance"); m.velocity_ok=true; assert(m.requestBrakeAndHold());
    m.position_ok=false;
    for(int i=0;i<3;i++) { m.sample(0.1); m.tick(); }
    assert(m.m_brake_status.phase==Phase::Failed);
    assert(m.m_brake_status.reason=="HOLD_TARGET_QUEUE_FAILED");
    assert(m.mts_current_loiter_coord.get().lat==0); // failed send never installs anchor
  } else if(test=="preemption") {
    assert(m.requestBrakeAndHold()); m.sample(.1); m.tick();
    auto n=m.packets.size(); m.cancelBrakeAndHold("LAND_REQUEST");
    for(int i=0;i<4;i++) { m.sample(.0); m.tick(); }
    assert(m.packets.size()==n && m.m_brake_status.phase==Phase::Canceled);
    for(auto mode:{mavsdk::Telemetry::FlightMode::Land,mavsdk::Telemetry::FlightMode::ReturnToLaunch}) {
      m.mode=mavsdk::Telemetry::FlightMode::Guided; assert(m.requestBrakeAndHold());
      n=m.packets.size(); m.mode=mode; m.tick();
      assert(m.m_brake_status.phase==Phase::Canceled && m.packets.size()==n);
    }
    // FC safety modes also revoke an already-installed hold transaction.
    m.mode=mavsdk::Telemetry::FlightMode::Guided; assert(m.requestBrakeAndHold());
    for(int i=0;i<3;i++) { m.sample(.1); m.tick(); }
    assert(m.m_brake_status.phase==Phase::HoldTargetSent);
    m.mode=mavsdk::Telemetry::FlightMode::Land; m.tick();
    assert(m.m_brake_status.phase==Phase::Canceled);
  } else if(test=="native_packets") {
    m.subscribeMotion(); mavlink_message_t packet;
    packet.raw={1000,420000000,-710000000,9000,8000,260,0,0};
    m.m_mavPass_ptr->receive(packet); auto received=m.mts_motion_snapshot.get();
    assert(received.position.latitude_deg==42 && received.velocity.north_m_s>2.59);
    assert(m.requestBrakeAndHold());
    for(int i=0;i<4;i++) { packet.raw.vx=0; m.m_mavPass_ptr->receive(packet); m.tick(); }
    assert(m.m_brake_status.stopped_samples==0); // duplicate boot time never counts/renews
    assert(m.mts_motion_snapshot.get().received_s==received.received_s);
    packet.sysid=18; packet.raw.time_boot_ms=1100; m.m_mavPass_ptr->receive(packet);
    assert(m.mts_motion_snapshot.get().boot_ms==1000);
    packet.sysid=17; packet.compid=191; m.m_mavPass_ptr->receive(packet);
    assert(m.mts_motion_snapshot.get().boot_ms==1000);
    packet.compid=1; packet.raw.time_boot_ms=10; m.m_mavPass_ptr->receive(packet);
    assert(m.m_brake_status.phase==Phase::Failed); m.tick();
    assert(m.m_brake_status.reason=="NATIVE_CLOCK_RESET_OR_REORDER");
    m.cancelBrakeAndHold("new_guidance");
    assert(!m.requestBrakeAndHold()); // pre-reboot armed/air cache cannot authorize
    m.mts_armed_snapshot.set({true,clockNow()});
    m.m_last_landed_state_update_s=clockNow();
    assert(m.requestBrakeAndHold());
  } else if(test=="concurrent") {
    assert(m.requestBrakeAndHold());
    std::thread late([&] { for(int i=0;i<100;i++) assert(!m.commandCopterHelmSetpoint(20,3,8)); });
    for(int i=0;i<3;i++) { m.sample(.1); m.tick(); }
    late.join();
    for(char p:m.packets) assert(p!='V');
    assert(m.m_brake_status.phase==Phase::HoldTargetSent);
  } else assert(false);
}
"""


class BrakeAndHoldTest(unittest.TestCase):
    def test_production_native_brake_arbiter(self):
        source = (BASE / "UAV_Model.cpp").read_text()
        header = (BASE / "UAV_Model.h").read_text()
        declarations = header[header.index("  static constexpr double BRAKE_MOTION_MAX_AGE_S"):
                              header.index("  bool commandAndSetAltitudeAGL")]
        start = source.index("std::string UAV_Model::brakePhaseName(")
        end = source.index("bool UAV_Model::sendCopterVelocitySetpoint(", start)
        methods = source[start:end]
        start = source.index("  m_mavPass_ptr->subscribe_message(\n      MAVLINK_MSG_ID_GLOBAL_POSITION_INT,")
        end = source.index("  m_mavPass_ptr->subscribe_message(\n      MAVLINK_MSG_ID_LANDING_TARGET,", start)
        receiver = "void UAV_Model::subscribeMotion() {\n" + source[start:end] + "}\n"
        with tempfile.TemporaryDirectory(prefix="brake-hold-unit-") as directory:
            cpp = Path(directory) / "test.cpp"
            cpp.write_text(PRELUDE + declarations + STUB + methods + receiver + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                            "-Werror", "-pthread", str(cpp), "-o", str(binary)],
                           check=True, timeout=30)
            for case in ("ordering", "admission", "motion", "active_guards", "timeout", "failures",
                         "preemption", "native_packets", "concurrent"):
                with self.subTest(case=case):
                    subprocess.run([str(binary), case], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
