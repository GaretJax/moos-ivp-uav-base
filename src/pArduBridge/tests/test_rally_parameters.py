"""Production rally integer verification against deterministic native-API doubles."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]
PRELUDE = r"""
#include <cassert>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace mavsdk {
struct Param {
  enum class Result { Success, Timeout, WrongType, ConnectionError };
  std::vector<std::pair<Result,int>> reads{{Result::Success,0}};
  Result write = Result::Success;
  int get_count = 0, set_count = 0;
  std::pair<Result,int> get_param_int(const std::string&) {
    return reads.at(get_count++);
  }
  Result set_param_int(const std::string&, int value) {
    assert(value == 0); ++set_count; return write;
  }
};
struct Telemetry { enum class LandedState { OnGround, InAir }; };
}
std::ostream& operator<<(std::ostream& out, mavsdk::Param::Result r) {
  return out << static_cast<int>(r);
}
struct System { bool connected = true; bool is_connected() { return connected; } };
constexpr double LANDED_STATE_TELEMETRY_MAX_AGE_S = 2;
class UAV_Model {
public:
  std::unique_ptr<System> m_system_ptr = std::make_unique<System>();
  std::unique_ptr<mavsdk::Param> m_param_ptr = std::make_unique<mavsdk::Param>();
  bool armed = false, landed_received = true;
  double age = 0.1;
  int ground_reads = 0, change_after = -1;
  std::vector<std::string> events;
  bool isArmed() {
    return armed || (change_after >= 0 && ground_reads++ >= change_after);
  }
  bool hasLandedStateTelemetry() { return landed_received; }
  double getLandedStateTelemetryAge() { return age; }
  mavsdk::Telemetry::LandedState getLandedState() {
    return mavsdk::Telemetry::LandedState::OnGround;
  }
  void reportEventFromCallback(const std::string& e) { events.push_back(e); }
  bool verifyRallyIntParameter(const std::string&, int, std::string&);
};
"""
MAIN = r"""
int main() {
  using R = mavsdk::Param::Result;
  std::string reason;
  { UAV_Model u; // Already configured: fresh GET, no gratuitous SET.
    assert(u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
    assert(u.m_param_ptr->get_count == 1 && u.m_param_ptr->set_count == 0);
  }
  { UAV_Model u; // False write-echo timeout is not proof; new GET is.
    u.m_param_ptr->reads = {{R::Success,1},{R::Success,0}};
    u.m_param_ptr->write = R::Timeout;
    assert(u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
    assert(u.m_param_ptr->get_count == 2 && u.events.size() == 1);
  }
  for (R write : {R::Success,R::Timeout}) {
    UAV_Model u; u.m_param_ptr->write = write;
    u.m_param_ptr->reads = {{R::Success,1},{R::Success,1}};
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
    assert(reason.find("readback_failed") != std::string::npos);
  }
  { UAV_Model u;
    u.m_param_ptr->reads = {{R::Success,1},{R::WrongType,0}};
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
  }
  { UAV_Model u;
    u.m_param_ptr->reads = {{R::Timeout,0}};
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
    assert(u.m_param_ptr->set_count == 0);
  }
  { UAV_Model u;
    u.m_param_ptr->write = R::ConnectionError;
    u.m_param_ptr->reads = {{R::Success,1},{R::Success,0}};
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
  }
  for (int change : {1,2}) {
    UAV_Model u; u.change_after = change;
    u.m_param_ptr->reads = {{R::Success,1},{R::Success,0}};
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
  }
  for (int block=0; block<4; ++block) {
    UAV_Model u;
    if (block==0) u.armed=true;
    if (block==1) u.landed_received=false;
    if (block==2) u.age=3;
    if (block==3) u.m_system_ptr->connected=false;
    assert(!u.verifyRallyIntParameter("RALLY_INCL_HOME",0,reason));
    assert(u.m_param_ptr->get_count == 0 && u.m_param_ptr->set_count == 0);
  }
}
"""


class RallyParametersTest(unittest.TestCase):
    def test_native_readback_and_ground_guards(self):
        source = (BASE / "src/pArduBridge/UAV_Rally.cpp").read_text()
        start = source.index("bool UAV_Model::verifyRallyIntParameter(")
        stop = source.index("bool UAV_Model::readRallyPoint(", start)
        with tempfile.TemporaryDirectory(prefix="rally-parameters-") as temp:
            cpp = Path(temp) / "test.cpp"
            cpp.write_text(PRELUDE + source[start:stop] + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                            "-Werror", str(cpp), "-o", str(binary)], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
