"""Compile the actual publication method with isolated model/Notify test doubles.

Uses real MOOS-IvP angle wrapping, a controlled grid-convergence offset, and no
MOOS database, autopilot, simulator, flight commands, or live-process changes.
The method is extracted unchanged so tests cannot drift into a copied algorithm.
"""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]
IVP = Path(os.environ.get("MOOS_IVP_ROOT", BASE.parent / "moos-ivp"))
MISSION = BASE.parent / "moos-ivp-pearl/missions/briggs_field/2026_10_09_water_ops_prep"

PRELUDE = r"""
#include "AngleUtils.h"
#include <cassert>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>

struct Model {
  double yaw = 240, cog = 90, speed = 0;
  double getLatitude() const { return 42.356424; }
  double getLongitude() const { return -71.098393; }
  bool isHoldCourseGuidedSet() const { return false; }
  double getSOG() const { return speed; }
  double getHeading() const { return angle360(yaw); }
  struct Attitude { struct Euler { double roll_deg, pitch_deg, yaw_deg; unsigned long timestamp_us; } euler; double received_s, received_moos_time; };
  Attitude getAttitudeTelemetry() const { return {{12, -7, yaw, 123456}, 9.9, 9.9}; }
  double getAttitudeTelemetryAge(const Attitude&) const { return 0.1; }
  std::string getAttitudeSamples() const { return "123456,12,-7,240,9.9"; }
  double getCOG() const { return angle360(cog); }
  double getAltitudeAGL() const { return 8; }
  double getAltitudeMSL() const { return 12; }
};
struct WarningStub {
  void queue_monitorWarningForXseconds(const std::string&, double) {}
};
class ArduBridge {
public:
  Model m_uav_model;
  double m_curr_time = 10, grid_north = -1.5;
  std::shared_ptr<WarningStub> m_warning_system_ptr = std::make_shared<WarningStub>();
  std::map<std::string, double> values;
  void postTelemetryUpdate(const std::string& prefix);
  bool Notify(const std::string& key, double value, double) {
    values[key] = value;
    return true;
  }
  bool Notify(const std::string&, const std::string&, double) { return true; }
  bool isHelmCommanding() const { return false; }
  void visualizeCourseHoldTarget(bool) {}
  XYPoint transformLatLonToXY(const XYPoint&) const { return XYPoint(51, -56); }
  double gridBearingOfTrueNorth(double, double) const { return grid_north; }
};
"""

MAIN = r"""
void check(double actual, double expected) {
  assert(std::fabs(actual - expected) < 1e-8);
}
int main(int argc, char** argv) {
  assert(argc == 2);
  ArduBridge bridge; // one instance: production method's static Notify captures this
  const std::string test = argv[1];
  if (test == "stationary") {
    for (double cog : {0.0, 90.0, 180.0, 270.0, 359.0}) {
      bridge.m_uav_model.cog = cog;
      bridge.postTelemetryUpdate("UAV");
      check(bridge.values.at("UAV_HEADING"), 238.5);
      check(bridge.values.at("UAV_YAW"), 240);
      check(bridge.values.at("UAV_COG"), angle360(cog - 1.5));
      check(bridge.values.at("UAV_SPEED"), 0);
    }
    bridge.m_uav_model.yaw = 70; // actual yaw rotation, independent of COG
    bridge.postTelemetryUpdate("UAV");
    check(bridge.values.at("UAV_HEADING"), 68.5);
    check(bridge.values.at("UAV_YAW"), 70);
  } else if (test == "wrapping") {
    bridge.grid_north = 1.3;
    bridge.m_uav_model.yaw = 359.9;
    bridge.m_uav_model.cog = 120;
    bridge.postTelemetryUpdate("UAV");
    check(bridge.values.at("UAV_HEADING"), 1.2);
    check(bridge.values.at("UAV_COG"), 121.3);
    check(bridge.values.at("UAV_YAW"), 359.9);
    bridge.grid_north = 359.1; // relAng's wrapped representation of -0.9 degrees
    bridge.m_uav_model.yaw = 0.2;
    bridge.m_uav_model.cog = 0;
    bridge.postTelemetryUpdate("UAV");
    check(bridge.values.at("UAV_HEADING"), 359.3);
    check(bridge.values.at("UAV_COG"), 359.1);
    bridge.grid_north = 0; // local-grid mode has no convergence correction
    bridge.postTelemetryUpdate("UAV");
    check(bridge.values.at("UAV_HEADING"), 0.2);
    check(bridge.values.at("UAV_COG"), 0);
  } else if (test == "prefix_and_nan") {
    bridge.m_uav_model.yaw = std::numeric_limits<double>::quiet_NaN();
    bridge.postTelemetryUpdate("DRONE");
    assert(!bridge.values.count("DRONE_HEADING"));
    assert(!bridge.values.count("DRONE_YAW"));
    check(bridge.values.at("DRONE_COG"), 88.5);
    bridge.values.clear();
    bridge.m_uav_model.yaw = 70;
    bridge.m_uav_model.cog = std::numeric_limits<double>::quiet_NaN();
    bridge.postTelemetryUpdate("DRONE");
    check(bridge.values.at("DRONE_HEADING"), 68.5);
    assert(!bridge.values.count("DRONE_COG"));
    assert(!bridge.values.count("UAV_HEADING"));
    check(bridge.values.at("DRONE_X"), 51);
    check(bridge.values.at("DRONE_Y"), -56);
    check(bridge.values.at("DRONE_ALTITUDE"), 8);
    check(bridge.values.at("DRONE_ALTITUDE_MSL"), 12);
    check(bridge.values.at("DRONE_DEPTH"), -8);
  } else return 1;
}
"""


class HeadingPublicationsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="heading-publications-")
        cls.addClassCleanup(cls.directory.cleanup)
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index("void ArduBridge::postTelemetryUpdate(")
        stop = source.index("void ArduBridge::postHealthUpdate(", start)
        harness = Path(cls.directory.name) / "test.cpp"
        harness.write_text(PRELUDE + source[start:stop] + MAIN)
        cls.binary = harness.with_suffix("")
        subprocess.run(
            [
                os.environ.get("CXX", "c++"),
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I" + str(IVP / "ivp/src/lib_geometry"),
                "-I" + str(IVP / "ivp/src/lib_mbutil"),
                str(harness),
                str(IVP / "lib/libgeometry.a"),
                str(IVP / "lib/libmbutil.a"),
                "-o",
                str(cls.binary),
            ],
            check=True,
            timeout=30,
        )

    def check_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=5)

    def test_stationary_cog_does_not_rotate_body_heading(self):
        self.check_case("stationary")

    def test_grid_conversion_and_north_wrap(self):
        self.check_case("wrapping")

    def test_custom_prefix_and_independent_nan_inputs(self):
        self.check_case("prefix_and_nan")

    def test_prep_navigation_wiring_keeps_true_yaw_for_target(self):
        config = (MISSION / "meta_uav.moos").read_text()
        self.assertIn("Echo = UAV_HEADING  -> NAV_HEADING", config)
        self.assertIn("Echo = UAV_COG      -> NAV_COG", config)
        self.assertIn("Echo = UAV_YAW          -> UAV_NAV_HEADING", config)
        self.assertIn("report_cog        = true", config)


if __name__ == "__main__":
    unittest.main()
