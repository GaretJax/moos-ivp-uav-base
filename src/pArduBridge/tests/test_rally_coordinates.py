"""Regression tests for MOOS rally X/Y to latitude/longitude conversion."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]

PRELUDE = r"""
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <limits>

struct Geodesy {
  enum class Method { None, UTM, LocalGrid };
  Method method = Method::None;
  double first = 0, second = 0;

  bool UTM2LatLong(double x, double y, double &latitude, double &longitude) {
    method = Method::UTM;
    first = x;
    second = y;
    latitude = y + 100;
    longitude = x - 100;
    return true;
  }

  bool LocalGrid2LatLong(double x, double y, double &latitude,
                         double &longitude) {
    method = Method::LocalGrid;
    first = x;
    second = y;
    latitude = y + 200;
    longitude = x - 200;
    return true;
  }
};

class ArduBridge {
public:
  bool m_geo_ok = true;
  bool m_geo_utm = true;
  Geodesy m_geodesy;
  bool transformXYToLatLon(double x, double y, double &latitude,
                           double &longitude);
};
"""

MAIN = r"""
int main() {
  ArduBridge bridge;
  double latitude = 0, longitude = 0;

  assert(bridge.transformXYToLatLon(15.042299, -51.993939,
                                    latitude, longitude));
  assert(bridge.m_geodesy.method == Geodesy::Method::UTM);
  assert(std::fabs(bridge.m_geodesy.first - 15.042299) < 1e-9);
  assert(std::fabs(bridge.m_geodesy.second + 51.993939) < 1e-9);
  assert(std::fabs(latitude - 48.006061) < 1e-9);
  assert(std::fabs(longitude + 84.957701) < 1e-9);

  bridge.m_geo_utm = false;
  assert(bridge.transformXYToLatLon(43.787856, -39.259280,
                                    latitude, longitude));
  assert(bridge.m_geodesy.method == Geodesy::Method::LocalGrid);
  assert(std::fabs(bridge.m_geodesy.first - 43.787856) < 1e-9);
  assert(std::fabs(bridge.m_geodesy.second + 39.259280) < 1e-9);

  bridge.m_geo_ok = false;
  bridge.m_geodesy.method = Geodesy::Method::None;
  assert(!bridge.transformXYToLatLon(1, 2, latitude, longitude));
  assert(bridge.m_geodesy.method == Geodesy::Method::None);

  bridge.m_geo_ok = true;
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
    assert(!bridge.transformXYToLatLon(invalid, 2, latitude, longitude));
    assert(!bridge.transformXYToLatLon(1, invalid, latitude, longitude));
  }
}
"""


class RallyCoordinatesTest(unittest.TestCase):
    def test_production_conversion_preserves_east_x_north_y_order(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index("bool ArduBridge::transformXYToLatLon(")
        stop = source.index("XYPoint ArduBridge::transformLatLonToXY(", start)

        with tempfile.TemporaryDirectory(prefix="rally-coordinates-") as temp:
            cpp = Path(temp) / "test.cpp"
            cpp.write_text(PRELUDE + source[start:stop] + MAIN)
            binary = cpp.with_suffix("")
            subprocess.run(
                [
                    os.environ.get("CXX", "c++"),
                    "-std=c++17",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    str(cpp),
                    "-o",
                    str(binary),
                ],
                check=True,
                timeout=30,
            )
            subprocess.run([str(binary)], check=True, timeout=5)

    def test_rally_mail_uses_named_x_y_order(self):
        source = (BASE / "src/pArduBridge/ArduBridge.cpp").read_text()
        start = source.index('else if (key == "RALLY_UPLOAD")')
        stop = source.index('else if (key == "DESIRED_HEADING")', start)
        rally_handler = source[start:stop]
        self.assertIn(
            "transformXYToLatLon(x, y, latitude, longitude)", rally_handler
        )
        self.assertNotIn("transformXYToLatLon(y, x", rally_handler)


if __name__ == "__main__":
    unittest.main()
