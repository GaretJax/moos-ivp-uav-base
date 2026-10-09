#!/usr/bin/env python3
"""Regression checks for clean pArduBridge startup failures (no autopilot needed)."""

import os
import socket
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BRIDGE = Path(os.environ.get("ARDUBRIDGE_BIN", ROOT / "bin/pArduBridge"))
MOOSDB = Path(os.environ.get("MOOSDB_BIN", ROOT.parent / "moos-ivp/bin/MOOSDB"))


class StartupFailureTest(unittest.TestCase):
    def run_startup(self, config: str, expected_error: str) -> None:
        with tempfile.TemporaryDirectory(prefix="ardubridge-startup-") as directory:
            mission = Path(directory) / "targ_startup_probe.moos"
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
            mission.write_text(
                f"ServerHost = localhost\nServerPort = {port}\n"
                "Community = startup_probe\n"
                "LatOrigin = 42.358436\nLongOrigin = -71.087448\n"
                "ProcessConfig = pArduBridge\n{\n"
                "AppTick = 10\nCommsTick = 10\n"
                "vname = startup_probe\nvehicle_type = copter\n"
                f"is_sim = true\nlogger = false\n{config}\n}}\n"
            )
            with (Path(directory) / "moosdb.log").open("w") as log:
                database = subprocess.Popen(
                    [str(MOOSDB), str(mission)], stdout=log, stderr=subprocess.STDOUT
                )
                try:
                    deadline = time.monotonic() + 5
                    while True:
                        self.assertIsNone(database.poll(), "MOOSDB failed to start")
                        try:
                            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                                break
                        except OSError:
                            if time.monotonic() >= deadline:
                                self.fail("MOOSDB did not start listening")
                            time.sleep(0.05)
                    result = subprocess.run(
                        [str(BRIDGE), str(mission), "pArduBridge"],
                        capture_output=True,
                        text=True,
                        timeout=15,
                        check=False,
                    )
                    output = result.stdout + result.stderr
                    self.assertEqual(result.returncode, 1, output)
                    self.assertIn(expected_error, output)
                    self.assertNotIn("terminate called", output)
                finally:
                    database.terminate()
                    try:
                        database.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        database.kill()
                        database.wait()

    def test_unavailable_endpoint(self) -> None:
        # A bound but non-listening socket reserves an unreachable TCP port.
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            port = endpoint.getsockname()[1]
            self.run_startup(
                f"url_protocol = tcp\nurl = 127.0.0.1:{port}",
                "Failed to connect to ArduPilot at",
            )

    def test_no_autopilot_heartbeat(self) -> None:
        # TCP opens successfully, but no MAVLink autopilot is discoverable.
        with socket.socket() as endpoint:
            endpoint.bind(("127.0.0.1", 0))
            endpoint.listen()
            port = endpoint.getsockname()[1]
            self.run_startup(
                f"url_protocol = tcp\nurl = 127.0.0.1:{port}",
                "UAV System NOT discovered",
            )

    def test_invalid_url(self) -> None:
        self.run_startup("url = invalid", "URL protocol not set")


if __name__ == "__main__":
    unittest.main()
