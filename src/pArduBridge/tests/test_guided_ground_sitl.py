"""Opt-in owned SITL: two grounded LAND -> Guided admissions, never ARM/TAKEOFF.

Unique ports/project; never connects to or restarts an existing mission.
Native heartbeats, not MOOS flags alone, verify each mode and disarm state.
"""

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import tempfile
import time
from pathlib import Path

BASE = Path(__file__).resolve().parents[3]
WRAPPER = BASE.parent / "ardupilot-sitl/bin/ardupilot-sitl"
MOOS = BASE.parent / "moos-ivp/bin"


def crc(data):
    value = 0xFFFF
    for byte in data:
        tmp = byte ^ (value & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        value = ((value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return value


class NativeModes:
    def __init__(self, port):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.socket.settimeout(1)
        self.buffer = bytearray()
        self.sequence = 0

    def set_ground_land(self):
        # SET_MODE only, to this test's owned FC. No heartbeat, RC, ARM or takeoff.
        payload = struct.pack("<IBB", 9, 1, 1)
        header = bytes([len(payload), self.sequence, 255, 190, 11])
        self.sequence = (self.sequence + 1) % 256
        self.socket.sendall(b"\xfe" + header + payload +
                            struct.pack("<H", crc(header + payload + bytes([89]))))

    def confirm(self, mode, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while self.buffer:
                if self.buffer[0] not in (0xFE, 0xFD):
                    del self.buffer[0]
                    continue
                v2 = self.buffer[0] == 0xFD
                header = 10 if v2 else 6
                if len(self.buffer) < header:
                    break
                length = self.buffer[1]
                total = header + length + 2 + (13 if v2 and self.buffer[2] & 1 else 0)
                if len(self.buffer) < total:
                    break
                frame = bytes(self.buffer[:total])
                del self.buffer[:total]
                sysid, compid = (frame[5], frame[6]) if v2 else (frame[3], frame[4])
                msgid = int.from_bytes(frame[7:10], "little") if v2 else frame[5]
                if (sysid, compid, msgid) != (1, 1, 0) or length < 9:
                    continue
                if crc(frame[1:header + length] + bytes([50])) != struct.unpack_from(
                        "<H", frame, header + length)[0]:
                    continue
                payload = frame[header:header + length]
                native_mode = struct.unpack_from("<I", payload)[0]
                assert not payload[6] & 128, "Owned FC unexpectedly armed"
                if native_mode == mode:
                    return
            try:
                data = self.socket.recv(8192)
            except TimeoutError:
                continue
            if not data:
                raise AssertionError("Owned diagnostic connection closed")
            self.buffer.extend(data)
        raise AssertionError(f"Native mode {mode} was not confirmed")


def run_test():
    directory = Path(tempfile.mkdtemp(prefix="guided-ground-sitl-"))
    reservations = []
    for _ in range(4):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        reservations.append(sock)
    database, endpoint, diagnostic, simulation = [s.getsockname()[1] for s in reservations]
    for sock in reservations:
        sock.close()
    overlay = directory / "initial-land.parm"
    overlay.write_text("# Owned regression: start disarmed in retained recovery mode.\nINITIAL_MODE 9\n")
    env = dict(os.environ, COMPOSE_PROJECT_NAME=f"guided-ground-check-{os.getpid()}")
    options = [str(WRAPPER), "--no-rc", "--home", "42.3569186052,-71.0990291116,0,0",
               "--port", str(endpoint), "--diagnostic-port", str(diagnostic),
               "--sim-port", str(simulation), "--param-file", str(overlay),
               str(directory / "sitl")]
    mission = directory / "test.moos"
    mission.write_text(f"""ServerHost=127.0.0.1
ServerPort={database}
Community=guided_probe
LatOrigin=42.3569186052
LongOrigin=-71.0990291116
ProcessConfig=ANTLER
{{
 MSBetweenLaunches=100
 Run={MOOS / 'MOOSDB'} @ NewConsole=false
 Run={BASE / 'bin/pArduBridge'} @ NewConsole=false
 Run={MOOS / 'pLogger'} @ NewConsole=false
 Run={MOOS / 'uTimerScript'} @ NewConsole=false ~ guidedRequest
}}
ProcessConfig=pArduBridge
{{
 AppTick=10
 CommsTick=10
 url=127.0.0.1:{endpoint}
 url_protocol=tcp
 vehicle_type=copter
 geodesy=utm
 prefix=UAV
 vname=guided_probe
 require_verified_rally=true
 is_sim=true
 logger=false
}}
ProcessConfig=guidedRequest
{{
 AppTick=10
 CommsTick=10
 condition=TEST_ENTER_GUIDED=true
 upon_awake=restart
 script_atomic=false
 reset_max=unlimited
 paused=false
 event=var=ARDU_COMMAND,val=ENTER_GUIDED,time=0
 event=var=TEST_ENTER_GUIDED,val=false,time=0.1
}}
ProcessConfig=pLogger
{{
 AppTick=10
 CommsTick=10
 File=LOG_PROBE
 PATH=./
 AsyncLog=true
 FileTimeStamp=true
 WildCardLogging=true
}}
""")
    process = None
    native = None
    started = False
    stream = (directory / "antler.log").open("w")
    print(f"Artifacts: {directory}", flush=True)

    def query(*conditions, wait=15):
        result = subprocess.run([str(MOOS / "uQueryDB"), str(mission), f"--wait={wait}",
                                 *(f"--condition={c}" for c in conditions),
                                 "--check_var=UAV_COMMAND_RESULT", "--esv"],
                                cwd=directory, capture_output=True, text=True,
                                check=False, timeout=wait + 10)
        assert result.returncode == 0, result.stdout + result.stderr + (
            (directory / ".checkvars").read_text() if (directory / ".checkvars").exists() else "")

    try:
        started = True
        subprocess.run([*options, "up", "--no-build", "-d"], env=env, check=True,
                       capture_output=True, timeout=45)
        process = subprocess.Popen([str(MOOS / "pAntler"), str(mission)], cwd=directory,
                                   stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        query("UAV_LANDED_STATE=ON_GROUND", "UAV_IS_ARMED=0", "UAV_GPS_AVAILABLE=1")
        native = NativeModes(diagnostic)
        native.confirm(9)
        for _ in range(2):
            query("UAV_GUIDED_MODE_ACTIVE=0", "UAV_LANDED_STATE=ON_GROUND", "UAV_IS_ARMED=0")
            subprocess.run([str(MOOS / "uPokeDB"), str(mission), "--quiet",
                            "TEST_ENTER_GUIDED:=true"], cwd=directory, check=True,
                           capture_output=True, timeout=12)
            query("UAV_GUIDED_MODE_ACTIVE=1", "UAV_IS_ARMED=0", wait=5)
            native.confirm(4)
            # Restore this owned disarmed FC's recovery mode for a second request.
            native.set_ground_land()
            native.confirm(9)
        query("UAV_GUIDED_MODE_ACTIVE=0", "UAV_IS_ARMED=0")
        result = {"passed": True, "grounded_recovery_admissions": 2,
                  "native_modes": [9, 4, 9, 4, 9], "armed": False,
                  "flight_test": False, "arm_or_takeoff_requested": False}
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print("PASS: two native grounded LAND -> Guided requests; disarmed throughout", flush=True)
    finally:
        if native is not None:
            native.socket.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=4)
        stream.close()
        if started:
            subprocess.run([*options, "down"], env=env, capture_output=True,
                           check=False, timeout=45)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-sitl", action="store_true", required=True)
    parser.parse_args()
    run_test()
