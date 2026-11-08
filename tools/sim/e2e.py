#!/usr/bin/env python3
"""
End-to-end checks of the published driver binary against a software ZED-F9P and a fake NTRIP caster.

    python3 tools/sim/e2e.py path/to/Zedf9p

Run on Linux. To test the real Raspberry Pi build, run it in an emulated ARM64 container:

    docker run --rm --platform linux/arm64 -v "$PWD:/w" -w /w python:3.13-slim-trixie \
        python3 tools/sim/e2e.py out/linux-arm64/Zedf9p
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from f9p_sim import F9pSim, rtcm_frame  # noqa: E402

DRIVER = None
FAILURES = []


class Caster:
    """Answers NTRIP handshakes the way a scenario asks and records what it received."""

    def __init__(self, reply=b"ICY 200 OK\r\n", stream=b""):
        self.reply = reply
        self.stream = stream
        self.requests = []
        self.received = bytearray()
        self.connections = []
        self.closed_connections = []
        self.attempt_times = []
        self.server = socket.create_server(("127.0.0.1", 0))
        self.port = self.server.getsockname()[1]
        self.running = True
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while self.running:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        self.attempt_times.append(time.monotonic())
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = conn.recv(4096)
            if not chunk:
                return
            data += chunk
        head, _, rest = data.partition(b"\r\n\r\n")
        self.requests.append(head.decode("ascii", "replace") + "\r\n\r\n")
        conn.sendall(self.reply)
        if not self.reply.startswith(b"ICY"):
            conn.close()
            return
        self.connections.append(conn)
        self.received.extend(rest)
        if self.stream:
            threading.Thread(target=self._send_stream, args=(conn,), daemon=True).start()
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                self.received.extend(chunk)
        except OSError:
            pass
        self.closed_connections.append(conn)

    def _send_stream(self, conn):
        try:
            while self.running:
                conn.sendall(self.stream)
                time.sleep(0.5)
        except OSError:
            pass

    def drop_all(self):
        for conn in self.connections:
            try:
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()
            except OSError:
                pass
        self.connections.clear()

    def close(self):
        self.running = False
        self.server.close()
        self.drop_all()


class Driver:
    """The driver binary plus the sync socket its owner would listen on."""

    def __init__(self, sim, args, with_sync=True):
        self.tmp = tempfile.mkdtemp()
        self.updates = []
        self.conn = None
        self.server = None
        command = [DRIVER, "--com-port", sim.path, *args]
        if with_sync:
            path = os.path.join(self.tmp, "sync.sock")
            self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.server.bind(path)
            self.server.listen(1)
            command += ["--sync-socket", path]
            threading.Thread(target=self._accept, daemon=True).start()
        self.log = open(os.path.join(self.tmp, "driver.log"), "w+")
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, cwd=self.tmp)

    def _accept(self):
        self.conn, _ = self.server.accept()
        buffer = b""
        while True:
            try:
                chunk = self.conn.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.strip()
                if line:
                    self.updates.append(json.loads(line))

    def send(self, command):
        self.conn.sendall(command.encode("ascii") + b"\n")

    def latest(self, key):
        for update in reversed(self.updates):
            if key in update:
                return update[key]
        return None

    def output(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()

    def stop(self):
        # Called from a finally block: show why the scenario failed
        if sys.exc_info()[0] is not None:
            tail = "\n".join(self.output().splitlines()[-25:])
            print(f"--- driver output ---\n{tail}\n--- end ---", flush=True)
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if self.conn:
            self.conn.close()
        if self.server:
            self.server.close()


def wait_for(condition, timeout, what):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.1)
    raise AssertionError(f"timed out after {timeout}s waiting for {what}")


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def whole_frames(data):
    """True when data is an unbroken run of complete RTCM3 frames."""
    i = 0
    while i < len(data):
        if data[i] != 0xD3 or i + 3 > len(data):
            return False
        i += (((data[i + 1] & 3) << 8) | data[i + 2]) + 6
    return i == len(data)


# Scenarios ----------------------------------------------------------------------------------------

def station_survey_then_upload():
    caster, sim = Caster(), F9pSim(survey_seconds=2)
    driver = Driver(sim, ["--mode", "Station", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-password", "secret", "--rtcm-accuracy-req", "3", "--rtcm-survey-time", "2"])
    try:
        wait_for(lambda: driver.latest("IsSurveyValid") is False, 15, "survey updates")
        check(len(caster.requests) == 0, "must not connect to the caster before corrections flow")
        wait_for(lambda: len(caster.requests) == 1, 30, "caster connection after survey")
        request = caster.requests[0]
        check(request.startswith("SOURCE secret /TEST\r\n"), f"bad SOURCE request: {request!r}")
        wait_for(lambda: len(caster.received) >= 300, 10, "corrections at the caster")
        check(whole_frames(bytes(caster.received)), "caster must receive whole RTCM frames")
        check(sim.rate_ms == 1000, f"base should run at 1 Hz, got {sim.rate_ms} ms")
        wait_for(lambda: driver.latest("LastNtripSent") is not None, 5, "LastNtripSent status")
        wait_for(lambda: driver.latest("Latitude") is not None, 5, "position updates")
        check(abs(driver.latest("Latitude") - 40.1234567) < 1e-6, "latitude scale")
        check(driver.latest("Altitude") == 300.0, f"altitude in meters, got {driver.latest('Altitude')}")
    finally:
        driver.stop(), sim.close(), caster.close()


def station_restart_to_fixed():
    caster, sim = Caster(), F9pSim(survey_seconds=1)
    driver = Driver(sim, ["--mode", "Station", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-password", "secret", "--rtcm-survey-time", "1"])
    try:
        wait_for(lambda: len(caster.received) > 0, 40, "corrections before the restart")
        first_connection_count = len(caster.closed_connections)

        # A trailing zero and a negative longitude: the old parser turned these into the wrong place
        driver.send("RESTART_FIXED:40.1234560:-80.1234567:300.25")
        wait_for(lambda: sim.mode == 2, 30, "fixed mode on the module")
        lat, lon, height, lat_hp, lon_hp, height_hp, lla = sim.fixed_position
        check((lat, lon) == (401234560, -801234567), f"fixed lat/lon wrong: {lat}, {lon}")
        check(height == 30025, f"fixed height must be in cm, got {height}")
        check((lat_hp, lon_hp, height_hp) == (0, 0, 0), f"fixed remainders wrong: {lat_hp}, {lon_hp}, {height_hp}")
        check(lla, "fixed position must be flagged as lat/lon/height")

        # The old connection must be gone or the caster reports the mountpoint as taken
        wait_for(lambda: len(caster.closed_connections) == first_connection_count + 1, 10, "old caster connection closed")
        wait_for(lambda: len(caster.requests) == 2, 45, "new caster connection after the restart")
        before = len(caster.received)
        wait_for(lambda: len(caster.received) > before + 200, 15, "corrections flowing in fixed mode")

        # A bad position changes nothing and is reported
        driver.send("RESTART_FIXED:95.0:10.0:300")
        wait_for(lambda: "INVALID_FIXED_POSITION" in (driver.latest("Errors") or []), 5, "invalid position error")
        check(sim.mode == 2 and sim.fixed_position[0] == 401234560, "a bad position must not touch the module")
        check(driver.process.poll() is None, "a bad position must not stop the driver")
    finally:
        driver.stop(), sim.close(), caster.close()


def station_starts_fixed_from_args():
    caster, sim = Caster(), F9pSim(survey_seconds=999)
    driver = Driver(sim, ["--mode", "Station", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-password", "secret", "--fixed-position=-33.8688000:151.2093000:58.5"])
    try:
        wait_for(lambda: sim.mode == 2, 20, "fixed mode")
        check(sim.fixed_position[0] == -338688000, f"negative latitude, got {sim.fixed_position[0]}")
        check(sim.fixed_position[2] == 5850, f"height, got {sim.fixed_position[2]}")
        wait_for(lambda: len(caster.received) > 200, 20, "corrections without a survey")
        check(all(c[1] != 0x3B for c in sim.commands), "fixed mode must not poll the survey")
    finally:
        driver.stop(), sim.close(), caster.close()


def station_reconnects_after_drop():
    caster, sim = Caster(), F9pSim(survey_seconds=1)
    driver = Driver(sim, ["--mode", "Station", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-password", "secret", "--rtcm-survey-time", "1"])
    try:
        wait_for(lambda: len(caster.received) > 0, 30, "first connection")
        polls_before = sum(1 for c in sim.commands if c[1] == 0x3B)
        caster.drop_all()
        wait_for(lambda: "NTRIP_CONNECTION_ERROR" in (driver.latest("Errors") or []), 10, "error flag after the drop")
        wait_for(lambda: len(caster.requests) == 2, 30, "reconnect")
        before = len(caster.received)
        wait_for(lambda: len(caster.received) > before + 200, 10, "corrections after reconnect")
        check(sum(1 for c in sim.commands if c[1] == 0x3B) == polls_before, "a caster drop must not restart the survey")
        wait_for(lambda: driver.latest("Errors") == [], 5, "error cleared")
    finally:
        driver.stop(), sim.close(), caster.close()


def client_receives_corrections():
    stream = b"".join(rtcm_frame(t) for t in (1005, 1074, 1084))
    # The first stream bytes can arrive in the same read as the status line
    caster, sim = Caster(reply=b"ICY 200 OK\r\n" + rtcm_frame(1230), stream=stream), F9pSim()
    driver = Driver(sim, ["--mode", "Client", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-username", "rover@example.com"])
    try:
        wait_for(lambda: len(caster.requests) == 1, 20, "caster connection")
        request = caster.requests[0]
        check(request.startswith("GET /TEST HTTP/1.0\r\n"), f"bad request line: {request!r}")
        check("Authorization: Basic cm92ZXJAZXhhbXBsZS5jb206\r\n" in request, f"email must be the user name: {request!r}")
        check(request.endswith("\r\n\r\n"), "the request must end with a blank line")
        wait_for(lambda: len(sim.rtcm_received) >= 200, 10, "corrections at the module")
        check(whole_frames(bytes(sim.rtcm_received)), "the module must receive whole RTCM frames")
        check(bytes(sim.rtcm_received).startswith(rtcm_frame(1230)), "bytes after the status line must not be lost")
        wait_for(lambda: driver.latest("CarrierSolution") == 2, 10, "RTK fixed status")
        check(driver.latest("Satellites") == 24, "satellites")
        check(abs(driver.latest("Accuracy") - 0.014) < 1e-6, f"accuracy in meters, got {driver.latest('Accuracy')}")
        check(abs(driver.latest("Heading") - 90) < 1e-6, "heading")
        check(driver.latest("GroundSpeed") == 0.5, "ground speed")
        check(sim.rate_ms == 200, f"rover should run at 5 Hz, got {sim.rate_ms} ms")
        wait_for(lambda: driver.latest("LastRtcmReceived") is not None, 5, "LastRtcmReceived status")
    finally:
        driver.stop(), sim.close(), caster.close()


def client_keeps_position_and_backs_off():
    caster, sim = Caster(reply=b"SOURCETABLE 200 OK\r\n\r\nENDSOURCETABLE\r\n"), F9pSim()
    driver = Driver(sim, ["--mode", "Client", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-username", "rover@example.com"])
    try:
        wait_for(lambda: len(caster.attempt_times) == 2, 30, "second attempt")
        gap = caster.attempt_times[1] - caster.attempt_times[0]
        check(gap >= 14, f"retry too fast: {gap:.1f}s")
        check("NTRIP_MOUNTPOINT_DOWN" in (driver.latest("Errors") or []), "mountpoint error flag")
        time.sleep(3)
        check(len(caster.attempt_times) == 2, "must keep waiting between attempts")
        before = sum(1 for u in driver.updates if "Latitude" in u)
        time.sleep(1.5)
        check(sum(1 for u in driver.updates if "Latitude" in u) > before, "position must keep flowing without corrections")
    finally:
        driver.stop(), sim.close(), caster.close()


def client_stops_retrying_when_banned():
    caster, sim = Caster(reply=b"HTTP/1.1 403 Forbidden\r\n\r\nYour IP has been banned\r\n"), F9pSim()
    driver = Driver(sim, ["--mode", "Client", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-username", "rover@example.com"])
    try:
        wait_for(lambda: "NTRIP_BANNED" in (driver.latest("Errors") or []), 15, "banned flag")
        time.sleep(20)
        check(len(caster.attempt_times) == 1, f"must not retry while banned, saw {len(caster.attempt_times)}")
    finally:
        driver.stop(), sim.close(), caster.close()


def bad_arguments_exit():
    sim = F9pSim()
    try:
        # The arguments gps-rider used before this version
        old = subprocess.run([DRIVER, "-client", "-com-port", sim.path], capture_output=True, timeout=30)
        check(old.returncode == 2, f"old arguments must fail, got {old.returncode}")
        no_email = subprocess.run([DRIVER, "--com-port", sim.path, "--mode", "Client", "--ntrip-mountpoint", "X", "--ntrip-username", "nobody"],
                                  capture_output=True, timeout=30)
        check(no_email.returncode == 2, f"RTK2go without an email must fail, got {no_email.returncode}")
        check(b"email" in no_email.stderr, "the error must say why")
    finally:
        sim.close()


def exits_when_owner_goes_away():
    caster, sim = Caster(), F9pSim()
    driver = Driver(sim, ["--mode", "Client", "--ntrip-server", "127.0.0.1", "--ntrip-port", str(caster.port),
                          "--ntrip-mountpoint", "TEST", "--ntrip-username", "rover@example.com"])
    try:
        wait_for(lambda: driver.latest("Latitude") is not None, 20, "position updates")
        driver.conn.close()
        wait_for(lambda: driver.process.poll() is not None, 10, "driver exit")
        check(driver.process.returncode != 0, "losing the owner is an abnormal exit")
    finally:
        driver.stop(), sim.close(), caster.close()


def no_module_keeps_retrying():
    sim = F9pSim()
    path = sim.path
    sim.close()

    class Gone:
        pass

    gone = Gone()
    gone.path = path
    driver = Driver(gone, ["--mode", "Client", "--ntrip-mountpoint", "X", "--ntrip-username", "a@example.com"])
    try:
        wait_for(lambda: "GPS_CONNECTION_ERROR" in (driver.latest("Errors") or []), 20, "error flag")
        check(driver.process.poll() is None, "must keep retrying the module")
    finally:
        driver.stop()


SCENARIOS = [
    bad_arguments_exit,
    station_survey_then_upload,
    station_restart_to_fixed,
    station_starts_fixed_from_args,
    station_reconnects_after_drop,
    client_receives_corrections,
    client_keeps_position_and_backs_off,
    client_stops_retrying_when_banned,
    exits_when_owner_goes_away,
    no_module_keeps_retrying,
]


def main():
    global DRIVER
    DRIVER = os.path.abspath(sys.argv[1])
    only = sys.argv[2:] or None
    results = {}

    def run(scenario):
        start = time.monotonic()
        try:
            scenario()
            results[scenario.__name__] = f"PASS {time.monotonic() - start:5.1f}s"
        except Exception as e:  # noqa: BLE001
            FAILURES.append(scenario.__name__)
            results[scenario.__name__] = f"FAIL {time.monotonic() - start:5.1f}s  {e}"

    threads = [threading.Thread(target=run, args=(s,)) for s in SCENARIOS if not only or s.__name__ in only]

    # Emulated ARM64 is too slow to start several .NET processes at once: E2E_SERIAL=1 runs one at a time
    if os.environ.get("E2E_SERIAL"):
        for t in threads:
            t.start()
            t.join()
    else:
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    for name, result in results.items():
        print(f"{result:60.200s}  {name}")
    print(f"\n{len(results) - len(FAILURES)} passed, {len(FAILURES)} failed")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
