#!/usr/bin/env python3
"""
A software ZED-F9P for testing the driver without hardware.

It answers the UBX commands the driver sends on a pseudo terminal, and behaves like the module on the
points that matter: survey-in takes a few seconds, RTCM output follows UBX-CFG-MSG, RTCM input from a
caster moves the solution from 3D to RTK fixed. Everything it sees is kept for tests to assert on.

Linux only (uses pty). It is not a GNSS simulator: positions are constants.
"""
import os
import pty
import struct
import threading
import time
import tty

SYNC = b"\xb5\x62"
CLS_NAV, CLS_ACK, CLS_CFG, CLS_MON = 0x01, 0x05, 0x06, 0x0A
ID_CFG_PRT, ID_CFG_MSG, ID_CFG_RATE, ID_CFG_TMODE3 = 0x00, 0x01, 0x08, 0x71
ID_NAV_PVT, ID_NAV_HPPOSECEF, ID_NAV_HPPOSLLH, ID_NAV_SVIN = 0x07, 0x13, 0x14, 0x3B
ID_MON_VER = 0x04
RTCM_MSG_CLASS = 0xF5


def ubx_frame(cls, msg_id, payload=b""):
    body = bytes([cls, msg_id]) + struct.pack("<H", len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return SYNC + body + bytes([ck_a, ck_b])


def rtcm_frame(message_type, length=60):
    """A correctly framed RTCM3 message. The payload is filler and the CRC is not computed."""
    payload = struct.pack(">H", (message_type << 4) & 0xFFFF) + bytes(length - 2)
    return b"\xd3" + struct.pack(">H", length & 0x3FF) + payload + b"\x00\x00\x00"


class F9pSim:
    def __init__(self, survey_seconds=3.0, latitude=40.1234567, longitude=-80.1234567, height_m=300.0):
        self.survey_seconds = survey_seconds
        self.latitude = latitude
        self.longitude = longitude
        self.height_m = height_m

        self.master, self.slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)

        self.lock = threading.Lock()
        self.running = True
        self.tmode3 = bytearray(40)
        self.survey_started = None
        self.rtcm_rates = {}
        self.rate_ms = 1000

        self.rtcm_received = bytearray()
        self.rtcm_sent_frames = []
        self.commands = []

        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._rtcm_output, daemon=True).start()

    # What tests look at -------------------------------------------------------------------------

    @property
    def mode(self):
        return self.tmode3[2]

    @property
    def fixed_position(self):
        """(lat 1e-7, lon 1e-7, height cm, lat hp, lon hp, height hp, LLA flag)"""
        lat, lon, height = struct.unpack_from("<iii", self.tmode3, 4)
        lat_hp, lon_hp, height_hp = struct.unpack_from("<bbb", self.tmode3, 16)
        return lat, lon, height, lat_hp, lon_hp, height_hp, bool(self.tmode3[3] & 1)

    @property
    def carrier_solution(self):
        return 2 if len(self.rtcm_received) > 0 else 0

    def close(self):
        self.running = False
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    # Serial side --------------------------------------------------------------------------------

    def _write(self, data):
        try:
            with self.lock:
                os.write(self.master, data)
        except OSError:
            pass

    def _reader(self):
        buffer = bytearray()
        while self.running:
            try:
                chunk = os.read(self.master, 4096)
            except OSError:
                return
            buffer.extend(chunk)
            self._parse(buffer)

    def _parse(self, buffer):
        while buffer:
            if buffer[0] == 0xB5:
                if len(buffer) < 6:
                    return
                length = struct.unpack_from("<H", buffer, 4)[0]
                if len(buffer) < 8 + length:
                    return
                frame = bytes(buffer[: 8 + length])
                del buffer[: 8 + length]
                self._handle_ubx(frame[2], frame[3], frame[6:-2])
            elif buffer[0] == 0xD3:
                if len(buffer) < 3:
                    return
                length = struct.unpack_from(">H", buffer, 1)[0] & 0x3FF
                if len(buffer) < length + 6:
                    return
                self.rtcm_received.extend(buffer[: length + 6])
                del buffer[: length + 6]
            else:
                # Noise between frames, such as the CR/LF after a caster's status line
                del buffer[0]

    def _ack(self, cls, msg_id):
        self._write(ubx_frame(CLS_ACK, 0x01, bytes([cls, msg_id])))

    def _handle_ubx(self, cls, msg_id, payload):
        self.commands.append((cls, msg_id, bytes(payload)))

        if cls == CLS_CFG:
            self._handle_cfg(msg_id, payload)
        elif cls == CLS_NAV:
            handler = {
                ID_NAV_PVT: self._nav_pvt,
                ID_NAV_HPPOSLLH: self._nav_hpposllh,
                ID_NAV_HPPOSECEF: self._nav_hpposecef,
                ID_NAV_SVIN: self._nav_svin,
            }.get(msg_id)
            if handler:
                self._write(ubx_frame(CLS_NAV, msg_id, handler()))
        elif cls == CLS_MON and msg_id == ID_MON_VER:
            self._write(ubx_frame(CLS_MON, ID_MON_VER, self._mon_ver()))

    def _handle_cfg(self, msg_id, payload):
        if msg_id == ID_CFG_PRT:
            if len(payload) <= 1:
                reply = bytearray(20)
                reply[0] = payload[0] if payload else 3
                self._write(ubx_frame(CLS_CFG, ID_CFG_PRT, bytes(reply)))
        elif msg_id == ID_CFG_RATE:
            if len(payload) == 0:
                self._write(ubx_frame(CLS_CFG, ID_CFG_RATE, struct.pack("<HHH", self.rate_ms, 1, 1)))
            else:
                self.rate_ms = struct.unpack_from("<H", payload, 0)[0]
        elif msg_id == ID_CFG_MSG:
            if len(payload) == 2:
                self._write(ubx_frame(CLS_CFG, ID_CFG_MSG, bytes(payload) + bytes(6)))
            elif len(payload) >= 8 and payload[0] == RTCM_MSG_CLASS:
                # The USB rate is the fourth of the six port rates
                self.rtcm_rates[payload[1]] = payload[2 + 3]
        elif msg_id == ID_CFG_TMODE3:
            if len(payload) == 0:
                self._write(ubx_frame(CLS_CFG, ID_CFG_TMODE3, bytes(self.tmode3)))
            else:
                self.tmode3 = bytearray(payload)
                self.survey_started = time.monotonic() if self.tmode3[2] == 1 else None
        self._ack(CLS_CFG, msg_id)

    # Messages -----------------------------------------------------------------------------------

    def _nav_pvt(self):
        message = bytearray(92)
        message[20] = 3  # 3D fix
        message[21] = self.carrier_solution << 6
        message[23] = 24  # satellites
        struct.pack_into("<ii", message, 24, int(self.longitude * 1e7), int(self.latitude * 1e7))
        struct.pack_into("<ii", message, 32, int(self.height_m * 1000), int((self.height_m - 34) * 1000))
        struct.pack_into("<i", message, 60, 500)  # 0.5 m/s
        struct.pack_into("<i", message, 64, 9000000)  # 90 degrees
        return bytes(message)

    def _nav_hpposllh(self):
        message = bytearray(36)
        struct.pack_into("<ii", message, 8, int(self.longitude * 1e7), int(self.latitude * 1e7))
        struct.pack_into("<ii", message, 16, int(self.height_m * 1000), int((self.height_m - 34) * 1000))
        struct.pack_into("<I", message, 28, 140 if self.carrier_solution == 2 else 14000)  # 14 mm or 1.4 m in 0.1 mm
        return bytes(message)

    def _nav_hpposecef(self):
        message = bytearray(28)
        struct.pack_into("<I", message, 24, 200)
        return bytes(message)

    def _survey_elapsed(self):
        if self.survey_started is None:
            return 0.0
        return time.monotonic() - self.survey_started

    def _nav_svin(self):
        message = bytearray(40)
        struct.pack_into("<I", message, 8, int(self._survey_elapsed()))
        struct.pack_into("<I", message, 28, 25000)  # 2.5 m
        done = self.survey_started is not None and self._survey_elapsed() >= self.survey_seconds
        message[36] = 1 if done else 0
        message[37] = 0 if done else 1
        return bytes(message)

    def _mon_ver(self):
        return b"ROM CORE 1.00".ljust(30, b"\x00") + b"00190000".ljust(10, b"\x00") + b"PROTVER=27.11".ljust(30, b"\x00")

    def _rtcm_output(self):
        types = {0x05: 1005, 0x4A: 1074, 0x54: 1084, 0x5E: 1094, 0x7C: 1124}
        while self.running:
            time.sleep(self.rate_ms / 1000.0)
            ready = self.mode == 2 or (self.mode == 1 and self._survey_elapsed() >= self.survey_seconds)
            if not ready:
                continue
            for key, message_type in types.items():
                if self.rtcm_rates.get(key):
                    frame = rtcm_frame(message_type)
                    self.rtcm_sent_frames.append(frame)
                    self._write(frame)
