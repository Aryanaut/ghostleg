"""
bus.py - The RS485 transport, plus a software simulator of three drivers.

Layering:
    MotorBus      high-level, thread-safe: "send these 3 setpoints", "read encoder"
       |
    port object   anything with write(bytes) / read(n) / reset_input_buffer()
       |-- serial.Serial   (real USB-RS485 adapter, via pyserial)
       |-- SimPort         (fake port that emulates MKS drivers in software)

Because SimPort behaves like a serial port at the BYTE level, the simulator
exercises exactly the same framing / checksum / parsing code as the hardware.
"""

from __future__ import annotations

import struct
import threading
import time

from . import protocol as P


class BusError(RuntimeError):
    """Raised when a reply is missing or corrupt."""


# ===========================================================================
# High-level bus
# ===========================================================================

class MotorBus:
    """Thread-safe request/response access to the drivers on one RS485 line.

    RS485 is half duplex: only one device may talk at a time.  Every
    transaction therefore holds `self._lock` from the moment we start writing
    until the reply has been read (or timed out), so the 50 Hz control thread
    and the GUI's "read encoder" / setup buttons can never interleave bytes.
    """

    def __init__(self, port, reply_timeout_s: float = 0.03):
        self.port = port
        self.reply_timeout_s = reply_timeout_s
        self._lock = threading.Lock()
        self.tx_frames = 0          # statistics shown in the GUI
        self.rx_errors = 0

    # ---- low level ------------------------------------------------------

    def _write(self, data: bytes) -> None:
        # One write() call -> bytes leave back-to-back.  The manual (Part 3
        # item 6) warns that a gap of more than one byte-time inside a frame
        # makes the driver treat it as two frames, so never split a frame.
        self.port.write(data)
        self.tx_frames += 1

    def _read_exact(self, n: int) -> bytes:
        """Read exactly n bytes or give up after reply_timeout_s."""
        deadline = time.monotonic() + self.reply_timeout_s
        buf = b""
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self.port.read(n - len(buf))
            if chunk:
                buf += chunk
            else:
                time.sleep(0.0005)
        return buf

    # ---- public API -----------------------------------------------------

    def send(self, data: bytes) -> None:
        """Fire-and-forget (long packets, broadcast frames - no reply comes)."""
        with self._lock:
            self._write(data)

    def transact(self, data: bytes, addr: int, code: int) -> bytes:
        """Send a frame to ONE driver and return the DATA bytes of its reply."""
        with self._lock:
            # Throw away any stale bytes (e.g. a late reply from a previous
            # timed-out request) so they can't be mistaken for this reply.
            self.port.reset_input_buffer()
            self._write(data)
            raw = self._read_exact(P.reply_length(code))
        try:
            return P.parse_reply(raw, addr, code)
        except ValueError as exc:
            self.rx_errors += 1
            raise BusError(f"addr {addr} cmd 0x{code:02X}: {exc or 'no reply'}") from exc

    # ---- convenience wrappers ------------------------------------------

    def read_encoder(self, addr: int) -> int:
        """Multi-turn encoder value (0x4000 per motor rev) - the F5 axis unit."""
        return P.decode_int48(self.transact(P.cmd_read_encoder(addr), addr, P.CMD_READ_ENCODER_ADD))

    def status_cmd(self, data: bytes, addr: int) -> int:
        """Send a command whose reply is a single status byte; return it."""
        code = data[2]
        return P.decode_status(self.transact(data, addr, code))

    def close(self) -> None:
        try:
            self.port.close()
        except Exception:
            pass


def open_serial(port_name: str, baud: int):
    """Open a real USB-RS485 adapter.  Imported lazily so the simulator runs
    even on a machine without pyserial installed."""
    import serial  # pyserial
    return serial.Serial(port=port_name, baudrate=baud, bytesize=8, parity="N",
                         stopbits=1, timeout=0, write_timeout=0.1)


def list_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


# ===========================================================================
# Simulator
# ===========================================================================

class SimMotor:
    """Very small model of one SERVO57E in RS485 closed-loop bus mode.

    State is the encoder axis (counts, 0x4000 per rev).  When a target is set
    the axis moves toward it at the commanded RPM (acceleration ignored -
    good enough to preview trajectories and test the GUI).
    """

    def __init__(self, addr: int):
        self.addr = addr
        self.axis = 0.0            # current encoder count (float for smooth motion)
        self.zero = 0.0            # offset applied by "set zero" (0x92)
        self.target = 0.0
        self.speed_rpm = 0.0
        self.enabled = True
        self.stalled = False       # test hook: locked-rotor protection tripped
        self.powered = True        # test hook: driver power on/off
        self.last_t = time.monotonic()

    def power_cycle_restore(self):
        """Emulate a driver rebooting: multi-turn count restarts at an arbitrary
        single-turn value, so any stored home is now meaningless."""
        import random
        self.powered = True
        self.axis = self.target = float(random.randint(0, P.ENCODER_COUNTS_PER_REV - 1))
        self.zero = 0.0
        self.speed_rpm = 0.0

    def update(self) -> None:
        now = time.monotonic()
        dt = now - self.last_t
        self.last_t = now
        if not self.enabled or self.stalled or not self.powered or self.speed_rpm <= 0:
            return
        step = self.speed_rpm / 60.0 * P.ENCODER_COUNTS_PER_REV * dt   # counts this tick
        err = self.target - self.axis
        self.axis = self.target if abs(err) <= step else self.axis + step * (1 if err > 0 else -1)

    @property
    def reported_axis(self) -> int:
        return int(round(self.axis - self.zero))


class SimPort:
    """A fake serial port.  Bytes written to it are decoded as MKS frames and
    the matching reply bytes are queued for read()."""

    def __init__(self, addrs=(1, 2, 3)):
        self.motors = {a: SimMotor(a) for a in addrs}
        self._rx = bytearray()
        self._lock = threading.Lock()

    # -- serial.Serial-compatible surface --------------------------------
    def write(self, data: bytes) -> int:
        with self._lock:
            for m in self.motors.values():
                m.update()
            if data and data[0] == P.HEAD_LONG:
                self._handle_long(data)
            elif data and data[0] == P.HEAD_DOWN:
                self._handle_frame(data)
        return len(data)

    def read(self, n: int) -> bytes:
        with self._lock:
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx.clear()

    def close(self) -> None:
        pass

    # -- decoding ------------------------------------------------------------
    def _reply(self, addr: int, code: int, payload: bytes) -> None:
        body = bytes([P.HEAD_UP, addr, code]) + payload
        self._rx += body + bytes([P.checksum(body)])

    def _apply(self, addr: int, code: int, payload: bytes) -> bytes | None:
        """Execute one command on the addressed sim motor(s); return reply data."""
        targets = list(self.motors.values()) if addr == P.BROADCAST else (
            [self.motors[addr]] if addr in self.motors else [])
        targets = [m for m in targets if m.powered]     # unpowered drivers are silent
        if not targets:
            return None
        for m in targets:
            if code == P.CMD_ABS_AXIS:
                speed, _acc, ax = struct.unpack(">HBi", payload[:7])
                if speed == 0:                    # F5 with speed 0 == stop
                    m.target, m.speed_rpm = m.axis, 0
                else:
                    m.target, m.speed_rpm = ax + m.zero, speed
            elif code == P.CMD_ENABLE:
                m.enabled = bool(payload[0])
            elif code == P.CMD_ESTOP:
                m.target, m.speed_rpm = m.axis, 0
            elif code == P.CMD_RELEASE_STALL:
                m.stalled = False
            elif code == P.CMD_SET_ZERO:
                m.zero = m.axis
                m.target = m.axis
        m = targets[0]
        if code == P.CMD_READ_ENCODER_ADD:
            return m.reported_axis.to_bytes(6, "big", signed=True)
        if code == P.CMD_READ_VERSION:
            return bytes([0x80 | 0x10 | 3, 1, 0, 0])
        if code == P.CMD_QUERY_STATUS:
            return bytes([1 if abs(m.target - m.axis) < 1 else 4])
        if code == P.CMD_ABS_AXIS:
            return bytes([1])
        if code == P.CMD_READ_STALL:
            return bytes([1 if m.stalled else 0])   # 1 = in stall protection
        return bytes([1])                        # generic "success"

    def _handle_frame(self, data: bytes) -> None:
        if P.checksum(data[:-1]) != data[-1]:
            return                                # real drivers ignore bad frames
        addr, code, payload = data[1], data[2], data[3:-1]
        reply = self._apply(addr, code, payload)
        if reply is not None and addr != P.BROADCAST:
            self._reply(addr, code, reply)

    def _handle_long(self, data: bytes) -> None:
        if len(data) != P.LONG_TOTAL_LEN or P.checksum(data[:-1]) != data[-1]:
            return
        seen = set()
        for i in range(P.LONG_SLOTS):
            slot = data[1 + i * P.LONG_SLOT_LEN: 1 + (i + 1) * P.LONG_SLOT_LEN]
            addr, code = slot[0], slot[1]
            if addr in seen:                      # manual: only first occurrence runs
                continue
            seen.add(addr)
            self._apply(addr, code, slot[2:])     # long packets never reply
