"""
protocol.py - Frame builders / parsers for the MKS SERVO42E/57E RS485 drivers.

Everything here is PURE (no I/O): functions take numbers and return bytes, or
take bytes and return numbers.  That makes it trivially unit-testable against
the worked examples printed in the user manual (see tests/test_protocol.py).

Reference: "MKS SERVO42E/57E_RS485 User Manual V1.0.1"
  Part 3  - serial data format (FA/FB heads, 8-bit CHECKSUM, big-endian data)
  Part 4  - read / set parameter commands (0x30..0x9F)
  Part 4.10 - long data packet (head 0xFC, five 10-byte commands, no reply)
  Part 5  - motion commands (F3 enable, F5 absolute-axis move, F7 e-stop ...)

Frame layout (Part 3):
    downlink  PC -> driver :  FA  addr  code  data...  CRC
    uplink    driver -> PC :  FB  addr  code  data...  CRC
    CRC = (sum of every preceding byte) & 0xFF        ("CHECKSUM 8bit")
"""

from __future__ import annotations

import struct

# ---------------------------------------------------------------------------
# Constants straight out of the manual
# ---------------------------------------------------------------------------

HEAD_DOWN = 0xFA          # PC -> driver frame head
HEAD_UP = 0xFB            # driver -> PC frame head
HEAD_LONG = 0xFC          # long data packet head (Part 4.10)
BROADCAST = 0x00          # broadcast address: every driver executes, none replies

ENCODER_COUNTS_PER_REV = 0x4000   # 16384 counts per motor revolution (Part 4.1 cmd 2)
MAX_SPEED_RPM = 3000              # bus-mode speed ceiling (Part 4.2 note 1, Part 5.1)

# Function codes we use.  Names follow the manual's section titles.
CMD_READ_ENCODER_CARRY = 0x30     # carry(int32) + value(uint16)
CMD_READ_ENCODER_ADD = 0x31       # "addition" value, int48 - the axis F5 targets
CMD_READ_SPEED = 0x32             # int16 RPM
CMD_READ_IO = 0x34
CMD_READ_ANGLE_ERROR = 0x39       # int32, 51200 == 360 deg
CMD_READ_EN = 0x3A
CMD_RELEASE_STALL = 0x3D
CMD_READ_STALL = 0x3E
CMD_READ_VERSION = 0x40
CMD_CALIBRATE = 0x80
CMD_SET_MODE = 0x82
CMD_SET_CURRENT = 0x83
CMD_SET_MICROSTEP = 0x84
CMD_SET_DIR = 0x86
CMD_SET_STALL_PROTECT = 0x88
CMD_SET_BAUD = 0x8A
CMD_SET_ADDRESS = 0x8B
CMD_SET_RESPOND = 0x8C
CMD_SET_GROUP = 0x8D
CMD_SET_AXIS_LOCK = 0x8F
CMD_SET_ZERO = 0x92               # "Set current axis to zero" without moving
CMD_QUERY_STATUS = 0xF1
CMD_ENABLE = 0xF3
CMD_ABS_AXIS = 0xF5               # position mode 4: absolute motion by axis
CMD_SPEED_MODE = 0xF6
CMD_ESTOP = 0xF7

# Work modes for CMD_SET_MODE (Part 4.2 #2).  0x05 = RS485 bus closed loop.
MODE_BUS_CLOSED_LOOP = 0x05

# Baud code table (Part 4.2 #10) -> actual baud
BAUD_CODES = {1: 9600, 2: 19200, 3: 25000, 4: 38400, 5: 57600, 6: 115200, 7: 256000}
BAUD_TO_CODE = {v: k for k, v in BAUD_CODES.items()}

# Status codes returned by F1 "query motor status" (Part 5.2)
MOTOR_STATUS = {0: "query fail", 1: "stopped", 2: "accelerating", 3: "decelerating",
                4: "full speed", 5: "homing", 6: "calibrating"}

# Number of DATA bytes in each uplink reply, keyed by function code.
# Needed so the reader knows how long a reply is before the CRC.
REPLY_DATA_LEN = {
    CMD_READ_ENCODER_CARRY: 6,   # int32 carry + uint16 value
    CMD_READ_ENCODER_ADD: 6,     # int48
    CMD_READ_SPEED: 2,           # int16
    CMD_READ_ANGLE_ERROR: 4,     # int32
    CMD_READ_VERSION: 4,         # byte4 = series|cal|hardVer, bytes5-7 = firmVer[3] (Part 4.7)
}
DEFAULT_REPLY_DATA_LEN = 1       # nearly every other command answers with one status byte


# ---------------------------------------------------------------------------
# Checksum + generic framing
# ---------------------------------------------------------------------------

def checksum(data: bytes) -> int:
    """8-bit additive checksum used by every frame (manual Part 3, item 5).

    Example from the manual:  FA 01 80 00  ->  (0xFA+0x01+0x80+0x00) & 0xFF = 0x7B
    """
    return sum(data) & 0xFF


def frame(addr: int, code: int, payload: bytes = b"") -> bytes:
    """Build a complete downlink frame:  FA addr code payload CRC."""
    if not 0 <= addr <= 0xFF:
        raise ValueError(f"slave address out of range: {addr}")
    body = bytes([HEAD_DOWN, addr, code]) + payload
    return body + bytes([checksum(body)])


def _check_speed_acc(speed_rpm: int, acc: int) -> tuple[int, int]:
    """Clamp/validate the speed (0-3000 RPM) and acceleration (0-255) fields."""
    speed_rpm = int(round(speed_rpm))
    speed_rpm = max(0, min(MAX_SPEED_RPM, speed_rpm))
    acc = int(acc)
    if not 0 <= acc <= 255:
        raise ValueError(f"acc out of range 0..255: {acc}")
    return speed_rpm, acc


# ---------------------------------------------------------------------------
# Motion commands (Part 5)
# ---------------------------------------------------------------------------

def abs_axis_payload(speed_rpm: int, acc: int, abs_axis: int) -> bytes:
    """Data bytes for position mode 4 (F5): speed(uint16) acc(uint8) absAxis(int32).

    Position mode 4 is the one we stream setpoints with, because the manual
    (Part 5.9, note 2) says it "supports real-time updates of speed and
    coordinates" - i.e. a new F5 can be sent while the previous one is still
    running and the driver simply re-targets.  The axis unit is the encoder
    "addition" value: 0x4000 counts per motor revolution.
    """
    speed_rpm, acc = _check_speed_acc(speed_rpm, acc)
    abs_axis = int(abs_axis)
    if not -(2 ** 31) <= abs_axis < 2 ** 31:
        raise ValueError("absAxis must fit in int32")
    return struct.pack(">HBi", speed_rpm, acc, abs_axis)


def cmd_abs_axis(addr: int, speed_rpm: int, acc: int, abs_axis: int) -> bytes:
    """Single-motor F5 frame (11 bytes).  Driver replies FB addr F5 status CRC
    (1 = started, 2 = complete, 3 = end-limit stop) if responses are enabled."""
    return frame(addr, CMD_ABS_AXIS, abs_axis_payload(speed_rpm, acc, abs_axis))


def cmd_abs_axis_stop(addr: int, acc: int = 0) -> bytes:
    """F5 with speed=0 stops the motor (acc=0 -> immediate, acc>0 -> decelerate)."""
    return frame(addr, CMD_ABS_AXIS, struct.pack(">HBi", 0, acc, 0))


def cmd_enable(addr: int, enable: bool) -> bytes:
    """F3: enable (hold torque) / disable (shaft free) the motor in bus mode."""
    return frame(addr, CMD_ENABLE, bytes([1 if enable else 0]))


def cmd_estop(addr: int) -> bytes:
    """F7: emergency stop.  Send to BROADCAST to stop every motor in one frame."""
    return frame(addr, CMD_ESTOP)


def cmd_query_status(addr: int) -> bytes:
    """F1: motor run state (see MOTOR_STATUS)."""
    return frame(addr, CMD_QUERY_STATUS)


# ---------------------------------------------------------------------------
# Read commands (Part 4.1)
# ---------------------------------------------------------------------------

def cmd_read_encoder(addr: int) -> bytes:
    """0x31: read the multi-turn encoder ("addition") value, int48."""
    return frame(addr, CMD_READ_ENCODER_ADD)


def cmd_read(addr: int, code: int) -> bytes:
    """Generic zero-payload read, e.g. cmd_read(1, CMD_READ_SPEED)."""
    return frame(addr, code)


# ---------------------------------------------------------------------------
# Configuration commands (Part 4.2 / 4.4) - used by the driver setup panel
# ---------------------------------------------------------------------------

def cmd_set_mode(addr: int, mode: int = MODE_BUS_CLOSED_LOOP) -> bytes:
    return frame(addr, CMD_SET_MODE, bytes([mode]))


def cmd_set_current(addr: int, milliamps: int) -> bytes:
    """0x83 working current.  57E max 5200 mA (default 3200)."""
    if not 0 <= milliamps <= 5200:
        raise ValueError("57E current must be 0..5200 mA")
    return frame(addr, CMD_SET_CURRENT, struct.pack(">H", milliamps))


def cmd_set_microstep(addr: int, microsteps: int) -> bytes:
    """0x84 subdivision.  Bus speeds are calibrated at 16/32/64 (Part 5.1),
    so we keep 16 to make the F5 speed field read directly in RPM."""
    return frame(addr, CMD_SET_MICROSTEP, bytes([microsteps & 0xFF]))


def cmd_set_baud(addr: int, baud: int) -> bytes:
    if baud not in BAUD_TO_CODE:
        raise ValueError(f"unsupported baud {baud}; choose one of {sorted(BAUD_TO_CODE)}")
    return frame(addr, CMD_SET_BAUD, bytes([BAUD_TO_CODE[baud]]))


def cmd_set_address(addr: int, new_addr: int) -> bytes:
    if not 1 <= new_addr <= 0xFF:
        raise ValueError("new slave address must be 1..255 (0 is broadcast)")
    return frame(addr, CMD_SET_ADDRESS, bytes([new_addr]))


def cmd_set_respond(addr: int, respond: bool, active: bool) -> bytes:
    """0x8C.  respond=1, active=0 means: reply immediately to each command but
    do NOT spontaneously send a second 'move complete' frame later.  We want
    exactly that, so unsolicited frames can never collide with our polling on
    the half-duplex bus."""
    return frame(addr, CMD_SET_RESPOND, bytes([1 if respond else 0, 1 if active else 0]))


def cmd_set_axis_lock(addr: int, lock: bool) -> bytes:
    """0x8F: hold the shaft when bus mode starts (default locked)."""
    return frame(addr, CMD_SET_AXIS_LOCK, bytes([1 if lock else 0]))


def cmd_set_zero(addr: int) -> bytes:
    """0x92: declare the current shaft position to be axis 0 (no motion)."""
    return frame(addr, CMD_SET_ZERO)


def cmd_release_stall(addr: int) -> bytes:
    """0x3D: clear the locked-rotor (stall) protection state."""
    return frame(addr, CMD_RELEASE_STALL)


def cmd_set_stall_protect(addr: int, enable: bool) -> bytes:
    return frame(addr, CMD_SET_STALL_PROTECT, bytes([1 if enable else 0]))


# ---------------------------------------------------------------------------
# Long data packet (Part 4.10)
# ---------------------------------------------------------------------------

LONG_SLOTS = 5          # always five 10-byte command slots
LONG_SLOT_LEN = 10      # each slot = normal command minus FA head and CRC, zero padded
LONG_TOTAL_LEN = 52     # 1 head + 5*10 + 1 checksum


def long_packet(commands: list[tuple[int, int, bytes]]) -> bytes:
    """Bundle up to five (addr, code, payload) commands into ONE 52-byte frame.

    Why we use it: all three motors receive their new setpoints in the same
    frame, so they start moving at the same instant (no 1-3 ms skew between
    legs), and the drivers do not answer (Part 4.10 note 5), so the bus is
    free again immediately.

    Unused slots: the manual fixes the packet at 52 bytes.  Filling a spare
    slot with zeros would put address 0x00 (BROADCAST!) and code 0x00 in it,
    which is undefined behaviour.  Instead we repeat the FIRST command:
    note 4 says that if two slots share an address only the earlier one is
    executed, so the duplicates are guaranteed no-ops.
    """
    if not 1 <= len(commands) <= LONG_SLOTS:
        raise ValueError("long packet takes 1..5 commands")
    addrs = [c[0] for c in commands]
    if len(set(addrs)) != len(addrs):
        raise ValueError("duplicate slave address in long packet (only first would run)")
    if BROADCAST in addrs:
        raise ValueError("do not put the broadcast address in a long packet")

    slots = []
    for addr, code, payload in commands:
        raw = bytes([addr, code]) + payload
        if len(raw) > LONG_SLOT_LEN:
            raise ValueError("command too long for a long-packet slot")
        slots.append(raw.ljust(LONG_SLOT_LEN, b"\x00"))
    while len(slots) < LONG_SLOTS:
        slots.append(slots[0])           # harmless duplicate, see docstring

    body = bytes([HEAD_LONG]) + b"".join(slots)
    pkt = body + bytes([checksum(body)])
    assert len(pkt) == LONG_TOTAL_LEN
    return pkt


def long_abs_axis(targets: list[tuple[int, int, int, int]]) -> bytes:
    """Convenience: one long packet of F5 moves.
    targets = [(addr, speed_rpm, acc, abs_axis), ...]"""
    return long_packet([(a, CMD_ABS_AXIS, abs_axis_payload(s, acc, ax))
                        for a, s, acc, ax in targets])


# ---------------------------------------------------------------------------
# Uplink (reply) parsing
# ---------------------------------------------------------------------------

def reply_length(code: int) -> int:
    """Total uplink frame length (head+addr+code+data+crc) for a function code."""
    return 3 + REPLY_DATA_LEN.get(code, DEFAULT_REPLY_DATA_LEN) + 1


def parse_reply(buf: bytes, expect_addr: int, expect_code: int) -> bytes:
    """Validate an uplink frame and return just its DATA bytes.

    Raises ValueError on wrong head / address / code / checksum so the caller
    can count it as a comms error rather than act on garbage.
    """
    n = reply_length(expect_code)
    if len(buf) != n:
        raise ValueError(f"reply length {len(buf)} != expected {n}: {buf.hex(' ')}")
    if buf[0] != HEAD_UP:
        raise ValueError(f"bad head 0x{buf[0]:02X}")
    if buf[1] != expect_addr:
        raise ValueError(f"reply from addr {buf[1]}, expected {expect_addr}")
    if buf[2] != expect_code:
        raise ValueError(f"reply code 0x{buf[2]:02X}, expected 0x{expect_code:02X}")
    if checksum(buf[:-1]) != buf[-1]:
        raise ValueError(f"checksum mismatch in {buf.hex(' ')}")
    return buf[3:-1]


def decode_int48(data: bytes) -> int:
    """Big-endian signed 48-bit integer (encoder 'addition' value, cmd 0x31)."""
    return int.from_bytes(data, "big", signed=True)


def decode_status(data: bytes) -> int:
    return data[0]
