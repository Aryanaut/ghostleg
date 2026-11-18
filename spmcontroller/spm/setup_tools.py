"""
setup_tools.py - One-off driver configuration over the bus (GUI "Driver setup" tab).

Out of the box every MKS SERVO57E_RS485 has slave address 01, 38400 baud, and
work mode 03 (pulse+direction).  Three drivers with the same address cannot
share a bus, so the procedure is:

  1. Wire ONE driver to the adapter.  Connect at 38400.
  2. "Configure" it: bus closed-loop mode (05), 16 microsteps, respond-but-not-
     active (8C 01 00), axis lock on bus start, stall protection on.
  3. Give it its new address (1, 2 or 3).  Repeat 1-3 for each driver.
  4. Optionally raise the baud on each (115200 recommended: a 52-byte long
     packet takes ~4.5 ms at 115200 vs ~13.5 ms at 38400), then reconnect at
     the new baud.

All functions take an open MotorBus and return a list of human-readable log
lines, so the GUI can show exactly what was sent and what came back.
"""

from __future__ import annotations

from . import protocol as P
from .bus import BusError, MotorBus


def _do(bus: MotorBus, addr: int, frame: bytes, label: str) -> str:
    try:
        st = bus.status_cmd(frame, addr)
        return f"addr {addr}: {label} -> {'OK' if st == 1 else f'status {st}'}   [{frame.hex(' ').upper()}]"
    except BusError as e:
        return f"addr {addr}: {label} -> NO/BAD REPLY ({e})"


def scan(bus: MotorBus, addrs=range(1, 11)) -> list[str]:
    """Ping addresses with 'read version' (0x40); report who answers."""
    out = []
    for a in addrs:
        try:
            data = bus.transact(P.cmd_read(a, P.CMD_READ_VERSION), a, P.CMD_READ_VERSION)
            b = data[0]
            series = "E" if b & 0x80 else "D"
            cal = (b >> 4) & 0x3
            hw = b & 0x0F
            fw = ".".join(str(x) for x in data[1:4])
            out.append(f"addr {a}: FOUND  series={series} hardVer={hw} (3=S57E_RS485) cal={cal} fw=V{fw}")
        except BusError:
            pass
    if not out:
        out.append("No drivers answered on addresses "
                   f"{list(addrs)[0]}-{list(addrs)[-1]} (check baud, A/B wiring, power)")
    return out


def configure(bus: MotorBus, addr: int, current_ma: int | None = None) -> list[str]:
    """Put a driver into the state this application expects."""
    out = [
        _do(bus, addr, P.cmd_set_mode(addr, P.MODE_BUS_CLOSED_LOOP), "work mode 05 (RS485 bus closed loop)"),
        _do(bus, addr, P.cmd_set_microstep(addr, 16), "microstep 16 (F5 speed field = true RPM)"),
        _do(bus, addr, P.cmd_set_respond(addr, respond=True, active=False), "respond=1 active=0 (no unsolicited frames)"),
        _do(bus, addr, P.cmd_set_axis_lock(addr, True), "lock axis when bus mode starts"),
        _do(bus, addr, P.cmd_set_stall_protect(addr, True), "stall protection ON"),
    ]
    if current_ma:
        out.append(_do(bus, addr, P.cmd_set_current(addr, int(current_ma)), f"working current {current_ma} mA"))
    return out


def set_address(bus: MotorBus, addr: int, new_addr: int) -> list[str]:
    # The reply to 8B still comes from the OLD address.
    return [_do(bus, addr, P.cmd_set_address(addr, new_addr), f"set slave address -> {new_addr}")]


def set_baud(bus: MotorBus, addr: int, baud: int) -> list[str]:
    return [_do(bus, addr, P.cmd_set_baud(addr, baud), f"set baud -> {baud}"),
            "Reconnect at the new baud rate for further communication."]


def release_stall(bus: MotorBus, addr: int) -> list[str]:
    return [_do(bus, addr, P.cmd_release_stall(addr), "release stall protection")]


def driver_zero(bus: MotorBus, addr: int) -> list[str]:
    """0x92 - only needed if you want the driver's own axis zero at home;
    the GUI's 'Set home' works in software and does not need this."""
    return [_do(bus, addr, P.cmd_set_zero(addr), "set current axis to zero (0x92)")]
