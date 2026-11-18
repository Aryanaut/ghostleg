"""
server.py - Local web server for the SPM hip controller.

Run:   python server.py            then open http://localhost:8765
       python server.py --port 9000 --no-browser

Architecture
    browser (static/index.html + app.js, three.js 3D view)
        <-- WebSocket JSON -->  this server  -->  Controller thread  -->  RS485
The browser never talks to the serial port; it only sends intents
("set mode", "arm", "e-stop") and renders the ~30 Hz state snapshots we push.
If the browser tab closes, motion simply holds its last commanded pose.
"""

from __future__ import annotations

import argparse
import asyncio
import functools
import json
import sys
import webbrowser
from pathlib import Path

from aiohttp import WSMsgType, web

from spm import setup_tools
from spm.controller import Controller

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
PUSH_HZ = 30


def _np_default(o):
    """JSON fallback for numpy scalars/arrays that slip into snapshots."""
    import numpy as np
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")


dumps = functools.partial(json.dumps, default=_np_default)


async def index(_req):
    return web.FileResponse(STATIC / "index.html")


def handle_message(ctl: Controller, msg: dict):
    """Dispatch one command from the GUI.  Returns an optional reply dict.
    Raising an exception sends {"type": "error"} back to the browser."""
    t = msg.get("type")

    if t == "connect":
        ctl.connect(msg.get("port", ""), int(msg.get("baud", 38400)), bool(msg.get("simulate")))
        ctl.update_config({"serial": {"port": msg.get("port", ""), "baud": int(msg.get("baud", 38400))}})
    elif t == "disconnect":
        ctl.disconnect()
    elif t == "set_home":
        ctl.set_home()
    elif t == "torque":
        ctl.set_torque(bool(msg["on"]))
    elif t == "arm":
        ctl.arm()
    elif t == "disarm":
        ctl.disarm()
    elif t == "view_override":                      # show poses beyond the limits (viewing only)
        ctl.set_view_override(bool(msg.get("on")))
    elif t == "estop":
        ctl.estop()
    elif t == "reset_estop":
        ctl.reset_estop()
    elif t == "release_stalls":
        ctl.release_stalls()
    elif t == "imu_connect":
        ctl.imu.connect(msg.get("port") or None)
    elif t == "imu_disconnect":
        ctl.imu.disconnect()
    elif t == "imu_calibrate":
        ctl.imu.start_calibration(float(msg.get("tilt_deg", 15)), float(msg.get("twist_deg", 25)))
    elif t == "imu_align":
        ctl.imu.align()
    elif t == "mode":
        ctl.set_mode(msg["name"], msg.get("params", {}))
        return {"type": "static", **ctl.static_info()}       # gait preview may have changed
    elif t == "run":
        ctl.set_running(bool(msg["on"]))
    elif t == "reset_time":
        ctl.reset_time()
    elif t == "config":
        ctl.update_config(msg["patch"], save=bool(msg.get("save")), keep_femur=bool(msg.get("keep_femur")))
        return {"type": "static", **ctl.static_info()}
    elif t == "static":
        return {"type": "static", **ctl.static_info()}
    elif t == "setup":
        # Driver configuration helpers - only while connected and disarmed.
        if not ctl.connected or ctl.bus is None:
            raise RuntimeError("Connect to the bus first")
        if ctl.armed:
            raise RuntimeError("Disarm before using driver setup")
        a = int(msg.get("addr", 1))
        act = msg["action"]
        with ctl.lock:
            if act == "scan":
                lines = setup_tools.scan(ctl.bus, range(1, int(msg.get("max_addr", 10)) + 1))
            elif act == "configure":
                lines = setup_tools.configure(ctl.bus, a, msg.get("current_ma"))
            elif act == "set_address":
                lines = setup_tools.set_address(ctl.bus, a, int(msg["new_addr"]))
            elif act == "set_baud":
                lines = setup_tools.set_baud(ctl.bus, a, int(msg["baud"]))
            elif act == "release_stall":
                lines = setup_tools.release_stall(ctl.bus, a)
            elif act == "driver_zero":
                lines = setup_tools.driver_zero(ctl.bus, a)
            else:
                raise RuntimeError(f"unknown setup action {act}")
        for line in lines:
            ctl.log(line)
    else:
        raise RuntimeError(f"unknown message type {t!r}")
    return None


async def ws_handler(request):
    ctl: Controller = request.app["ctl"]
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    await ws.send_json(dumps=dumps, data={"type": "static", **ctl.static_info()})

    async def pusher():
        # Stream state snapshots to this client.
        while not ws.closed:
            try:
                await ws.send_json(dumps=dumps, data={"type": "state", **ctl.snapshot()})
            except (ConnectionResetError, RuntimeError):
                break
            except Exception as exc:               # a bad snapshot must not kill the stream
                ctl.log(f"state push error: {exc!r}")
                await asyncio.sleep(1)
            await asyncio.sleep(1 / PUSH_HZ)

    push_task = asyncio.create_task(pusher())
    try:
        async for m in ws:
            if m.type != WSMsgType.TEXT:
                continue
            try:
                msg = json.loads(m.data)
                # Serial I/O can block for tens of ms - keep it off the event loop.
                reply = await asyncio.to_thread(handle_message, ctl, msg)
                if reply:
                    await ws.send_json(dumps=dumps, data=reply)
            except Exception as exc:
                ctl.log(f"ERROR: {exc}")
                await ws.send_json(dumps=dumps, data={"type": "error", "msg": str(exc)})
    finally:
        push_task.cancel()
    return ws


def main():
    ap = argparse.ArgumentParser(description="SPM hip joint controller (web GUI)")
    ap.add_argument("--port", type=int, default=8765, help="HTTP port (default 8765)")
    ap.add_argument("--config", default=str(ROOT / "config.json"))
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    args = ap.parse_args()

    ctl = Controller(Path(args.config))
    app = web.Application()
    app["ctl"] = ctl
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", STATIC)

    async def on_shutdown(_app):
        ctl.shutdown()
    app.on_shutdown.append(on_shutdown)

    url = f"http://localhost:{args.port}"
    print(f"SPM controller running at {url}  (Ctrl+C to quit)")
    if not args.no_browser:
        webbrowser.open(url)
    # Bind to localhost only: the GUI can move hardware, so don't expose it on the LAN.
    web.run_app(app, host="127.0.0.1", port=args.port, print=None)


if __name__ == "__main__":
    sys.exit(main())
