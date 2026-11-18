"""
run_trial.py - Closed-loop check of the SPM model against the femur IMU.

Commands a motion over RS485, records COMMANDED vs ENCODER-FK vs IMU pose and
the IMU angular velocity at 50 Hz, and prints/saves a summary.  This is the
"did it move the way the model says it should?" test.

    python experiments/run_trial.py --sim                       # everything simulated
    python experiments/run_trial.py --port COM5 --baud 115200   # real drivers, real board
    python experiments/run_trial.py --port COM5 --profile plane --plane-az 45 --amp 20
    python experiments/run_trial.py --port COM5 --profile gait --cycles 3 --trim 1.0

Before a real run: the leg must be at anatomical NEUTRAL (the script sets home
there, exactly like the GUI's "Set home here"), and the IMU mount must be
calibrated (it runs the calibration if imu_cal.json is missing, or with --recal).

Output: experiments/results/trial_<time>_<profile>.csv  (+ _summary.txt)
(Testing/characterisation code: deliberately kept out of the main app.)
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from spm.controller import Controller            # noqa: E402
from spm.imu_control import ang_deg              # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--sim", action="store_true", help="simulated drivers AND simulated IMU")
ap.add_argument("--port", help="RS485 adapter port (e.g. COM5)")
ap.add_argument("--baud", type=int, default=38400)
ap.add_argument("--imu-port", default="", help="IMU board port (default: auto-detect)")
ap.add_argument("--profile", choices=["step", "plane", "gait"], default="step")
ap.add_argument("--amp", type=float, default=20, help="step size / plane amplitude, deg")
ap.add_argument("--plane-az", type=float, default=0, help="plane azimuth (0 sagittal, 90 frontal)")
ap.add_argument("--freq", type=float, default=0.3, help="plane swing frequency, Hz")
ap.add_argument("--cycles", type=float, default=2, help="plane/gait cycles")
ap.add_argument("--period", type=float, default=2.5, help="gait cycle period, s")
ap.add_argument("--rate", type=float, default=60, help="max angular rate, deg/s")
ap.add_argument("--trim", type=float, default=0.0, help="IMU tilt-trim gain (0 = open loop)")
ap.add_argument("--recal", action="store_true", help="force a new IMU mount calibration")
args = ap.parse_args()

out_dir = Path(__file__).with_name("results")
out_dir.mkdir(exist_ok=True)
stamp = time.strftime("%Y%m%d_%H%M%S")
csv_path = out_dir / f"trial_{stamp}_{args.profile}.csv"

ctl = Controller(ROOT / "config.json")
ctl.update_config({"control": {"max_rate_deg_s": args.rate}, "imu": {"trim_gain": 0.0}})
try:
    # ---- connect + home + arm -------------------------------------------
    ctl.connect(args.port or "", args.baud, simulate=args.sim)
    print("Leg must be at NEUTRAL now - setting home here.")
    ctl.set_home()
    ctl.imu.connect("SIM" if args.sim else (args.imu_port or None))
    time.sleep(1.5)
    ctl.arm()

    # ---- IMU calibration if needed ----------------------------------------------
    if args.recal or not ctl.imu.board.est.calibrated:
        print("Calibrating IMU mount (the leg will move)...")
        ctl.imu.start_calibration()
        while ctl.imu.cal_thread.is_alive():
            time.sleep(0.5)
        print("  ", ctl.imu.cal_status)
        if not ctl.imu.board.est.calibrated:
            raise SystemExit("IMU calibration failed")
    else:
        ctl.imu.align()
    ctl.update_config({"imu": {"trim_gain": args.trim}})

    # ---- motion ---------------------------------------------------------------
    if args.profile == "step":
        plan = [(0, 0, 0, 2.0), (args.amp, 0, 0, 3.0), (0, 0, 0, 3.0), (0, args.amp / 2, 0, 3.0),
                (0, 0, 0, 3.0), (0, 0, args.amp / 2, 3.0), (0, 0, 0, 3.0)]
        duration = sum(p[3] for p in plan)
    elif args.profile == "plane":
        ctl.set_mode("plane", {"azimuth_deg": args.plane_az, "elevation_deg": 0, "offset_deg": 0,
                               "amplitude_deg": args.amp, "freq_hz": args.freq})
        ctl.set_running(True)
        duration = args.cycles / args.freq + 2
    else:
        ctl.set_mode("gait", {"period_s": args.period})
        ctl.set_running(True)
        duration = args.cycles * args.period + 3

    # ---- record -----------------------------------------------------------------
    rows = []
    t0 = time.monotonic()
    step_i, step_t = 0, t0
    if args.profile == "step":
        ctl.set_mode("pose", dict(zip(("flex", "add", "rot"), plan[0][:3])))
    while time.monotonic() - t0 < duration:
        if args.profile == "step" and time.monotonic() - step_t > plan[step_i][3] and step_i + 1 < len(plan):
            step_i += 1
            step_t = time.monotonic()
            ctl.set_mode("pose", dict(zip(("flex", "add", "rot"), plan[step_i][:3])))
        s = ctl.snapshot()
        i = s["imu"]
        m = s.get("anat_meas", {})
        a = i.get("anat", {})
        w = i.get("w_pelvis_dps") or [math.nan] * 3
        rows.append([round(time.monotonic() - t0, 4),
                     *(s["anat_cmd"][k] for k in ("flex", "add", "rot")),
                     *(m.get(k, math.nan) for k in ("flex", "add", "rot")),
                     *(a.get(k, math.nan) for k in ("flex", "add", "rot")),
                     *w, i.get("err_vs_cmd_deg", math.nan), i.get("err_vs_enc_deg", math.nan),
                     i.get("tilt_err_vs_enc_deg", math.nan), *(s["theta_rel_deg"]), s["safety"]["ok"]])
        time.sleep(0.02)
    ctl.set_running(False)
    ctl.set_mode("pose", {"flex": 0, "add": 0, "rot": 0})
    time.sleep(2)

    hdr = ["t", "cmd_flex", "cmd_add", "cmd_rot", "enc_flex", "enc_add", "enc_rot",
           "imu_flex", "imu_add", "imu_rot", "w_x_dps", "w_y_dps", "w_z_dps",
           "err_imu_vs_cmd_deg", "err_imu_vs_enc_deg", "tilt_err_imu_vs_enc_deg",
           "ring1_deg", "ring2_deg", "ring3_deg", "safe"]
    with open(csv_path, "w", newline="") as f:
        csv.writer(f).writerows([hdr] + rows)

    A = np.array([r[:-1] for r in rows], float)
    def stat(col):
        v = A[:, hdr.index(col)]
        v = v[np.isfinite(v)]
        return f"mean {v.mean():.2f}  rms {math.sqrt((v ** 2).mean()):.2f}  max {v.max():.2f}" if len(v) else "n/a"
    summary = [f"trial {stamp}  profile={args.profile} amp={args.amp} rate={args.rate} trim={args.trim} sim={args.sim}",
               f"IMU calibration: {ctl.imu.cal_status}",
               f"|IMU - command|  [deg]: {stat('err_imu_vs_cmd_deg')}",
               f"|IMU - encoder|  [deg]: {stat('err_imu_vs_enc_deg')}",
               f"tilt IMU-encoder [deg]: {stat('tilt_err_imu_vs_enc_deg')}",
               f"peak |w| [dps]: {np.nanmax(np.linalg.norm(A[:, 10:13], axis=1)):.1f}",
               f"samples {len(rows)}  -> {csv_path.name}"]
    for k in ("flex", "add", "rot"):
        d = A[:, hdr.index(f'imu_{k}')] - A[:, hdr.index(f'cmd_{k}')]
        d = d[np.isfinite(d)]
        summary.append(f"  {k}: IMU-cmd mean {d.mean():+.2f} deg, rms {math.sqrt((d ** 2).mean()):.2f}")
    print("\n".join(summary))
    csv_path.with_name(csv_path.stem + "_summary.txt").write_text("\n".join(summary) + "\n")
finally:
    ctl.estop()
    ctl.shutdown()
