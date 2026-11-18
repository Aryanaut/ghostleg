# HANDOFF — SPM hip-joint controller + femur IMU (as of 2026-11-18 11:50 SGT)

> **CURRENT STATE FIRST (2026-11-18 11:50):** the ACTIVE `config.json` is the **v2 redesign** (`bone-in SPM 2.step`):
> α1 45°, **6:1** gearing (108T internal rings, 18T pinions), mount 38.0° down / 8.3° forward from the orange
> ground face, neutral = femur standing. v2 collision limits are in place (§6.21). The v1 notes below describe the FIRST design
> (α1 40°, 4:1); v1 config: `config.v1_before_v2_2026-11-18.json`. Remaining v2 gap: the leg 1 elbow needs about 0.3 mm of
> relief for the full gait with margin (§6.21). 32/32 tests pass. Motors still never driven; with 6:1 gearing,
> `control.gear_ratio` = 6 converts ring angles to motor counts, so confirm the build matches before arming.

This document is for the next session (Claude Code, running locally on the Windows PC).
Read it fully before touching hardware. Everything below reflects the files that are
**actually in this folder** right now (checksums were verified after the last copy).

---------------------------------------------------------------------------------------------
## 0. TL;DR

* `spmcontroller/` is a **browser-based controller** (Python `aiohttp` server and a three.js GUI) for a
  **coaxial 3-RRR spherical parallel manipulator (SPM)** that acts as the **hip joint of a leg phantom**.
  Three MKS SERVO57E closed-loop steppers drive the SPM over **RS485**, each with a **4:1 gear** to its ring.
* A **femur-mounted ESP32-S3 "Space Bar training board"** streams raw IMU data over **USB**. This was
  added for closed-loop verification and correction. The firmware was modified on git branch
  `spm-imu-usb-stream`, **built with the user's ESP-IDF v5.5.2 and flashed successfully** (COM3).
  The stream was verified on hardware.
* Everything works **in simulation**: 25/25 unit tests pass, plus a headless-browser end-to-end test.
  **The SPM motors have never been driven from this software.** The RS485 adapter was not plugged in
  (only COM3 = the IMU board exists). The **real IMU mount calibration has not been run**, because it
  needs the motors.
* **FIXED 22:45 (host side, no reflash):** the IMU really samples at **230.35 Hz**, but the firmware
  timestamps samples on a nominal 240 Hz clock. The PC now measures the true rate (`SampleClock`) and
  times samples as seq × true period. Verified on the real board. See §6.1.
* **GEOMETRY NOW FROM CAD (23:00):** `bone-in SPM.step` (Onshape export, in this folder) was parsed by
  `experiments/cad_axes.py`. α1=40, α2=90, β=90, η=[-177.18,-57.18,62.82], **branch −1** (was +1), mount
  tilt_down 24.64 / forward **−20.07 (posterior)** / twist 91.45, neutral tilt_x 15.45 / tilt_y −9.11 (CAD pose
  = anatomical neutral, RIGHT femur, confirmed by user). Old config saved as `config.before_cad_2026-11-17.json`. See §2.1.
* **COLLISION LIMITS NOW FROM CAD (23:30):** `experiments/cad_collision.py` samples every CAD surface (1.5 mm) and
  moves the parts with the app's own IK. Result: arms touch at **43.5° ring separation** (config 45 with margin), plus
  per-leg **elbow/wrist fold limits** (`limits.joint_limits`, checked in `check_pose`). Every other pair keeps ≥ 4.8 mm.
  **Consequence: the real ROM is flex ≤ +26°, add ≤ +17°, int. rot ≤ +25°; the built-in gait (31.7° flex) is
  refused for 43% of its cycle.** See §6.13.
* **REMOUNT STUDY DONE (23:55), user to decide** (§6.14): the current mount cannot do gait at any arm width (the gait drives
  two rings through each other). Recommended option A: neutral tilt_x −10 / tilt_y +5 / twist 0, i.e. mount tilt_down 38.3,
  forward +10.2, base_twist 93.5 → gait fits up to ×1.75 (×1.45 worst case with ±5°/±10° build error). config.json is
  NOT changed; it still describes the rig as built.
* **View beyond limits (00:05):** Motion-tab checkbox `view-override` → `Controller.set_view_override()`. Refused but
  IK-reachable poses are SHOWN (`safety.overridden`). It is guarded three ways: it cannot be enabled while armed, `arm()`
  refuses while it is on, and the send step skips the bus while it is on (`test_view_override_shows_but_never_sends`).
  Switching off restores the last gate-passing pose (`_safe_R/_safe_theta`). Works in all modes incl. Gait; the Gait scale
  sliders widen to ×3 while on. `Controller.gait_preview()` adds per-sample `ok`/`reason` to the gait preview (IK chained
  from home); the Gait plot shades refused phases and `#g-limits` summarises them (today: 44% of the built-in cycle, all
  "rings too close"; flexion WITH adduction closes the rings from ~14° flex, not just past 26°).
  `test_gait_preview_flags_refused_phases`. NOTE: a server started before these changes must be restarted to get them.
* **Next:** the user picks a remount (or not), then commission the motors with them present.

---------------------------------------------------------------------------------------------
## 1. The user and how they want you to work

* Marcus (SUTD, Capstone 1). Windows PC, user folder `C:\Users\Penelope\...`.
  Project folder: `C:\Users\Penelope\Documents\y4t1\Capstone 1\spmcontroller`.
* Standing preferences (from the user's profile, they apply):
  * **Lots of comments** that explain the logic flow in all code.
  * **Never work in git worktrees**: edit the local files directly.
  * **Testing / recording / characterisation code goes on a testing branch, never polluting main.**
    `spmcontroller` is **not a git repo**, so such code has been kept in `experiments/`. The firmware change
    is on its own branch (§4). Consider `git init` for spmcontroller and moving `experiments/`
    onto a `testing` branch, but **ask first**.
  * **Verify claims with web research / primary sources** (datasheets, manuals, standards) before stating them.
  * At ~80% context, **stop and write/refresh HANDOFF.md** (this file); don't let the session compact.
  * If you launch multiple agents, make each log its work continuously to files in a local folder.
* The user may be at the rig. **Ask before any action that moves hardware** unless they told you to.

---------------------------------------------------------------------------------------------
## 2. Hardware (what is known vs assumed)

### 2.1 SPM
* Coaxial 3-RRR SPM: three concentric internal rings all turn about the **same base axis**. Each ring
  carries a proximal link, then a revolute "middle" joint, then a curved distal link to the end effector.
  All joint axes pass through the centre of rotation.
* Each ring is driven by a stepper through a **4:1 parallel-axis helical gear stage**: an 18T pinion meshing with a 72T
  ring (motor turns 4× per ring turn). It is NOT bevel. This is from the CAD: part names `pinion1/2/3 gear 18T`,
  `Helical 1M 72T` / `internal gear 72T`; pinion axes are parallel to the ring axis at a 45.0 mm centre distance
  = (18+72)/2 × module 1; ring root radius 34.75 = 36 − 1.25 × 1. Module 1 is therefore the TRANSVERSE module in this CAD.
* The **end effector was originally an orange ring** (CAD screenshots). The user then said
  **"this model replaces the orange ring"**: a **femoral-head ball with a cylindrical rod through it,
  attached to a full femur model**. The centre of rotation = the femoral head centre.
* **Geometry measured from the CAD** (`bone-in SPM.step`, Onshape AP242, metres; parsed with no CAD library by
  `experiments/cad_axes.py`, which reads the parts, their assembly transforms, and every cylindrical surface):
  * Parts: `casing` (base), rings `internal gear 72T` (z 9.55) / `Helical 1M 72T` (z 21.95, 34.35), each driven by
    `pinion1/2/3 gear 18T` (18:72 = the 4:1), each carrying `arm1/2/3` (proximal link, 2 pins) → `middle arm`
    (distal link) → `femur head` (effector), then `upper femur` / `lower femur`, and a `positioning key`.
    **Leg i = pinion i = arm i**; assumed motor address i (to be confirmed at commissioning).
  * All 7 joint axes (ring, 3 middle, 3 head) meet at one point, the centre of rotation (CAD (−5.39, 33.68, 117.55) mm),
    with a residual of **0.0000 mm**. **α1 = 40.0°**, **α2 = 90.0°** for every leg. The middle-joint bore centre is at 75 mm
    and the head-joint bore centre at 55 mm.
  * Femur head: its three joint axes are exactly **90° from its symmetry axis and 120° apart** (β=90). That axis is the
    head's screw-hole axis, i.e. the **femoral neck** (124° from the mechanical axis). Effector z := that axis.
  * Base frame = CAD axes moved to the centre (z = ring axis, base→head; θ=0 along CAD +x). Effector x chosen so
    neutral has no twist → **η = [−177.182, −57.179, 62.82]**. **branch = [−1, −1, −1]**; IK reproduces the CAD ring
    angles (80.82, −154.20, −3.20)° to 0.003°, and FK returns the CAD pose exactly (`test_config_reproduces_cad_pose`).
* **Neutral = the CAD pose** (assumption: the assembly was saved in the standing pose; the user's screenshot
  "is what it should look like"). It is **17.9° tilted from the symmetric home** (`neutral.tilt_x 15.449, tilt_y −9.108`),
  so the GUI's "tilt" reads 17.9° at neutral, and `max_tilt_deg` counts from the symmetric home.
* **Pelvis frame / mount**, from the femur in the CAD (**right femur**, confirmed by the user): Y = knee centre → head
  centre (mechanical axis, 445.6 mm, hence `femur.mech_length_m 0.446`); Z (lateral) = widest direction of the distal
  40 mm (condyles; ~±5° uncertain, since it comes from sparse mesh vertices); X = Y×Z (anterior). Sanity: shaft-to-mech 5.6°,
  anterior shaft bow +20 mm, neck anteversion +2.8°. Result: the ring axis points lateral, **24.6° down and 20.1°
  POSTERIOR** (`forward_deg −20.07`, `base_twist_deg 91.45`). My old guess was 45° down and 20° forward.
* The femur stick model now draws the neck along the effector z axis (`anatomy.py`), so it follows the real neck.
* **Link collisions are modelled from the CAD** (§6.13): ring separation ≥ 45° (contact 43.5°) and elbow/wrist fold
  limits. Resulting ROM from neutral: flex −84 (excursion limit) … **+26**, add −34.5 … **+17**, int. rot −49.5 … **+25**.

### 2.2 Motor drivers: MKS SERVO57E RS485 (manual V1.0.1, uploaded by the user)
Facts used (all from the manual, and the protocol unit tests reproduce its example frames):
* Frame `FA addr code data… CRC`, reply `FB …`. CRC = 8-bit sum. Big-endian data. Addr 0 = broadcast (no reply).
* Work mode **0x82 → 05 = RS485 bus closed loop**. Default is 03 (pulse+dir), **address 1, 38400 baud**.
* **F5 = position mode 4, absolute by encoder axis**: `speed u16 (RPM) | acc u8 | absAxis i32`.
  It supports **real-time retargeting** while moving, which is why it is used for 50 Hz streaming.
* Encoder "addition" value via **0x31** (int48), **0x4000 counts per motor rev**.
* **Long packet 0xFC**: 5 × 10-byte slots, 52 bytes, no reply. Duplicate address → only the first executes.
  Unused slots are filled with copies of slot 0 (a zero slot would be the broadcast address).
* Speed field is true RPM only at 16/32/64 microsteps (setup sets 16).
* 0x8C `respond=1, active=0`: no unsolicited "move complete" frames, needed for half-duplex polling.
* F7 = e-stop, F3 = enable, 0x3D/0x3E = release/read stall protection, 0x92 = set axis zero, 0x8A = baud.

### 2.3 Femur IMU board ("Space Bar Avionics Training Board", ESP32-S3-WROOM-1U N16R8)
* Rigidly mounted on the femur standoff; **mounting orientation unknown**. It is solved by the mount calibration (§5.3).
* Sensors: **LSM6DSV80X** (SPI; low-g ±16 g, high-g ±64 g, gyro ±2000 dps; FIFO; ODR set to 240 Hz),
  **MMC5983MA** magnetometer (I2C), **BMP390** barometer. The board also logs to NOR/SD and streams
  29-byte IMU frames on **UART0 TX (GPIO43) at 921600** to "Avionics C" (rocket flight computer). That path is unchanged.
* Plugged into the PC by USB-C (native USB Serial/JTAG, **VID 303A PID 1001**), currently **COM3**.

---------------------------------------------------------------------------------------------
## 3. Repository map (spmcontroller)

```
server.py              aiohttp server: static GUI + WebSocket /ws (JSON in/out, 30 Hz state push). 127.0.0.1:8765
run_windows.bat        pip install -r requirements.txt; python server.py (opens browser)
config.json            ALL tunables: geometry, mount, neutral, femur, motors, control, limits, serial, visual, imu
requirements.txt       numpy, aiohttp, pyserial
example_gait.csv       sample CSV for Gait mode (phase,flex,add,rot)
README.md              user docs (start-up, E-stop recovery, commissioning, IMU, experiments)
HANDOFF.md             this file
imu_cal.json           (created by the first REAL mount calibration - does not exist yet)
imu_cal_sim.json       (created when using the simulated IMU "SIM"; never mixed with the real one)
spm/
  protocol.py          pure MKS frame builders/parsers (+ long packet)
  bus.py               MotorBus (thread-safe RS485 transactions) + SimPort/SimMotor (byte-level driver simulator)
  kinematics.py        coaxial SPM: closed-form IK per leg, Newton FK, singularity metrics, drawing polylines
  anatomy.py           ISB hip angles <-> effector orientation; base-on-pelvis mount; femur stick model
  motion.py            modes: Pose, Plane (arbitrary plane swing), Gait (Catmull-Rom curve or CSV), Joint jog, Effector
  controller.py        50 Hz loop: mode target -> rate limit -> IK -> safety gate -> counts -> FC long packet;
                       encoder feedback (round-robin, 1 motor/tick) -> FK; E-stop latch + recovery; IMU hooks
  setup_tools.py       driver setup: scan, configure (mode 05, 16 ustep, 8C 01 00, axis lock, stall prot), address, baud
  imu_board.py         USB frame parser, ImuBoard serial thread, OrientationEstimator, MountCalibration, CLI
  imu_control.py       IMU <-> controller: connect, mount calibration routine (moves the rig), align, compare, tilt trim
  imu_sim.py           simulated IMU board (reads sim motor shafts, unknown mount, bias, noise, 1.5 deg "compliance")
static/                index.html, app.js (all drawing; NO kinematics in JS), style.css, vendor/three.module.js (r170), OrbitControls.js
tests/test_all.py      32 tests; run `python tests/test_all.py` (no pytest needed) - ~2 min (sim tests sleep)
experiments/           TESTING CODE (kept out of the app on purpose)
  imu_check.py/.bat    8 s USB-stream smoke test -> imu_check_log.txt (+ imu_raw.bin raw bytes)
  cad_axes.py          STEP (Onshape) -> parts, placements, joint axes; no CAD library needed (~7 s)
  cad_collision.py     STEP -> surface points -> part clearances vs pose -> limits for config.json (~7 min)
  remount_study.py     scan neutral/mount options for gait headroom (~25 s); remount_verify.py: candidate details + CAD check
  cad_cache/           cached surface samples / layout / limits_result.json (safe to delete)
  run_trial.py         closed-loop trial: step/plane/gait, records cmd vs encoder vs IMU -> results/*.csv + _summary.txt
firmware/              COPY of the modified ESP-IDF project (MARCUS-ESP32-DEV-BOARD-Software), see section 4
```

---------------------------------------------------------------------------------------------
## 4. Firmware (IMU board): where it is and how it was built

* Source origin: the user uploaded `Space-Bar-Avionics-Training-Board.zip` (a git repo, on branch
  `Avionics-C-Sensor-Cluster`; remotes origin/main, yan-ning, Kiran, JuniorBryan). **The user's real
  clone of that repo is somewhere on the PC; its location is unknown.** Ask before touching it.
* I created branch **`spm-imu-usb-stream`** (2 commits on top of `09a882d`) in a copy in the cloud sandbox:
  * `65de5b4 feat: stream raw IMU + mag over USB Serial/JTAG for the SPM hip rig`
  * `f474fb4 docs: describe the USB sensor stream`
  These are delivered as `firmware/0001-…patch`, `firmware/0002-…patch` and `firmware/spm-imu-usb-stream.bundle`
  (bundle of `Avionics-C-Sensor-Cluster..spm-imu-usb-stream`). To bring them into the real repo:
  `git checkout -b spm-imu-usb-stream Avionics-C-Sensor-Cluster && git am firmware/000*.patch`
  (or `git fetch <path>/spm-imu-usb-stream.bundle spm-imu-usb-stream`). **Do not merge into main/flight branches.**
* Files changed: new `main/usb_stream.{c,h}`; hooks in `main/bridge.c` (send each sample),
  `main/magnetometer.c` (send each mag read), `main/main.c` (`usb_stream_init()` first thing in app_main),
  `main/CMakeLists.txt` (+usb_stream.c, PRIV_REQUIRES esp_driver_usb_serial_jtag esp_ringbuf), README section.
* **USB frame format:** `A5 96 | type | len | payload | CRC-16/CCITT-FALSE LE over type,len,payload`.
  Types: `01 IMU` (27 B: u32 seq, u32 t_us, i16 xl[3], i16 gy[3], i16 hg[3], u8 flags), `02 MAG` (20 B: u32 t_us, f32 mag_uT[3], f32 temp),
  `03 TEXT` (ESP_LOG output, re-routed via esp_log_set_vprintf), `04 INFO` 1 Hz (27 B: proto, odr, xl mg/LSB,
  gy mdps/LSB, hg mg/LSB, frames_dropped, imu_frames, uptime_ms). The host takes its scale factors from INFO.
  A writer task (core 0, prio 4) drains a 16 KB NOSPLIT ring buffer, and producers never block the IMU bridge.
* **Build environment on this PC** (found in `C:\Users\Penelope\.espressif\esp_idf.json`):
  selected IDF **v5.5.2 at `C:\Users\Penelope\esp\v5.5.2\esp-idf`**, python env
  `C:\Users\Penelope\.espressif\python_env\idf5.5_py3.11_env`, git `...\.espressif\tools\idf-git\2.39.2\cmd`.
  Also installed: `C:\esp\v5.4.3` and `C:\esp\v6.0` (**don't use v6.0**: breaking API changes; the project targets 5.5.x).
  The original project was built with 5.5.4 on the user's Mac.
* `firmware/build_flash.bat` sets those paths, runs `export.bat`, `idf.py build`, finds the VID_303A port
  via PowerShell and runs `idf.py -p COMx flash`, logging to `firmware/build_flash_log.txt`.
  **Result 22:06: BUILD_OK (no warnings), flashed to COM3, all hashes verified.**
  `firmware/build/` is the Windows build output. The component manager rewrote `firmware/dependencies.lock`
  to idf 5.5.2, which differs from the repo's 5.5.4 lock; don't commit that lock change upstream.
* The `.vscode/` folder of the firmware was NOT copied (remote tooling refused to write .vscode). Harmless.
* Hardware result (first run, 22:23, board still on the desk): INFO received, 230.8 Hz,
  **seq gaps 0, CRC errors 0**, accel |a| = 0.9964 g, noise 1–2 mg, gyro bias ≈ (-0.40, -0.16, -0.44) °/s,
  mag ≈ (6, -34, -23) µT. Board self-report: `tx 230.x Hz, fifo overruns 0, isr->uart ~0.2 ms`.

---------------------------------------------------------------------------------------------
## 5. Design notes you need before changing code

### 5.1 Kinematics (spm/kinematics.py)
Base frame B: origin = head centre, **z = common ring axis pointing base → head**, x = θ=0 direction.
`w_i(θ) = [sinα1 cosθ, sinα1 sinθ, −cosα1]`, `v_i = R·[sinβ cosη, sinβ sinη, cosβ]`, constraint `w·v = cosα2`
→ `θ = atan2(B,A) + branch·acos(C/√(A²+B²))`. FK is Newton iteration from the last pose (picks the continuous
assembly). Pure twist about z moves all rings equally (unlimited twist). Kinematic tilt limit with α1=50 is
~50–61° (40–48° with α1=40), but collisions are not modelled.
Home ring angles with the default config: **[90, 210, −30]°**.

### 5.2 Hip angles (spm/anatomy.py)
ISB (Wu et al. 2002) pelvis frame, **X anterior, Y superior, Z right**. `R_anat = Rz(flex)·Rx(add)·Ry(introt)`, with
add/rot sign-flipped for the left leg. Effector: `R_BE = R_PBᵀ·R_anat·R_PB·R_neutral`.
Femoral neck offset does **not** enter IK (rotation is about the head centre); it only affects drawing.

### 5.3 Controller loop and safety (spm/controller.py)
* Rate limit (`control.max_rate_deg_s`, default 90) on every target, so all transitions are smooth.
* Safety gate refuses a pose and holds the last safe one if any of these fail: unreachable, leg margin, parallel
  and serial singularity indices, tilt, ring separation, ring excursion.
* Motor counts: `home_counts + dir·gear·(θ−θ_home)·16384/2π`. **Home is software**: encoder counts read at
  "Set home here". It must be redone after every power cycle (the multi-turn count isn't retained, and 1 motor
  turn = 90° of ring).
* Speed per F5 = needed RPM × `speed_headroom` (1.3), clamped to `min/max_motor_rpm`; `acc` = 250.
* **E-stop latch**: E-stop broadcasts F7 ×2 and latches. ARM is refused until **Reset E-stop**, which:
  checks comms; checks stalls (0x3E); clears home if a driver dropped off the bus (3 missed reads, i.e. a
  possible power loss) or the measured pose is implausible; then sets every mode's target to the current pose
  (`_hold_here`), so nothing moves. While latched, the command tracks the measured pose (no 3D drift).
  A driver going silent while armed triggers an automatic latched E-stop.
* ARM snaps the command to the encoder-FK pose and runs `_hold_here()`, so arming never moves the leg.
* Mode time only advances while the command is tracking (sync threshold), so timed modes never start "behind".

### 5.4 IMU pipeline (spm/imu_board.py, imu_control.py)
* Frames: E (effector), I (IMU), B (base). `R_BI = R_BE·R_EI`.
* **Timing:** `SampleClock` (imu_board.py) fits `t_seen = a + T·seq` through the lower envelope (quickest
  arrival per 0.25 s bucket) of PC arrival times over a 60 s window, with one-sided outlier rejection.
  It converges after 2 s of stream (`converged`) and falls back to the nominal period before that.
  Corrected time is accumulated as `t += Δseq·T`. The estimator, gyro integrator and every `sample_listener`
  get this `t`. `board.t_board` (firmware t_us) is kept for diagnostics only. It resets on port open and
  when seq goes backwards (board reboot). Snapshot fields: `true_rate_hz`, `rate_converged`, `rate_fit_rms_ms`.
* Estimator: gyro integration + Mahony-style accel tilt correction (kp 0.4, gated at ±0.08 g) + gyro bias
  learning at standstill (requires the controller's "commanded still" hint). **Tilt is absolute; heading
  (rotation about gravity) is gyro-only and drifts.** Use "Align heading" to re-reference it to the encoder FK.
  The magnetometer is deliberately unused (3 stepper motors a few cm away).
* **Mount calibration** (GUI IMU tab or `run_trial.py`; rig must be homed and ARMED): neutral hold 2 s
  (gravity + bias), then ±15° about effector x, ±15° about y, ±25° about z, each out and back
  (12 moves, ~18 s). Each move pairs the encoder-FK rotation (effector axes) with the gyro-integrated rotation
  (IMU axes). R_EI comes from SVD/Kabsch, which needs no clock sync. Gravity in base = `R_BE_home·R_EI·â`. Saved to `imu_cal.json`.
  **Residuals** (typically < 0.3° when clean) are the model-validation signal: compliance/backlash/geometry
  error shows up as larger residuals. `gyro_scale` (|dI|/|dE|) should be ≈ 1 (≈ 0.96 would mean the
  §6.1 rate correction is broken). Calibration waits up to 10 s for `clock.converged` before the first move.
* Opening the port flushes 0.3 s of stale data. The board keeps queuing frames while no program reads, and the
  first run after boot captured the IMU's power-up settling transient (±155 °/s garbage) plus an 80 s timestamp jump.
* **Tilt trim** (`imu.trim_gain`, IMU tab slider, default 0 = monitor only): integrates IMU-measured **tilt**
  error into a ≤ `trim_limit_deg` (3°) correction applied to the command, safety-checked. Heading is excluded.
* Snapshot gives: IMU hip angles, `w_pelvis_dps` (Z = flexion axis), err IMU-vs-cmd, IMU-vs-encoder, tilt-only.

---------------------------------------------------------------------------------------------
## 6. Known problems / caveats (ordered by importance)

1. **IMU timestamp scale error (~4%): FIXED on the host (2026-11-17 22:45). Kept here for context.**
   The firmware's `bridge.c` builds `t_us` from a free-running counter advancing `1e6/IMU_LINK_RATE_HZ`
   (=4166 µs) per sample, but the LSM6DSV80X really delivers **230.35 Hz** on this board. It also re-anchors
   that clock whenever it falls more than 250 ms behind, which produces ~254 ms jumps about every 6 s with
   zero seq gaps. Fix: option (a), host-side `SampleClock` (§5.4). Real-board result, `imu_check.py --seconds 30`:
   host estimate **230.352 Hz**; independent cross-check from the board's own esp_timer (INFO uptime vs seq)
   **230.350 Hz** (0.001% apart); 5 re-anchor jumps in t_us, none in corrected time; envelope rms 1.05 ms.
   Tests: `test_sample_clock_recovers_true_rate_and_angle` (230.5 Hz labelled 240, bursty USB, stalls → rate
   within 0.05 Hz and integrated angle within 0.2%; it also asserts that the old t_us timing reads > 2.5% short),
   `test_sample_clock_survives_seq_gap_and_reboot`. `imu_sim.py` now emulates the real timing fault
   (`true_hz=230.5`, nominal stamps, re-anchoring), and the end-to-end test asserts calibration `gyro_scale`
   is within 2% of 1 (got 1.0015, sim rate 230.49 Hz).
   Still open: (b) firmware-side real timestamps. The LSM6DSV80X driver exposes the factory ODR trim
   (`lsm6dsv80x_odr_cal_reg_get`, register INTERNAL_FREQ_FINE); check its scaling in the datasheet
   before relying on it. **The UART link to Avionics C has the same nominal-240 Hz assumption.** That link is
   not ours to change, but tell the user (their flight computer integrates ~4% short as well).
2. **Never tested with real motors.** Expect: motor direction signs, which motor drives which leg (address ↔
   leg mapping), the IK `branch` per leg, and `eta_deg` to need correcting. Use the GUI **Joint jog** with
   ±5° first (README "Commissioning checklist").
3. **Driver setup has not been done.** New drivers are all addr 1 @ 38400 in pulse mode. Configure ONE at a time
   (Driver setup tab: Configure → Set address 1/2/3 → optionally Set baud 115200 and reconnect).
   `config.json` `serial.baud` is 38400 and `serial.port` is "COM3" from the very first config. **COM3 is the IMU
   board, NOT the RS485 adapter.** Don't connect the motor bus to COM3.
4. **Encoder feedback lags** (round-robin, each motor read every 60 ms). IMU-vs-encoder errors during fast
   motion are inflated (sim gait trial: rms 2°, max 6°, mostly lag). Judge accuracy at holds, or add
   time-alignment in run_trial.
5. **Board-side "dropped" counter** in INFO counts frames dropped while no program has the port open. It
   grows into the hundreds of thousands and is harmless. Host `seq_gaps` is the meaningful drop metric.
6. **Heading drift** (rotation about gravity) is unobservable without the magnetometer. Re-align at holds.
   Simulated drift was ~0.8°/min with a 0.6°/s bias. Real bias ≈ 0.4°/s; bias learning needs still periods.
7. **The IMU sits on the femur at a lever arm** from the head centre: centripetal/tangential acceleration
   (~0.1–0.2 g at gait speeds) slightly biases accel tilt correction. Gating plus a low kp mitigate it; a
   lever-arm estimate (`r_I` least squares) was designed but not implemented.
8. The board still runs its flight datalogger (NOR logging "armed", SD absent → errors in the log). Harmless,
   but every boot dumps/erases NOR. Leave it unless the user wants it disabled on this branch.
9. GUI: the IMU-trim slider and max-rate slider aren't persisted unless "Apply & save" is used in Geometry;
   config changes from sliders are in-memory only.
10. `run_trial.py` **sets home at its start**, so the leg MUST be at anatomical neutral when it is launched.
11. `still (bias learning)` in the IMU tab flickers "no" because the window resets after each bias update. Cosmetic.
13. **Collision limits from the CAD (done 23:30), and the ROM they leave is short of gait.**
    `python experiments/cad_collision.py "bone-in SPM.step" limits` (~7 min; `baseline` for contacts at the CAD pose).
    No CAD kernel: gmsh/OCP were NOT installable (pip install was blocked by the permission system), so the script samples
    surfaces itself (planes/cylinders/cones/tori/spheres trimmed by their boundary loops; rational B-spline patches
    trimmed in (u, v)). It is cached in `experiments/cad_cache/`. Two STEP quirks were fixed: a 0.01 mm arc of a 381 mm ellipse
    (rounding gave a full ellipse), and trimmed bone patches.
    Motion groups: fixed (casing, pinions); leg i = ring i + arm i (rotate about z); middle arm i (frame from (w_i, v_i));
    femur = head + upper/lower femur + positioning key (the key joins the two femur halves).
    Pairs are sorted by relative motion. Rotation about z gives a 1-D table vs relative ring angle (arms touch at 43.5°,
    the same for all pairs; the arm also hits the next ring's arm bosses at the same angle). Gear meshes, bearings, stacked
    rings and arms vs casing are running contacts (min > 0.05 mm over a full turn). The arm↔middle arm pair rotates about w:
    the elbow interval. The middle arm↔femur pair rotates about v: the wrist interval (the middle arm hits the upper femur's
    neck). Everything else ("general", 67 pairs) was swept over all 1652 allowed poses of a 5°/10° grid: closest 4.81 mm
    (middle arm 3 vs middle arm 1).
    Runtime: `CoaxialSPM.joint_fold_angles()` gives elbow/wrist angles relative to neutral (= the CAD pose). `check_pose`
    refuses poses outside `joint_limits` (minus `margin_deg` 3) and reports `joint_room_deg` (GUI "joint fold room", 33° at
    neutral). Tests: `test_joint_fold_angles_match_link_rotations`, `test_safety_gate_refuses_elbow_fold_collision`.
    **Consequence:** flexion is limited at +26° by arms 1/2 (or 2/3) meeting, adduction at +17°, and internal rotation at
    +25° by leg 2's elbow. The built-in gait (flex 31.7°) is refused for 43% of its cycle; Gait mode then holds at the
    last safe pose. Possible responses (user's call): a different mounting of the base / head on the femur (neutral further
    from the arm-contact direction), narrower arms, or a scaled-down gait. A remount study is a natural next task: vary
    mount/neutral, then rerun the ROM / gait check, which takes seconds once the joint limits are known. Limits are only valid
    for this CAD + this neutral: changing `neutral.*` or `geometry` invalidates them (re-run the script).
14. **Remount study** (`experiments/remount_study.py` scan ~25 s, `experiments/remount_verify.py` per-candidate details;
    results in `experiments/results/remount_*`). Key reduction: R_BE = R_n · A(t), with A(t) = F_E R_anat F_Eᵀ the hip motion seen
    from the head (fixed by hardware) and R_n = neutral. Spinning the SPM about its ring axis changes nothing, so the design
    space is R_n = Ry(tilt_y) Rx(tilt_x) Rz(twist). Score = largest gait scale passing the full gate (`FastGate`, a vectorised
    `check_pose` that agreed with the controller on 3000/3000 random poses).
    | option | neutral tx/ty/tw | mount down/fwd/twist | gait fits | worst w/ build err | ROM flex / add / rot |
    |---|---|---|---|---|---|
    | today | 15.4/−9.1/0 | 24.6/−20.1/91.5 | ×0.55 | ×0.20 | −84…+26 / −34…+17 / −49…+25 |
    | symmetric home | 0/0/0 | 34.4/−2.8/94.2 | ×1.30 | ×1.00 | −54…+58 / −31…+29 / −36…+38 |
    | **A (recommended)** | −10/5/0 | 38.3/10.2/93.5 | ×1.75 | ×1.45 | −32…+72 / −30…+35 / −22…+50 |
    | B (smaller change) | −5/5/0 | 39.0/3.9/93.9 | ×1.60 | ×1.30 | −40…+67 / −28…+34 / −29…+43 |
    | peak (fragile) | −15/5/20 | 42.5/14.6/72.9 | ×1.90 | ×0.95 | −26…+71 / −27…+40 / −18…+52 |
    CAD clearances of the pose-dependent pairs along the gait stay ≥ 9 mm for A/B (≥ 5 mm for the fragile one). Option A checked
    through the REAL controller: 200/200 gait samples pass at ×1.0 and ×1.4 (today: 114/200).
    Two ways to realise a new R_n: (1) re-angle the casing on the pelvis fixture (new `mount` + `neutral` values, CAD limits
    unchanged, nothing to recompute), or (2) keep the casing and re-angle the femur-head part on the femur by the same rotation
    (A: 29.0° about pelvis axis (−0.42, −0.72, −0.54)). Option (2) changes the CAD (femur relative to head), so re-export the STEP
    and re-run `cad_collision.py limits` (the wrist limits involve the upper femur).
    **Arm-arm contact angle needed for a full gait** (min ring separation along the cycle, other checks passing):
    today 0.2° (impossible: the rings would have to pass through each other); symmetric home 69.6° (47.4° worst case with
    build error); A 78.5° (60.0° worst case; 71.7° at ×1.3 gait); B 88.4° (69.6° worst case). Arms touch at 43.5° now.
    Suggested margins: ~2° ring angle for control/backlash (0.6 mm at the arm), plus design for ×1.2–1.3 gait amplitude and
    ±5° mount error. With A, the arm contact angle must stay ≤ ~58° (60.0 − 2); the current 43.5° leaves ~14° spare.
    `joint_limits.ref_R_BE` (CAD pose) now decouples the fold limits from neutral, so a remount only needs `mount` + `neutral`
    changes (`test_joint_limits_follow_cad_pose_not_neutral`).
15. **Motor torque is short for an 8 kg leg** (`experiments/torque_estimate.py`, 2026-11-18). Rigid straight leg, Winter
    segment table → CoM 0.386 m from the hip, I_hip 1.87 kg·m². τ_spm = Iα + ω×Iω − r×mg; ring torques τ_ring = M⁻ᵀτ
    (M = Jb⁻¹Ja; checked by virtual work to 6e-5). Built-in gait on mount A, timed so the fastest ring hits 90°/s (cycle
    2.94 s, motor 60 rpm): ring peak 11.7 N·m → motor **3.26 N·m** (4:1, η 0.9). Gravity alone gives 2.6 N·m and
    inertia alone 1.7 N·m. Static holding worst case over all allowed poses: motor 5.24 N·m. The user's motor is rated
    **1.8 N·m stall** (user-stated; not found in MKS docs online), i.e. 6.5 N·m per ring. For 8 kg at ×1.5 margin, a gear
    ratio of about 11:1 is needed (or a max leg mass of about 2.9 kg at 4:1, or gravity compensation, since gravity is the
    larger part). Rate sensitivity: 45°/s → 2.68, 120°/s → 3.87 N·m.
17. **Speed derating checked against a motor curve (2026-11-18).** The user supplied the pull-out curve of a SIMILAR-size
    57 mm stepper (DM556T open-loop, 24 V, 2.69 A; not the SERVO57E's own curve). It is stored in
    `experiments/motor_curve_57_similar.csv`: 1.70 N·m at 20 rpm, 1.87 at 50, 1.47 at 100, then a fall to 0.76 at 150 and
    0.30 at 400 rpm. `torque_estimate.curve_margin()` compares needed vs available torque per motor at its own speed at every
    gait instant. Finding: the peak load is at heel strike (maximum flexion, nearly zero velocity: motor about 6 rpm), so the
    binding figure is the low-speed torque (about 1.7 N·m), not the derating. Worst margin (mount A): 4 kg & 6:1 → ×1.59 at
    90°/s ring and ×1.82 at 60°/s; 8 kg needs 8:1 for ×1.06 at 90°/s, or 10:1 for ×1.5 up to 60°/s. Above about 100 motor rpm
    the curve collapses, so keep ring rate × gear / 6 ≲ 100 rpm.
    **48 V supply (user plans it):** the SERVO57E manual (V1.0.1, now in this folder as
    `MKS SERVO42&57E_RS485 User Manual V1.0.1.pdf`) gives **operating voltage 20–60 V** (100 V/25 A MOSFETs; LED fault codes for
    supply voltage high/low), so 48 V is in spec, with 12 V headroom for regeneration when the leg's weight back-drives the
    motors. Working current (0x83): 57E 0–5200 mA, **default 3200 mA**. Set it to the motor's rated phase current (the user's
    motor rating is not known yet); torque scales with it. The comparison curve was measured at 2.69 A peak. There is no
    measured 48 V curve. A first-order model (same current-limited low-speed torque, knee speed ×2) changes NO margin in the
    operating range, because the worst instant is always at < 20 rpm (heel strike). It helps only beyond about 150°/s ring rate.
    (The manual's text was extracted with a small zlib/regex script; pdftoppm and PDF libraries are not installed.)
21. **Redesign v2 analysed AND ACTIVE (2026-11-18): `bone-in SPM 2.step` → `config_v2.json`, copied to config.json on the user's go-ahead.** v1 backups: `config.v1_before_v2_2026-11-18.json`, test fixture `tests/config_v1_cad.json` (tests that check v1 CAD facts use it). The IMU mount calibration now picks the largest SAFE swing per axis/sign (`_safe_swing`, min 4°); v2 sim used x +15/−10, y +15/−7, z ±25.
    New tools: `experiments/cad_to_config.py` (STEP → geometry/mount/neutral/gear; reproduces v1 exactly),
    `experiments/rom_check.py` (ROM / gait / torque with the real gate), `cad_collision.py --config X --write`.
    rig_layout now finds rings / middle links by geometry (v2 links are "Part 11/12/13"); the colour reader handles
    OVER_RIDING_STYLED_ITEM (how Onshape writes a face colour). v2: α1 45, α2 90, β 90, η [144.4, 24.4, −95.6], branch −1,
    108T internal rings / 18T pinions = 6:1, centre residual 0.0000 mm. Orange casing face = ground: ring axis 38.0° down,
    8.3° forward (≈ option A). **The v2 CAD leg is NOT standing**: flex −17.7, add +17.1, rot +5.4 relative to standing,
    so neutral = femur vertical (from the ground face), knee axis horizontal; neutral is ~9° from the symmetric home.
    Collision limits (v2): arm contact 16.8° (legs 1-2) and 8.9° (2-3, 1-3), gate uses 18.3; elbow leg 1 −16, leg 2 −53,
    leg 3 free; wrist leg 3 +78; middle links ≥ 8.8 mm from everything along the gait (1.31 mm only at flex 85/add 25/rot −40).
    ROM from neutral: flex −11.5…+41, add −14.5…+48, rot −7.5…+27.5; gait ×0.85, 8% refused, all by the LEG 1 ELBOW, and
    only because of the 3° margin: physically the gait folds it to −14.2° vs −16.0° contact. Needs +1.2° fold clearance for
    the full gait with margin, about +4° (leg 1) and +3° (leg 2) for ×1.3. The contact is arm↔middle link 13–16 mm from the
    elbow axis, 102–104 mm from the centre (1° ≈ 0.25 mm of material), the same spot on legs 1 and 2; arm3 never collides.
    Torque (α1 45 costs torque): 4 kg ×1.36 margin at 90°/s, 6:1.
20. **alpha1 study** (`experiments/alpha1_study.py`, 2026-11-18; mount A, α2/β/η as built; CAD elbow/wrist limits OFF,
    45° arm contact assumed unchanged). Going from α1 30 → 40 → 50 → 60: usable flex+ 59/72/82/93°, flex− (extension) −19/−32/
    −48/−73°, gait headroom ×1.40/1.75/2.10/2.45, neutral ring separation 90/99/105/110°. The cost: quasi-static motor torque
    (4 kg, 6:1) 0.70/0.87/1.11/1.52 N·m, because the rings turn less per degree of hip motion (gait cycle at 90°/s ring
    3.66/2.94/2.50/2.18 s), and the parallel index at neutral 0.95/0.97/0.81/0.55. 40–45° is the balanced range. Any change
    needs new arm CAD and a re-run of cad_collision.py.
19. **Geometry-tab remount (2026-11-18).** Bug report: "the override stops working after changing the mount tilt". Cause:
    the override was fine, but a mount-ONLY change re-bolts the femur on the head, and flexion REACH (any IK solution) fell
    from +54° to +33°, so poses beyond that cannot be drawn. Fix: `update_config(..., keep_femur=True)` (GUI checkbox
    "Remount casing", default on) recomputes neutral = R_PB_newᵀ·F_Eᵀ when only the mount changed (option A's mount →
    neutral −9.98/+5.02/0.04 ✓). `safety.unreachable` → GUI badge "OUT OF REACH … cannot be drawn even with the override".
    Also fixed: Geometry forms did not refresh after Apply (the focused Apply button counted as "typing"), so stale
    neutral values would have been sent back. `test_remount_keeps_femur_on_head`.
18. **Gear strength check, PETG herringbone 18:108, module 1, 12 mm face (2026-11-18).** The user's helix angle is **25°**
    (first computed at 30°). With transverse module 1 the result is unchanged within 1%: pinion 29.7 MPa for 4 kg gait, 47.4 for
    the worst pose, 56.5 for a jam. If the module is normal, stresses are about 9% higher than at 30°. Figures below are at 30°. Lewis bending,
    with Y from the virtual tooth count (pinion z_v 27.7 → Y 0.352; ring z_v 166 → Y 0.461), full load on one tooth.
    The current CAD has centre distance 45 = (18+72)/2 × 1, so "module 1" there is TRANSVERSE → normal module 0.866 at 30°.
    Pinion root stress (the weak member; it also sees 6× the load cycles of a ring tooth): 4 kg gait peak 30 MPa;
    4 kg worst holding pose 48; 8 kg gait 59; 8 kg worst pose 96; jam at 1.87 N·m motor torque 57 MPa. With normal module 1
    it is about 25% lower. Prusament PETG printed yield is 47 ± 2 MPa (horizontal; TDS v1.1). Verdict: OK only for 4 kg gait,
    with a ×1.6 margin to yield (not fatigue-rated); fails for 8 kg, for the extreme holding poses and for a jam. Remedies:
    wider face / bigger module (stress ∝ 1/(b·m_n)), a metal or nylon pinion, capping the driver current (0x83) to limit jam
    torque, keeping the ROM limits tight, printing teeth flat in XY.
16. **Ground face by colour (planned by the user, 2026-11-18):** the user will colour ONE planar face differently from its
    part to mark the face that must be parallel to the ground. `python experiments/cad_axes.py <step> --colours` lists part
    colours and marked faces (part, surface type, centroid, outward normal). `cad_axes.ground_up(E, parts, head_to_knee)`
    returns 'up', choosing the sign so the femur hangs down. Validated with an in-memory mark on the casing's bottom face
    (outward normals correct, sign rule works). The current file has part colours only (17 solids, 6 colours, no face
    colours). Once a marked file arrives: (a) confirm the export kept the face colour; (b) use 'up' instead of the femur
    mechanical axis for the pelvis Y axis / mount angles (§2.1), as gravity in torque_estimate.py, and as a cross-check
    of the IMU calibration's gravity_in_base. Note: the CAD pose would then not be exactly "femur vertical" (anatomically
    the mechanical axis is ~3 deg off vertical), so neutral.* and mount.* get re-derived.
12. Gait built-in curve = approximate textbook shape (flex ~30°/ext ~10°, add ~10°/abd ~5°, small rotation). Use real CSV data for quantitative work.

---------------------------------------------------------------------------------------------
## 7. Verified vs not verified

| Item | Status |
|---|---|
| MKS protocol frames/checksums incl. long packet 0x11 example | ✅ unit tests vs manual |
| IK/FK round trip (1e-8°), workspace estimate | ✅ sim |
| Controller, safety, E-stop latch/recovery (stall, power-loss) | ✅ sim + tests |
| GUI (pose/plane/gait/joint/geometry/setup/IMU tabs) | ✅ headless Chrome, sim |
| IMU firmware builds with IDF 5.5.2, flashes, streams | ✅ **real hardware** (COM3) |
| USB stream integrity (CRC 0, gaps 0 after flush fix) | ✅ real hardware |
| IMU estimator + mount calibration + trim | ✅ sim only (`imu_sim.py`) |
| Real mount calibration / real trial | ❌ needs motors + RS485 adapter |
| Real SPM geometry α1/α2/β/η/branch | ✅ from CAD, IK reproduces the CAD pose to 0.003° |
| Mount + neutral | ⚠️ from CAD, assuming the CAD pose = standing neutral; knee ML axis ±~5° |
| Collision limits | ✅ from CAD (ring sep 43.5°+1.5°, elbow/wrist intervals; others ≥ 4.8 mm). ⚠️ ROM < gait flexion |
| Timestamp/ODR mismatch | ✅ fixed host-side, verified on real board (230.35 Hz, cross-checked) |

---------------------------------------------------------------------------------------------
## 8. Next steps (recommended order)

1. ~~Fix §6.1~~ **done 22:45**: SampleClock, 2 new tests, sim emulates the fault, real board verified.
   GUI checked in the browser pane with the SIM board: the IMU tab row "true sample rate" shows a red
   "measuring…" for about 2 s, then settles at 230.50 Hz. The sim used to drift to ~229 Hz because its
   thread dropped time when late on Windows; it now catches up in bursts like the real FIFO.
   (`.claude/launch.json` "spm-gui" runs `server.py --no-browser` for the browser pane.)
2. ~~Ask the user for the femoral-head joint directions~~ **done from the CAD (23:00)**, collision limits **done
   (23:30)**. Remaining: the ROM shortfall (§6.13). Original item: the **femoral-head joint directions** (β, η per leg), α1 (40 or 50), the real mount angles,
   and collision limits. Update `config.json` (Geometry tab or by hand) and check the 3D view against the CAD.
3. With the user present: plug in the USB-RS485 adapter, find its COM port, and do driver setup one driver at a time.
4. Commission: Torque OFF → place at neutral → Set home → ARM → Joint jog ±5° per ring (fix direction/address/branch).
5. Connect the IMU (IMU tab) → **Calibrate mount**. Check residuals and gyro_scale (≈1 after step 1).
6. Run `python experiments/run_trial.py --port COMx --profile step` then `--profile plane` / `gait` (start with
   low `--rate`, e.g. 30) to quantify model error. Then try `--trim 1.0` and compare.
7. Optional improvements: time-aligned encoder vs IMU comparison; lever-arm compensation; firmware real timestamps;
   persistent IMU UI settings; `git init` spmcontroller with experiments on a testing branch (ask first).

---------------------------------------------------------------------------------------------
## 9. Likely mistakes the next session will make (avoid these)

* **Connecting the motor bus to COM3.** COM3 is the IMU board (VID 303A). The RS485 adapter will be another COM port.
* **Using `board.t_board` / firmware `t_us` for dt anywhere.** It is the nominal 240 Hz clock with ~254 ms jumps.
  Use the `t` that sample_listeners receive (or `board.t`), which comes from SampleClock.
* **Using ESP-IDF v6.0 or v5.4.3.** Use `C:\Users\Penelope\esp\v5.5.2\esp-idf` (or just run `firmware\build_flash.bat`).
* **Opening the board's serial port with DTR/RTS asserted** (pyserial default). That can reset the ESP32-S3 or put it
  in download mode. `ImuBoard.open()` sets `dtr=rts=False` before `open()`. Keep it that way in any new tool.
* **Forgetting that stale frames arrive on port open.** Always flush first, like `ImuBoard.open()`.
* **Committing firmware changes to the flight branch / main**, or modifying the user's real firmware repo without asking.
  Keep `spm-imu-usb-stream` separate. The UART protocol to Avionics C must stay byte-identical.
* **Re-guessing geometry.** It is measured from the CAD now (§2.1). If the CAD changes, re-run
  `python experiments/cad_axes.py "bone-in SPM.step"`. The derivation of the config values (centre, branches, mount) was
  done in scratch scripts; its method is described in §2.1, and `test_config_reproduces_cad_pose` guards the result.
* **Changing `neutral.*`, `mount.*` or `geometry.*` without re-running `cad_collision.py limits`.** The joint limits are
  angles relative to neutral = the CAD pose.
* **Forgetting that neutral is 17.9° from the symmetric home**, so tilt ≠ 0 at neutral, and `R_neutral` ≠ I.
* **Moving the rig without the user's go-ahead**, or running `run_trial.py` when the leg isn't at neutral (it sets home).
* **Putting characterisation/recording code into `spm/`.** It belongs in `experiments/` (or a testing branch).
* **Feeding IMU heading back into control.** Only tilt is absolute; heading drifts.
* **Treating IMU-vs-encoder spikes during fast moves as model error.** That's encoder readback lag (§6.4).
* **Removing comments / writing sparse code.** The user wants heavy explanatory comments.
* **Breaking `python tests/test_all.py`.** Run it after every change (~2 min; it includes simulated-IMU end-to-end tests).

---------------------------------------------------------------------------------------------
## 10. How to run things (locally in Claude Code you can just use the shell)

```bat
cd "C:\Users\Penelope\Documents\y4t1\Capstone 1\spmcontroller"
python -m pip install -r requirements.txt
python tests\test_all.py                         :: all should pass (32)
python server.py                                 :: GUI at http://localhost:8765 (Simulate box = no hardware)
python experiments\imu_check.py                  :: 8 s IMU smoke test -> experiments\imu_check_log.txt
python -m spm.imu_board --raw                    :: live raw IMU from the board (COM auto-detect)
python experiments\run_trial.py --sim            :: full simulated closed-loop trial
firmware\build_flash.bat                         :: rebuild + flash the IMU board (IDF 5.5.2), log in firmware\build_flash_log.txt
```
In the GUI, use `SIM` as the IMU port for the simulated board, and tick "Simulate drivers" for simulated motors.

### Context from the previous (cloud) session's tooling, in case it matters
* The previous session ran in a cloud sandbox. It wrote files into this folder remotely and could only start
  programs by double-clicking `.bat` files in File Explorer. Terminals were typing-blocked for it, which is why
  the `.bat` launchers and `*_log.txt` files exist. Claude Code locally doesn't need that indirection.
* Earlier work in this project (same session): the SPM GUI app, then E-stop recovery, then the IMU firmware
  and host pipeline. The README documents user-facing behaviour for all three.

### Sources relied on
* MKS SERVO42E/57E RS485 User Manual V1.0.1 (user upload): protocol details in §2.2.
* Wu et al. 2002, ISB recommendation (hip JCS); ISO 13850 (E-stop reset must not restart the machine).
* Normal-gait hip ranges (~30° flex / ~10° ext; ~10° add in loading, ~5° abd in swing; small rotation).
* Tursynbek & Shintemirov, coaxial SPM kinematics papers (background for the coaxial IK).
