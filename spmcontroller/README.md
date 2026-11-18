# SPM Hip Controller

Browser GUI for driving a **coaxial 3-RRR spherical parallel manipulator** used as the hip joint
of a leg phantom. The manipulator is driven by three **MKS SERVO57E RS485** closed-loop steppers,
each geared **4:1** to its internal ring.

* **Pose**: set clinical hip angles (flexion, adduction, internal rotation; ISB convention).
* **Plane**: swing the femur in **any plane** through the hip centre: sagittal, frontal, transverse
  or any oblique plane, either static or oscillating.
* **Gait**: replay a walking hip cycle, either the built-in approximate curve or your own CSV.
* **Joint jog**: move individual rings, for commissioning.
* 3D preview of the linkage and femur (orange = commanded, blue = measured from encoders).
* A **simulation mode** to try everything without hardware.
* A driver setup tab for the out-of-box address, mode and baud configuration.

## Quick start (Windows)

1. Install Python 3.9 or newer from python.org and tick *"Add python to PATH"*.
2. Double-click `run_windows.bat`. This installs `numpy aiohttp pyserial` and opens
   `http://localhost:8765`.
   Alternatively, run `pip install -r requirements.txt` then `python server.py`.
3. With **Simulate drivers** ticked, click **Connect**, then **Set home here**, then **ARM**. Then use the Motion tab.

The server only listens on `127.0.0.1`, so other machines on the network can't move the rig.

## First-time driver setup (one driver at a time)

New MKS drivers are all **address 1, 38400 baud, pulse mode**. Because they share an address,
configure them one at a time:

1. Wire **one** driver to the USB-RS485 adapter (A to A, B to B, common GND). Untick *Simulate*
   and connect at 38400.
2. Go to **Driver setup** and click **Scan** to confirm the driver answers.
3. Click **Configure**. This sets work mode `05` (RS485 bus, closed loop), 16 microsteps (so the
   F5 speed field is true RPM), respond on and active off (`8C 01 00`, so there are no
   unsolicited "move complete" frames on the bus), axis lock and stall protection.
4. **Set address** to 1, 2 or 3. Repeat for each driver.
5. (Recommended) **Set baud** to 115200 on each driver, then reconnect at 115200. One 52-byte
   long packet takes ~4.5 ms at 115200 versus ~13.5 ms at 38400.
6. Put all three on the bus, reconnect, and **Scan** to check all three answer.

## Every power-up

1. **Connect**.
2. **Torque OFF**, then place the leg in **anatomical neutral** (a jig helps).
   The 3D view at 0/0/0 shows what neutral should look like.
3. **Set home here**. Home is stored in software as the encoder counts at this pose. It must be
   redone after each power cycle, because the multi-turn count is not retained and 1 motor turn
   = 90° of ring.
4. **ARM**. The command first snaps to the measured pose, so the leg won't jump.
5. **Esc** or the red button sends a broadcast **F7 emergency stop** and disarms.

## Recovering from an E-stop

The E-stop (red button or **Esc**) broadcasts F7 to all drivers and **latches**. Arming is blocked
until you reset it deliberately, and resetting never moves the leg by itself. This follows the usual
machinery rule that resetting an E-stop only permits a restart.

1. **Find and remove the cause** (jam, collision, wrong pose, loose part). While latched, the
   3D view shows the leg where it actually stopped.
2. If the log reports **stall protection** (the driver released its shaft after a locked rotor),
   click **Release stalls** (0x3D to every motor).
3. Click **Reset E-stop** (header or Connect tab). The server checks that:
   * every driver still answers;
   * no driver is in stall protection;
   * no driver dropped off the bus during the stop (3+ missed encoder reads = possible power loss,
     after which the multi-turn count is no longer trustworthy);
   * the measured pose is still inside the safety limits.

   It logs how far each ring drifted since the stop, then sets **every mode's target to the
   current pose**. The Pose sliders and joint jog sliders update to match.
4. If it says **home was cleared**, re-home: Torque OFF, place at neutral, Set home here.
5. **ARM**. The leg holds still. Press Run or move a slider to resume.

A driver that stops answering **while armed** triggers the same latched E-stop automatically.

### Commissioning checklist

* In **Joint jog**, move ring 1 by +5°. The physical ring 1 should turn the same way as the 3D
  view. If it doesn't, set that motor's **direction** to -1 (Geometry tab). Do the same for rings 2 and 3.
* Check that motor *n* actually drives leg *n*, i.e. the leg whose head joint is at η*n*. Swap
  the **motor addresses** in the Geometry table if not.
* Compare the 3D home pose with the CAD. If a leg's elbow points the other way, flip its **elbow branch**.
* Tune **min ring separation** and **max tilt** (Safety limits) to where the links start to touch.
  Link collisions are not otherwise modelled.

## How it works

```
browser (static/)  <--WebSocket JSON-->  server.py  -->  spm/controller.py (50 Hz thread)
                                                            |-- spm/motion.py      target pose vs time
                                                            |-- spm/anatomy.py     hip angles <-> effector
                                                            |-- spm/kinematics.py  IK / FK / singularities
                                                            |-- spm/bus.py         RS485 or simulator
                                                            '-- spm/protocol.py    MKS frames + checksum
```

Each control tick: mode target → **rate limit** (max °/s) → **IK** → **safety gate**
(reachability, singularity indices, ring separation, tilt, excursion; a failing pose is refused
and the last safe pose held) → ring angle × 4:1 × direction → encoder counts → **one long
packet (0xFC)** carrying three **F5 absolute-axis** moves. All three motors therefore update in the same
frame. F5 accepts new targets while moving (manual §5.9). One motor's encoder is read back per
tick, and forward kinematics gives the measured pose.

### Kinematics (coaxial SPM)

Base frame: z = common ring axis (base → head), origin = femoral head centre. For leg i:

* middle joint axis `w_i = [sin α1 cos θ_i, sin α1 sin θ_i, −cos α1]`
* head joint axis `v_i = R · [sin β cos η_i, sin β sin η_i, cos β]`
* constraint `w_i · v_i = cos α2`, which gives the closed-form IK
  `θ_i = atan2(B, A) ± acos(C / √(A²+B²))`.

FK is solved by Newton iteration from the last pose. The femoral neck offset does **not** enter the
IK, because the manipulator rotates the head about its own centre. It only affects how the femur is drawn.

### Hip angles

Pelvis frame per ISB (Wu et al. 2002): X anterior, Y superior, Z right. Hip rotation is
`Rz(flexion) · Rx(adduction) · Ry(internal rotation)`, mirrored for the left leg.
The **mount** settings say how the SPM base sits on the pelvis. The default is the ring axis pointing
laterally, 45° down and 20° forward, roughly like an acetabulum. Adjust it to match the rig.

## Femur IMU board (closed-loop check)

The Space Bar training board (ESP32-S3 + LSM6DSV80X) is rigidly mounted on the femur standoff and
streams raw IMU samples over its USB port. The firmware is in `firmware/`, on git branch
`spm-imu-usb-stream`. The laptop turns those samples into the femur's **orientation and angular velocity**.

* **Sample rate.** The firmware labels the stream 240 Hz and stamps samples on a nominal 240 Hz clock,
  but the sensor's own oscillator really delivers about **230.35 Hz** on this board (measured
  2026-11-17). Using the nominal clock would make every gyro-integrated angle about 4% too small. The
  laptop therefore measures the true rate from the stream itself (`SampleClock` in `spm/imu_board.py`)
  and times samples by their sequence number × the true period. The IMU tab shows it as
  "true sample rate". It takes about 2 s after connecting, and mount calibration waits for it.

* **IMU tab → Connect IMU.** The board is auto-detected by Espressif's USB ID. Enter `SIM` to use the
  simulated board instead.
* **Calibrate mount** (rig homed and armed). The leg holds still at neutral to measure gravity and gyro
  bias. It then rotates ±15° about the head's x and y axes and ±25° about z. Pairing each move's
  encoder rotation with the gyro-integrated rotation gives the board-to-femur mounting
  (`imu_cal.json`). Redo it only if the board is re-mounted.
  Check the reported **residuals**: a few tenths of a degree is a clean fit, while larger values mean
  the linkage is not doing exactly what the model says (compliance, backlash, geometry error).
  The reported **gyro/encoder scale** should be about 1.00. If it is about 0.96, the sample-rate
  correction is not working.
* **What is measured.** Tilt relative to gravity is absolute and does not drift. Heading (rotation
  about gravity) is gyro-only, so it drifts slowly; gyro bias is re-learned whenever the leg is held
  still, and **Align heading** re-references it to the encoders. The magnetometer is streamed but not
  used, because three stepper motors sit a few cm away.
* **Readouts.** The angle table shows commanded, encoder-FK and IMU values side by side. The IMU tab shows
  IMU-vs-encoder and IMU-vs-command error, with the tilt part on its own. Angular velocity is given in
  pelvis axes (Z = flexion axis). The 3D view draws the IMU femur in green.
* **IMU tilt trim** (optional, 0 = monitor only) closes an outer loop. It integrates the IMU-measured
  tilt error into a correction of up to 3° on the motor command, to remove what the motor encoders
  cannot see. Heading is excluded on purpose, since feeding it back would steer by gyro drift.
* **Encoder lag.** Encoder readback is round-robin (each motor every ~60 ms at 50 Hz), so during fast
  moves IMU-vs-encoder includes that lag. Judge static accuracy at holds.

### Experiments (testing code, kept in `experiments/`)

```
python experiments/imu_check.py                              # 8 s smoke test of the USB stream (+ true rate)
python experiments/imu_check.py --seconds 30                 # longer run = tighter rate estimate
python -m spm.imu_board                                      # live orientation/rates from the board alone
python experiments/run_trial.py --sim                        # whole pipeline, simulated
python experiments/run_trial.py --port COM5 --profile step   # real rig: steps, logs CSV + summary
python experiments/run_trial.py --port COM5 --profile gait --cycles 3 --trim 1.0
```
Results go to `experiments/results/`, with commanded, encoder, IMU and angular velocity per sample.

## Geometry parameters (config.json)

**Active design: v2** (`bone-in SPM 2.step`, switched on 2026-11-18). The v1 config is kept as
`config.v1_before_v2_2026-11-18.json`. Every value is measured from the CAD:

```
python experiments/cad_to_config.py "bone-in SPM 2.step" --out config_v2.json      # geometry, mount, neutral, gear ratio
python experiments/cad_collision.py "bone-in SPM 2.step" limits --config config_v2.json --write   # collision limits (~10 min)
python experiments/rom_check.py config_v2.json                                      # range of motion, gait, torque
```

| key | meaning | v2 (active) | v1 |
|---|---|---|---|
| `alpha1_deg` | ring axis (−z) to middle-joint axis | 45 | 40 |
| `alpha2_deg` | middle-joint axis to head-joint axis | 90 | 90 |
| `beta_deg` | effector axis (femoral neck) to head-joint axis | 90 | 90 |
| `eta_deg` | azimuth of each head joint around the effector axis | 144.45, 24.45, −95.55 | −177.18, −57.18, 62.82 |
| `branch` | ±1 elbow direction per leg | −1, −1, −1 | −1, −1, −1 |
| `control.gear_ratio` | ring : pinion | 6 (108T internal / 18T) | 4 (72T / 18T) |
| `mount.*` | how the base sits on the pelvis (right leg) | 38.0° down, 8.3° forward | 24.6° down, 20.1° back |
| `neutral.*` | standing pose relative to the symmetric home | about 9° | 17.9° (the CAD pose) |

**Neutral (v2)** = the femur standing: mechanical axis vertical, with "vertical" taken from the casing's bottom
face, coloured orange in the CAD, and the knee axis horizontal. The leg in the v2 CAD file itself is at flex −17.7°,
add +17.1°, rot +5.4°. In v1 there was no ground face, so the saved CAD pose was taken as neutral.

### Link-collision limits (also from the CAD)

`cad_collision.py ... limits` samples every CAD surface with points 1.5 mm apart, moves the parts with the
controller's own IK, and finds where they touch (closer than 0.3 mm). Everything that can collide reduces to two
kinds of check, which the safety gate runs every tick:

* **Ring separation** (`limits.min_ring_separation_deg`). v2: arms touch at 16.8° (legs 1–2) and 8.9°
  (legs 2–3, 1–3); the gate uses 16.8° + 1.5° = 18.3° for every pair. (v1: 43.5°.)
* **Passive-joint fold limits** (`limits.joint_limits`). The *elbow* angle is how far a middle link has turned
  against its arm, and the *wrist* angle how far against the femur head, both measured from the CAD pose
  (`ref_R_BE`). Each is allowed an interval, minus `margin_deg` (3°). The Safety panel shows the remaining room as
  "joint fold room".

The pose-dependent pairs (middle links against other legs or each other, femur against arms / rings / casing) keep
≥ 8.8 mm along the gait. Their closest approach anywhere is 1.31 mm, at an extreme pose (flex 85, add 25, rot −40).

**What this leaves (v2):** from neutral, flexion −11.5…+41°, adduction −14.5…+48°, int. rotation −7.5…+27.5°.
The built-in gait fits to ×0.85 (8% of the cycle refused), all by the **leg 1 elbow**, and only because of the 3°
margin: the gait folds it to −14.2° against contact at −16.0°. About 0.3 mm of relief at the arm/middle-link
contact (13–16 mm from the elbow axis) gives the full gait with margin; about 1 mm on legs 1 and 2 gives ×1.3.

### Remounting in the Geometry tab

`mount` and `neutral` together describe how the femur sits on the femur head. With **"Remount casing: keep the femur
attached to the head"** ticked (the default), changing only the mount recomputes neutral, so the change models a real
remount of the casing on the pelvis. For example, entering option A's mount (38.3 / 10.2 / 93.5) gives option A's neutral
(−10 / +5 / 0). Untick it only if you mean to change how the femur is attached to the head. Changing the mount alone models
a different build: changing only the tilt to 38.3° cut flexion reach from +54° to +33°.

### Viewing poses beyond the limits

**Motion tab → "View beyond limits"** shows poses the safety gate refuses, so you can see *why* a pose is
refused (the Safety panel turns the violated limit red, and the header says
`BEYOND LIMIT (view only): <reason>`). It is **for viewing only**:

* It can only be switched on while **disarmed**, and **ARM is refused** while it is on (the button is greyed
  out, and the server refuses as well). The control loop also never sends setpoints while it is on.
* The Pose sliders widen (flexion −60…120°, adduction ±60°, rotation ±80°) while it is on, and so do the Gait
  scale sliders (up to ×3).
* It works in every Motion mode, Gait included. The **Gait plot always shades the phases the safety gate
  refuses** (whether or not the switch is on), with a summary line underneath, e.g. "44% of the cycle beyond limits
  (shaded: rings too close)". Without the switch those phases are held at the last safe pose; with it they play
  in the 3D view.
* It cannot show poses the mechanism cannot reach at all (no IK solution); the header then says `OUT OF REACH`. The view stops at the last reachable
  pose (about 52° flexion with today's mount).
* Switching it off pulls the sliders back into range and returns the view to the last pose that passed the gate.

## Gait CSV format

A header row, then one row per sample. The phase column may be `phase`/`pct`/`percent`
(0-100) or `time` (rescaled to 0-100). Angle columns are `flex`, `add` (or `abd`, which is sign-flipped)
and `rot`, in degrees, positive meaning flexion, adduction and internal rotation. See `example_gait.csv`.

**The built-in curve is an approximate textbook shape:** ~30° flexion at contact, ~10° extension
near toe-off, ~10° adduction in loading and ~5° abduction in swing, plus a small, least-certain
rotation profile. Use measured data for anything quantitative.

## Tests

```
python tests/test_all.py
```
The protocol tests use the example frames printed in the MKS manual, including the long-packet
checksum 0x11.
