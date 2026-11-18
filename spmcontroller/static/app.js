// app.js - browser side of the SPM hip controller.
//
// Responsibilities:
//   * open a WebSocket to server.py, send user intents as small JSON messages
//   * render the ~30 Hz state snapshots: badges, tables, log, and a three.js
//     3D view of the linkage + femur (commanded = orange, measured = blue)
// All kinematics live in Python; this file only DRAWS what the server computed,
// so the preview and the hardware can never disagree.

import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/OrbitControls.js';

// ---------------------------------------------------------------------------
// WebSocket plumbing
// ---------------------------------------------------------------------------
let ws = null;
let cfg = null;              // latest config.json contents (from "static" messages)
let gaitPreview = null;      // sampled gait curves for the plot
let lastState = null;

function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'state') onState(msg);
    else if (msg.type === 'static') onStatic(msg);
    else if (msg.type === 'error') toast(msg.msg);
  };
  // If the server restarts, keep trying to reconnect every second.
  ws.onclose = () => { setBadge('b-conn', 'server offline', 'bad'); setTimeout(connectWS, 1000); };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

// Sliders fire dozens of events per second; send at most every 50 ms per key,
// always delivering the LAST value (trailing edge) so nothing is lost.
const throttles = {};
function sendThrottled(key, makeMsg) {
  const t = throttles[key] || (throttles[key] = { timer: null, last: 0 });
  const fire = () => { t.last = performance.now(); t.timer = null; send(makeMsg()); };
  const wait = 50 - (performance.now() - t.last);
  if (wait <= 0) fire();
  else if (!t.timer) t.timer = setTimeout(fire, wait);
}

function toast(text) {
  const el = document.getElementById('toast');
  el.textContent = text; el.classList.add('show');
  clearTimeout(el._t); el._t = setTimeout(() => el.classList.remove('show'), 4000);
}

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(d);

// ---------------------------------------------------------------------------
// Tabs, mode buttons, generic slider wiring
// ---------------------------------------------------------------------------
document.querySelectorAll('.tabs button').forEach((b) => b.onclick = () => {
  document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === b.dataset.tab));
});

// Every range input shows its value in the <output> next to it.
document.querySelectorAll('.slider-row input[type=range]').forEach((r) => {
  const out = r.parentElement.querySelector('output');
  const upd = () => { out.textContent = r.value; };
  r.addEventListener('input', upd); upd();
});

let currentMode = 'pose';
function selectMode(name, notify = true) {
  currentMode = name;
  document.querySelectorAll('#mode-seg button').forEach((b) => b.classList.toggle('active', b.dataset.mode === name));
  document.querySelectorAll('.mode-panel').forEach((p) => p.hidden = p.dataset.mode !== name);
  if (notify) sendModeParams(name);
}
document.querySelectorAll('#mode-seg button').forEach((b) => b.onclick = () => selectMode(b.dataset.mode));

// Gather the parameters of a mode panel from its widgets.
function modeParams(name) {
  const v = (id) => parseFloat($(id).value);
  if (name === 'pose') return { flex: v('p-flex'), add: v('p-add'), rot: v('p-rot') };
  if (name === 'plane') return {
    azimuth_deg: v('pl-az'), elevation_deg: v('pl-el'), offset_deg: v('pl-off'),
    amplitude_deg: v('pl-amp'), freq_hz: v('pl-f'),
    base: $('pl-base').checked ? [v('p-flex'), v('p-add'), v('p-rot')] : [0, 0, 0],
  };
  if (name === 'gait') return { period_s: v('g-per'), scale: [v('g-sf'), v('g-sa'), v('g-sr')] };
  if (name === 'joint') return { offsets_deg: [v('j-0'), v('j-1'), v('j-2')] };
  return {};
}
function sendModeParams(name) {
  sendThrottled('mode-' + name, () => ({ type: 'mode', name, params: modeParams(name) }));
}
// Any input inside a mode panel re-sends that mode's parameters.
document.querySelectorAll('.mode-panel').forEach((p) => {
  p.querySelectorAll('input').forEach((inp) => {
    if (inp.type === 'file') return;
    inp.addEventListener('input', () => { if (p.dataset.mode === currentMode) sendModeParams(currentMode); });
  });
  p.querySelectorAll('.run').forEach((b) => b.onclick = () => { sendModeParams(currentMode); send({ type: 'run', on: true }); });
  p.querySelectorAll('.pause').forEach((b) => b.onclick = () => send({ type: 'run', on: false }));
  p.querySelectorAll('.reset').forEach((b) => b.onclick = () => send({ type: 'reset_time' }));
});
// The Pose sliders also feed the plane mode's base posture.
['p-flex', 'p-add', 'p-rot'].forEach((id) => $(id).addEventListener('input', () => {
  if (currentMode === 'plane' && $('pl-base').checked) sendModeParams('plane');
}));

function setRange(id, val) { const r = $(id); r.value = val; r.dispatchEvent(new Event('input')); }
$('p-zero').onclick = () => { ['p-flex', 'p-add', 'p-rot'].forEach((id) => setRange(id, 0)); };
$('j-zero').onclick = () => { ['j-0', 'j-1', 'j-2'].forEach((id) => setRange(id, 0)); };
document.querySelectorAll('.preset').forEach((b) => b.onclick = () => {
  setRange('pl-az', b.dataset.az); setRange('pl-el', b.dataset.el);
});

// ---- View beyond limits (viewing only) -------------------------------------
// The server refuses to turn this on while armed and refuses ARM while it is
// on, so this checkbox can never make the motors leave the safe set.  While
// on, the Pose sliders and the Gait scale sliders get wider ranges so poses
// past the limits can be dialled in.  [mode, normal range, wide range] per slider.
const VIEW_RANGES = {
  'p-flex': ['pose', [-25, 50], [-60, 120]], 'p-add': ['pose', [-25, 25], [-60, 60]], 'p-rot': ['pose', [-30, 30], [-80, 80]],
  'g-sf': ['gait', [0, 1.5], [0, 3]], 'g-sa': ['gait', [0, 1.5], [0, 3]], 'g-sr': ['gait', [0, 1.5], [0, 3]],
};
function applyPoseRanges(wide) {
  const clampedModes = new Set();
  for (const [id, [mode, normal, wider]] of Object.entries(VIEW_RANGES)) {
    const [lo, hi] = wide ? wider : normal;
    const r = $(id); const v = parseFloat(r.value);
    r.min = lo; r.max = hi;
    if (v < lo || v > hi) { setQuiet(id, Math.min(hi, Math.max(lo, v))); clampedModes.add(mode); }
  }
  // a slider pulled back into range must also pull the server's target back
  if (clampedModes.has(currentMode)) sendModeParams(currentMode);
  updateGaitLimits();
}
$('view-override').onchange = (e) => send({ type: 'view_override', on: e.target.checked });

// Global rate limit lives in config.control.max_rate_deg_s
$('max-rate').addEventListener('input', () => sendThrottled('rate',
  () => ({ type: 'config', patch: { control: { max_rate_deg_s: parseFloat($('max-rate').value) } } })));

// Gait CSV / built-in
$('g-csv').onchange = async (e) => {
  const f = e.target.files[0]; if (!f) return;
  send({ type: 'mode', name: 'gait', params: { csv: { text: await f.text(), name: f.name } } });
  e.target.value = '';
};
$('g-builtin').onclick = () => send({ type: 'mode', name: 'gait', params: { reset_keys: true } });

// ---------------------------------------------------------------------------
// Connection tab
// ---------------------------------------------------------------------------
$('connect').onclick = () => send({ type: 'connect', port: $('port').value, baud: parseInt($('baud').value), simulate: $('simulate').checked });
$('disconnect').onclick = () => send({ type: 'disconnect' });
$('refresh').onclick = () => send({ type: 'static' });
$('torque-off').onclick = () => send({ type: 'torque', on: false });
$('set-home').onclick = () => send({ type: 'set_home' });
$('arm').onclick = () => send({ type: 'arm' });
$('disarm').onclick = () => send({ type: 'disarm' });

// E-stop: button and Escape key (works from anywhere on the page).
const estop = () => { send({ type: 'estop' }); toast('E-STOP sent'); };
$('estop').onclick = estop;
// E-stop recovery: reset the latch (server checks comms/stall/home and holds pose)
const resetEstop = () => send({ type: 'reset_estop' });
$('estop-reset').onclick = resetEstop;
$('estop-reset2').onclick = resetEstop;
$('release-stalls').onclick = () => send({ type: 'release_stalls' });

// IMU board
$('imu-connect').onclick = () => send({ type: 'imu_connect', port: $('imu-port').value.trim() });
$('imu-disconnect').onclick = () => send({ type: 'imu_disconnect' });
$('imu-cal').onclick = () => { if (confirm('Calibration moves the leg (±15° tilts, ±25° twist). Continue?')) send({ type: 'imu_calibrate' }); };
$('imu-align').onclick = () => send({ type: 'imu_align' });
$('imu-trim').addEventListener('input', () => sendThrottled('trim',
  () => ({ type: 'config', patch: { imu: { trim_gain: parseFloat($('imu-trim').value) } } })));
window.addEventListener('keydown', (e) => { if (e.key === 'Escape') estop(); });

// Driver setup buttons
document.querySelectorAll('[data-setup]').forEach((b) => b.onclick = () => {
  const action = b.dataset.setup;
  const msg = { type: 'setup', action, addr: parseInt($('s-addr').value) };
  if (action === 'set_address') {
    msg.new_addr = parseInt($('s-new').value);
    if (!confirm(`Change driver ${msg.addr} to address ${msg.new_addr}?`)) return;
  }
  if (action === 'set_baud') {
    msg.baud = parseInt($('s-baud').value);
    if (!confirm(`Change driver ${msg.addr} baud to ${msg.baud}? You must reconnect afterwards.`)) return;
  }
  if (action === 'configure' && $('s-cur').value) msg.current_ma = parseInt($('s-cur').value);
  send(msg);
});

// ---------------------------------------------------------------------------
// Geometry / config forms (built from config.json)
// ---------------------------------------------------------------------------
const GEOM_ROWS = [
  ['alpha1_deg', 'α1 (°)', 'geometry'], ['alpha2_deg', 'α2 (°)', 'geometry'],
  ['beta_deg', 'β (°)', 'geometry'], ['eta_deg', 'η (°)', 'geometry'],
  ['branch', 'elbow branch', 'geometry'], ['address', 'motor address', 'motors'],
  ['direction', 'motor direction', 'motors'],
];
const FORMS = {
  'mount-form': ['mount', [['side', 'side'], ['tilt_down_deg', 'tilt down (°)'], ['forward_deg', 'forward (°)'], ['base_twist_deg', 'base twist (°)']]],
  'neutral-form': ['neutral', [['tilt_x_deg', 'tilt about x (°)'], ['tilt_y_deg', 'tilt about y (°)'], ['twist_deg', 'twist about z (°)']]],
  'control-form': ['control', [['gear_ratio', 'gear ratio (motor:ring)'], ['loop_hz', 'loop rate (Hz)'], ['acc', 'driver acc (0-255)'],
    ['max_motor_rpm', 'max motor RPM'], ['speed_headroom', 'speed headroom ×'], ['sync_threshold_deg', 'sync threshold (°)'], ['read_feedback', 'read encoders']]],
  'limits-form': ['limits', [['max_tilt_deg', 'max tilt (°)'], ['min_ring_separation_deg', 'min ring separation (°)'],
    ['max_ring_excursion_deg', 'max ring excursion (°)'], ['min_leg_margin', 'min leg margin'],
    ['min_parallel_index', 'min parallel index'], ['min_serial_index', 'min serial index']]],
};

function buildGeomForms() {
  const tb = document.querySelector('#geom-table tbody');
  tb.innerHTML = '';
  for (const [key, label, sec] of GEOM_ROWS) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${label}</td>`;
    for (let i = 0; i < 3; i++) {
      const val = sec === 'geometry' ? cfg.geometry[key][i] : cfg.motors[i][key];
      const td = document.createElement('td');
      if (key === 'branch' || key === 'direction') {
        td.innerHTML = `<select data-sec="${sec}" data-key="${key}" data-i="${i}">
          <option value="1"${val >= 0 ? ' selected' : ''}>+1</option><option value="-1"${val < 0 ? ' selected' : ''}>−1</option></select>`;
      } else {
        td.innerHTML = `<input type="number" step="any" data-sec="${sec}" data-key="${key}" data-i="${i}" value="${val}">`;
      }
      tr.appendChild(td);
    }
    tb.appendChild(tr);
  }
  for (const [formId, [sec, fields]] of Object.entries(FORMS)) {
    const f = $(formId); f.innerHTML = '';
    for (const [key, label] of fields) {
      const val = cfg[sec][key];
      let html;
      if (key === 'side') html = `<select data-sec="${sec}" data-key="${key}"><option${val === 'right' ? ' selected' : ''}>right</option><option${val === 'left' ? ' selected' : ''}>left</option></select>`;
      else if (typeof val === 'boolean') html = `<select data-sec="${sec}" data-key="${key}" data-bool="1"><option value="true"${val ? ' selected' : ''}>yes</option><option value="false"${!val ? ' selected' : ''}>no</option></select>`;
      else html = `<input type="number" step="any" data-sec="${sec}" data-key="${key}" value="${val}">`;
      f.insertAdjacentHTML('beforeend', `<span>${label}</span>${html}`);
    }
  }
}

// Build a partial config ("patch") from every field on the Geometry tab.
function collectGeomPatch() {
  const patch = { geometry: JSON.parse(JSON.stringify(cfg.geometry)), motors: JSON.parse(JSON.stringify(cfg.motors)) };
  document.querySelectorAll('#t-geom [data-sec]').forEach((el) => {
    const { sec, key, i } = el.dataset;
    let val = el.dataset.bool ? el.value === 'true' : (key === 'side' ? el.value : parseFloat(el.value));
    if (sec === 'geometry') patch.geometry[key][+i] = val;
    else if (sec === 'motors') patch.motors[+i][key] = val;
    else (patch[sec] = patch[sec] || {})[key] = val;
  });
  return patch;
}
$('geom-apply').onclick = () => send({ type: 'config', patch: collectGeomPatch(), keep_femur: $('keep-femur').checked });
$('geom-save').onclick = () => send({ type: 'config', patch: collectGeomPatch(), save: true, keep_femur: $('keep-femur').checked });

// ---------------------------------------------------------------------------
// "static" message: config, ports, gait preview
// ---------------------------------------------------------------------------
let firstStatic = true;
function onStatic(msg) {
  cfg = msg.config;
  gaitPreview = msg.gait_preview;
  updateGaitLimits();
  // Port list
  const sel = $('port'); const keep = sel.value || cfg.serial.port;
  sel.innerHTML = '';
  const ports = [...new Set([...(msg.ports || []), cfg.serial.port].filter(Boolean))];
  for (const p of ports) sel.insertAdjacentHTML('beforeend', `<option${p === keep ? ' selected' : ''}>${p}</option>`);
  if (firstStatic) {
    $('baud').value = String(cfg.serial.baud);
    setRange('max-rate', cfg.control.max_rate_deg_s);
    buildGeomForms();
    firstStatic = false;
  } else {
    // Refresh unless the user is typing in a Geometry field.  (A focused Apply
    // BUTTON must not block it: after a remount the server rewrites 'neutral',
    // and stale fields would be sent back on the next Apply.)
    const a = document.activeElement;
    const typing = a && ['INPUT', 'SELECT', 'TEXTAREA'].includes(a.tagName) && a.id !== 'keep-femur'
      && document.querySelector('#t-geom').contains(a);
    if (!typing) buildGeomForms();
  }
  drawGaitPlot(null);
}

// ---------------------------------------------------------------------------
// "state" message: badges, tables, 3D
// ---------------------------------------------------------------------------
function setBadge(id, text, cls) { const b = $(id); b.textContent = text; b.className = 'badge ' + (cls || ''); }

function onState(s) {
  lastState = s;
  setBadge('b-conn', s.connected ? (s.sim ? 'SIMULATED' : 'connected') : 'disconnected', s.connected ? (s.sim ? 'warn' : 'ok') : '');
  setBadge('b-home', s.homed ? 'homed' : 'not homed', s.homed ? 'ok' : 'warn');
  setBadge('b-arm', s.estopped ? 'E-STOPPED (latched)' : (s.armed ? 'ARMED'
      : (s.view_override ? 'VIEW ONLY: limits overridden, ARM blocked' : 'disarmed (preview only)')),
    (s.armed || s.estopped) ? 'bad' : (s.view_override ? 'warn' : ''));
  $('estop-reset').hidden = !s.estopped;
  syncWidgets(s);
  // view-beyond-limits checkbox mirrors the server; can't be ticked while armed
  const vo = $('view-override');
  if (vo.checked !== !!s.view_override) { vo.checked = !!s.view_override; applyPoseRanges(vo.checked); }
  vo.disabled = s.armed && !s.view_override;
  $('arm').disabled = !!s.view_override;
  // Out of reach (no IK solution) can't be drawn even with the override: say so plainly.
  const outOfReach = !s.safety.ok && s.safety.unreachable;
  setBadge('b-safe', s.safety.ok ? 'safe'
      : outOfReach ? `OUT OF REACH: no linkage pose exists (${s.safety.reason.replace(/^unreachable \(leg \[(.*)\]\)$/, 'leg $1')})` +
          (s.view_override ? ', cannot be drawn even with the override' : '')
      : (s.safety.overridden ? 'BEYOND LIMIT (view only): ' : 'REFUSED: ') + s.safety.reason,
    s.safety.ok ? 'ok' : (s.safety.overridden || (outOfReach && s.view_override) ? 'warn' : 'bad'));
  setBadge('b-rate', 'rate-limited', s.rate_limited ? 'warn' : 'muted');

  $('a-f').textContent = fmt(s.anat_cmd.flex); $('a-a').textContent = fmt(s.anat_cmd.add); $('a-r').textContent = fmt(s.anat_cmd.rot);
  const m = s.anat_meas || {};
  $('m-f').textContent = fmt(m.flex); $('m-a').textContent = fmt(m.add); $('m-r').textContent = fmt(m.rot);
  updateImu(s.imu || {});

  const rows = [];
  for (let i = 0; i < 3; i++) {
    rows.push(`<tr><td><span style="color:${LEG_COLORS[i]}">■</span> ${i + 1} (addr ${cfg ? cfg.motors[i].address : '?'})</td>
      <td>${fmt(s.theta_rel_deg[i])}</td><td>${s.counts_cmd[i]}</td><td>${s.counts_meas[i] ?? '–'}</td></tr>`);
  }
  $('motor-rows').innerHTML = rows.join('');

  const mt = s.metrics || {}; const L = cfg ? cfg.limits : {};
  const metr = [
    ['tilt', fmt(mt.tilt_deg) + '°', mt.tilt_deg > L.max_tilt_deg],
    ['min ring separation', fmt(mt.min_ring_sep_deg) + '°', mt.min_ring_sep_deg < L.min_ring_separation_deg],
    ['max ring excursion', fmt(mt.max_excursion_deg) + '°', mt.max_excursion_deg > L.max_ring_excursion_deg],
    // smallest remaining room before any passive joint (elbow / wrist) folds into a link collision
    ['joint fold room', mt.joint_room_deg === undefined ? '–' : fmt(mt.joint_room_deg) + '°', mt.joint_room_deg < 0],
    ['parallel index', fmt(mt.parallel_index, 2), mt.parallel_index < L.min_parallel_index],
    ['serial index', fmt(mt.serial_index, 2), mt.serial_index < L.min_serial_index],
    ['Jacobian cond.', fmt(mt.cond, 1), false],
    ['loop period', fmt(s.loop_ms, 1) + ' ms', s.loop_ms > 1.5 * 1000 / (cfg ? cfg.control.loop_hz : 50)],
    ['bus frames / errors', s.bus ? `${s.bus.tx} / ${s.bus.rx_err}` : '–', s.bus && s.bus.rx_err > 0],
  ];
  $('metrics').innerHTML = metr.map(([k, v, bad]) => `<div class="${bad ? 'bad' : ''}"><span>${k}</span><span>${v}</span></div>`).join('');

  let info = `mode: <b>${s.mode}</b>${s.running ? ' (running)' : ''}, t = ${fmt(s.mode_t, 2)} s`;
  if (s.gait_phase !== undefined) info += `, phase ${fmt(s.gait_phase, 0)} %`;
  if (s.mode === 'gait') $('g-label').textContent = 'curve: ' + s.mode_params.label;
  $('mode-info').innerHTML = info;

  const logEl = $('log');
  const atBottom = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 4;
  logEl.textContent = s.log.join('\n');
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;

  drawGaitPlot(s.mode === 'gait' ? s.gait_phase : null);
  update3D(s);
}

// ---------------------------------------------------------------------------
// When the SERVER changes mode targets (on ARM or E-stop reset it sets them to
// the current pose so nothing moves), copy them into the sliders WITHOUT
// firing input events - otherwise the old slider values would be sent back.
// ---------------------------------------------------------------------------
let lastUiSeq = null;
function setQuiet(id, val) {
  const r = $(id); r.value = val;
  const out = r.parentElement.querySelector('output'); if (out) out.textContent = r.value;
}
function syncWidgets(s) {
  if (s.ui_seq === lastUiSeq) return;
  const first = lastUiSeq === null;
  lastUiSeq = s.ui_seq;
  if (first && s.ui_seq === 0) return;          // fresh page, nothing pushed yet
  const p = s.mode_params;
  if (s.mode === 'pose') { setQuiet('p-flex', p.flex); setQuiet('p-add', p.add); setQuiet('p-rot', p.rot); }
  // joint offsets are sent alongside; fetch them from theta_rel
  s.theta_rel_deg.forEach((v, i) => setQuiet('j-' + i, v.toFixed(1)));
  // 'effector' (IMU calibration) has no panel of its own: keep the current one visible
  if (['pose', 'plane', 'gait', 'joint'].includes(s.mode)) selectMode(s.mode, false);
}

// ---------------------------------------------------------------------------
// IMU panel + readouts
// ---------------------------------------------------------------------------
function updateImu(i) {
  const a = i.anat || {};
  $('i-f').textContent = fmt(a.flex); $('i-a').textContent = fmt(a.add); $('i-r').textContent = fmt(a.rot);
  const w = i.w_pelvis_dps;
  $('omega').innerHTML = w
    ? `<div>about Z (flex+): <b>${fmt(w[2])}</b> °/s</div><div>about X (anterior axis): ${fmt(w[0])} °/s</div>` +
      `<div>about Y (vertical): ${fmt(w[1])} °/s</div><div>|ω| = ${fmt(Math.hypot(...w))} °/s</div>`
    : '<span class="hint">no calibrated IMU</span>';
  $('imu-cal-status').textContent = i.cal_status ? 'calibration: ' + i.cal_status : '';
  if (!i.present) { $('imu-status').innerHTML = '<span class="hint">not connected</span>'; return; }
  const rows = [
    ['port / rate', `${i.port} / ${fmt(i.rate_hz, 0)} Hz`, !i.connected],
    // true sensor rate measured on the PC (firmware labels 240 Hz; the real part runs ~230 Hz).
    // Gyro integration uses this; '(measuring)' until ~2 s of stream has been fitted.
    ['true sample rate', i.rate_converged ? fmt(i.true_rate_hz, 2) + ' Hz' : 'measuring…', !i.rate_converged],
    ['calibrated / aligned', `${i.calibrated ? 'yes' : 'NO'} / ${i.aligned ? 'yes' : 'no'}`, !i.calibrated],
    ['still (bias learning)', i.still ? 'yes' : 'no', false],
    ['IMU vs encoder', i.err_vs_enc_deg === undefined ? '–' : fmt(i.err_vs_enc_deg, 2) + '°', i.err_vs_enc_deg > 2],
    ['  tilt only', i.tilt_err_vs_enc_deg === undefined ? '–' : fmt(i.tilt_err_vs_enc_deg, 2) + '°', i.tilt_err_vs_enc_deg > 1.5],
    ['IMU vs command', i.err_vs_cmd_deg === undefined ? '–' : fmt(i.err_vs_cmd_deg, 2) + '°', false],
    ['trim (°)', (i.trim_deg || []).map((x) => fmt(x, 2)).join(', '), false],
    ['gyro bias (°/s)', (i.bias_dps || []).map((x) => fmt(x, 3)).join(', '), false],
    ['accel (g)', (i.accel_g || []).map((x) => fmt(x, 3)).join(', '), false],
    ['frames / gaps / crc', `${i.frames} / ${i.seq_gaps} / ${i.crc_errors}`, i.crc_errors > 0],
  ];
  $('imu-status').innerHTML = rows.map(([k, v, bad]) => `<div class="${bad ? 'bad' : ''}"><span>${k}</span><span>${v}</span></div>`).join('');
  $('imu-log').textContent = (i.text || []).join('\n');
}

// ---------------------------------------------------------------------------
// Gait plot (plain 2D canvas)
// ---------------------------------------------------------------------------
// One-line summary under the gait plot: how much of the cycle the gate refuses, and why.
function updateGaitLimits() {
  const el = $('g-limits'); if (!el || !gaitPreview || !gaitPreview.ok) return;
  const bad = gaitPreview.ok.filter((x) => !x).length, n = gaitPreview.ok.length;
  if (!bad) { el.textContent = 'whole cycle inside the limits'; el.className = 'hint'; return; }
  const reasons = [...new Set(gaitPreview.reason.filter(Boolean).map((r) => r.split(' (')[0].replace(/ at .*/, '')))];
  el.textContent = `${Math.round(100 * bad / n)}% of the cycle beyond limits (shaded: ${reasons.join(', ')}). ` +
    (lastState && lastState.view_override ? 'Shown anyway: view only.' : 'Held at the last safe pose; tick "View beyond limits" to see it.');
  el.className = 'hint warn-text';
}

function drawGaitPlot(phase) {
  const c = $('gait-plot'); if (!c || !gaitPreview) return;
  const g = c.getContext('2d'); const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);
  const all = [...gaitPreview.flex, ...gaitPreview.add, ...gaitPreview.rot];
  const lo = Math.min(-15, ...all) - 2, hi = Math.max(35, ...all) + 2;
  const X = (p) => 28 + (p / 100) * (W - 36), Y = (v) => 8 + (hi - v) / (hi - lo) * (H - 22);
  // Shade the phases the safety gate refuses (server: gait_preview.ok).  With
  // 'View beyond limits' on these phases are still played in the 3D view.
  if (gaitPreview.ok) {
    const n = gaitPreview.phase.length, dp = 100 / Math.max(1, n - 1);
    g.fillStyle = 'rgba(255, 90, 90, 0.18)';
    gaitPreview.ok.forEach((good, i) => {
      if (!good) { const p = gaitPreview.phase[i]; g.fillRect(X(Math.max(0, p - dp / 2)), 8, X(Math.min(100, p + dp / 2)) - X(Math.max(0, p - dp / 2)), H - 22); }
    });
  }
  g.strokeStyle = '#2c323c'; g.fillStyle = '#8b95a3'; g.font = '10px sans-serif';
  for (let v = Math.ceil(lo / 10) * 10; v <= hi; v += 10) {
    g.beginPath(); g.moveTo(28, Y(v)); g.lineTo(W - 8, Y(v)); g.stroke(); g.fillText(v + '°', 2, Y(v) + 3);
  }
  g.fillText('0%', 24, H - 2); g.fillText('% gait cycle', W / 2 - 30, H - 2); g.fillText('100%', W - 30, H - 2);
  const series = [['flex', '#e2843a'], ['add', '#58b8ff'], ['rot', '#b98bff']];
  for (const [k, col] of series) {
    g.strokeStyle = col; g.lineWidth = 1.6; g.beginPath();
    gaitPreview.phase.forEach((p, i) => { const x = X(p), y = Y(gaitPreview[k][i]); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke();
  }
  g.fillStyle = '#e2843a'; g.fillText('flex', W - 90, 14); g.fillStyle = '#58b8ff'; g.fillText('add', W - 62, 14); g.fillStyle = '#b98bff'; g.fillText('rot', W - 36, 14);
  if (phase !== null && phase !== undefined) {
    g.strokeStyle = '#fff'; g.lineWidth = 1; g.beginPath(); g.moveTo(X(phase), 8); g.lineTo(X(phase), H - 14); g.stroke();
  }
}

// ---------------------------------------------------------------------------
// 3D view.  World frame = pelvis frame (ISB): X anterior, Y up, Z right.
// Units: metres.  Server sends linkage points in the SPM BASE frame; we rotate
// them into the pelvis frame with R_PB (base axes as columns).
// ---------------------------------------------------------------------------
const LEG_COLORS = ['#ff6b6b', '#ffd166', '#6bd3ff'];
const container = $('three');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color('#15181d');
const camera = new THREE.PerspectiveCamera(40, 1, 0.005, 10);
camera.position.set(0.55, 0.15, 0.75);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, -0.12, 0);
controls.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.2));
const dl = new THREE.DirectionalLight(0xffffff, 1.4); dl.position.set(1, 2, 1.5); scene.add(dl);

// Pelvis axes + floor grid for orientation
const axes = new THREE.AxesHelper(0.12); scene.add(axes);
const grid = new THREE.GridHelper(1.0, 20, 0x2c323c, 0x22272e); grid.position.y = -0.46; scene.add(grid);

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize); resize();

// Camera presets
document.querySelectorAll('[data-view]').forEach((b) => b.onclick = () => {
  const t = controls.target, d = 0.9;
  const v = { front: [d, 0, 0], side: [0, 0, d], top: [0.001, d, 0], iso: [0.55, 0.3, 0.75] }[b.dataset.view];
  camera.position.set(t.x + v[0], t.y + v[1], t.z + v[2]); controls.update();
});

// --- helpers to place primitives -------------------------------------------
const V3 = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const mat = (color, opacity = 1) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.1,
  transparent: opacity < 1, opacity, depthWrite: opacity >= 1 });
const UNIT_CYL = new THREE.CylinderGeometry(1, 1, 1, 14);
function cyl(material) { const m = new THREE.Mesh(UNIT_CYL, material); scene.add(m); return m; }
// Stretch a unit cylinder between points a and b with radius r.
function setCyl(mesh, a, b, r) {
  const A = V3(a), B = V3(b), d = B.clone().sub(A), len = d.length();
  mesh.visible = len > 1e-6;
  if (!mesh.visible) return;
  mesh.position.copy(A).addScaledVector(d, 0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  mesh.scale.set(r, len, r);
}
// Tube along a polyline (rebuilt each frame; the polylines are tiny).
function tube(material) { const m = new THREE.Mesh(new THREE.BufferGeometry(), material); scene.add(m); return m; }
function setTube(mesh, pts, r) {
  mesh.geometry.dispose();
  mesh.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts.map(V3)), 24, r, 8, false);
}

// Base frame -> pelvis frame
let R_PB = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const toP = (v) => [
  R_PB[0][0] * v[0] + R_PB[0][1] * v[1] + R_PB[0][2] * v[2],
  R_PB[1][0] * v[0] + R_PB[1][1] * v[1] + R_PB[1][2] * v[2],
  R_PB[2][0] * v[0] + R_PB[2][1] * v[1] + R_PB[2][2] * v[2]];
const scale = (v, s) => [v[0] * s, v[1] * s, v[2] * s];

// --- scene objects ------------------------------------------------------------
const housing = cyl(mat('#6f86a3', 0.35));                 // gearbox housing (schematic)
const rings = [0, 1, 2].map((i) => {                        // the three coaxial rings
  const m = new THREE.Mesh(new THREE.TorusGeometry(1, 0.06, 8, 48), mat(LEG_COLORS[i], 0.8)); scene.add(m); return m;
});
const legs = [0, 1, 2].map((i) => ({
  prox: tube(mat(LEG_COLORS[i])), dist: tube(mat('#a9b8c8')),
  wAxis: cyl(mat(LEG_COLORS[i])), vAxis: cyl(mat('#e2843a')),
}));
function femurParts(color, opacity) {
  const m = mat(color, opacity);
  return { head: (() => { const s = new THREE.Mesh(new THREE.SphereGeometry(1, 24, 16), m); scene.add(s); return s; })(),
    neck: cyl(m), shaft: cyl(m), condyles: cyl(m), anterior: cyl(m) };
}
const femCmd = femurParts('#e2843a', 1.0);
const femMeas = femurParts('#58b8ff', 0.35);
const femImu = femurParts('#5fe08a', 0.45);
const planeDisc = new THREE.Mesh(new THREE.CircleGeometry(0.3, 48),
  new THREE.MeshBasicMaterial({ color: 0xe2843a, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
scene.add(planeDisc);

function placeFemur(parts, f, show) {
  for (const p of Object.values(parts)) p.visible = show;
  if (!show) return;
  parts.head.position.set(...f.head); parts.head.scale.setScalar(f.head_r);
  setCyl(parts.neck, f.head, f.neck_end, 0.011);
  setCyl(parts.shaft, f.neck_end, f.knee, 0.012);
  setCyl(parts.condyles, f.epi_med, f.epi_lat, 0.016);
  setCyl(parts.anterior, f.knee, f.toe, 0.005);              // points anterior: shows axial rotation
}

function update3D(s) {
  R_PB = s.R_PB;
  const zB = toP([0, 0, 1]);
  const rp = cfg ? cfg.visual.r_prox_m : 0.055, rd = cfg ? cfg.visual.r_dist_m : 0.07;

  // Housing behind the rings, along -z_B.
  setCyl(housing, scale(zB, -0.07), scale(zB, -0.15), 0.07);
  // Rings: small circles around z_B near the -z pole, slightly staggered.
  rings.forEach((m, i) => {
    const h = -rp * Math.cos(15 * Math.PI / 180) - i * 0.004;
    m.position.set(...scale(zB, h));
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), V3(zB));
    m.scale.setScalar(rp * Math.sin(15 * Math.PI / 180) + i * 0.003);
  });
  // Linkage
  s.linkage_cmd.legs.forEach((L, i) => {
    setTube(legs[i].prox, L.proximal.map(toP), 0.0045);
    setTube(legs[i].dist, L.distal.map(toP), 0.0045);
    const w = toP(L.w), v = toP(L.v);
    setCyl(legs[i].wAxis, scale(w, rp - 0.006), scale(w, rd + 0.006), 0.0035);   // middle joint pin
    setCyl(legs[i].vAxis, scale(v, rd - 0.018), scale(v, rd + 0.006), 0.0045);    // head joint pin
  });
  // Femurs
  placeFemur(femCmd, s.femur_cmd, true);
  placeFemur(femMeas, s.femur_meas, !!s.femur_meas);
  placeFemur(femImu, s.imu && s.imu.femur, !!(s.imu && s.imu.femur));
  // Plane of motion (plane mode only): disc perpendicular to the rotation axis
  planeDisc.visible = s.mode === 'plane';
  if (planeDisc.visible) planeDisc.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), V3(s.mode_params.axis));
}

function animate() { requestAnimationFrame(animate); renderer.render(scene, camera); }
animate();

selectMode('pose', false);
connectWS();
