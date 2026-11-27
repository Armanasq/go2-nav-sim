// Run replay: top-down map + scrubber + error chart for one recorded Go2 Nav2 run.
// Usage: import { mountReplay } from './replay.js'; mountReplay(el, { base: 'data/runs/', runs: ['r1','r2','r3'], initial: 'r3' });
// Data contract: <base><id>/run.json, slam_map.json, slam_map.png (see site/data/runs). Metrics are shown as exported, never recomputed.

const TOKENS = ['gt', 'odom', 'amcl', 'plan', 'map', 'good', 'bad', 'ink', 'muted', 'line', 'surface', 'bg', 'mono'];
const LAYERS = [['gt', 'Ground truth'], ['odom', 'Odometry'], ['amcl', 'AMCL'], ['plan', 'Nav2 plan', 'dash'], ['map', 'SLAM map', 'area']];
const SPEEDS = [1, 2, 4];

// --- pure helpers ---------------------------------------------------------
async function getJSON(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.json();
}
function getImage(url) {
  return new Promise((res, rej) => {
    const im = new Image();
    im.onload = () => res(im);
    im.onerror = () => rej(new Error(`${url}: image failed to load`));
    im.src = url;
  });
}
// last index i with T[i] <= t (0 if t precedes all)
function bisect(T, t) {
  let lo = 0, hi = T.length - 1;
  if (t >= T[hi]) return hi;
  while (lo < hi) { const m = (lo + hi + 1) >> 1; if (T[m] <= t) lo = m; else hi = m - 1; }
  return lo;
}
function poseAt(arr, i, f) {
  const a = arr[i], b = arr[i + 1];
  if (!a || !b || f <= 0) return a;
  const dth = Math.atan2(Math.sin(b[2] - a[2]), Math.cos(b[2] - a[2]));
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + dth * f];
}
function sizeCanvas(cv, w, h) {
  const d = window.devicePixelRatio || 1;
  cv.width = Math.max(1, Math.round(w * d)); cv.height = Math.max(1, Math.round(h * d));
  cv.style.width = w + 'px'; cv.style.height = h + 'px';
  const c = cv.getContext('2d');
  c.setTransform(d, 0, 0, d, 0, 0);
  return c;
}

function prep(run, meta, img) {
  const S = run.samples;
  const pick = k => S.map(s => s[k] || null);
  const err = k => Float32Array.from(S, s => (s[k] && s.gt) ? Math.hypot(s[k][0] - s.gt[0], s[k][1] - s.gt[1]) : NaN);
  const A = run.arena;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  const grow = (x, y, hx, hy) => { x0 = Math.min(x0, x - hx); x1 = Math.max(x1, x + hx); y0 = Math.min(y0, y - hy); y1 = Math.max(y1, y + hy); };
  for (const [cx, cy, sx, sy] of [...A.walls, ...A.boxes]) grow(cx, cy, sx / 2, sy / 2);
  for (const [cx, cy, r] of A.cylinders) grow(cx, cy, r, r);
  grow(A.A[0], A.A[1], 0, 0); grow(A.B[0], A.B[1], 0, 0);
  const pad = 0.15;
  const eo = err('odom'), ea = err('amcl');
  let emax = 0;
  for (const v of [...eo, ...ea]) if (v > emax) emax = v;
  return {
    run, meta, img, arena: A, M: run.metrics || {},
    dur: run.duration ?? S[S.length - 1].t,
    T: Float64Array.from(S, s => s.t),
    gt: pick('gt'), odom: pick('odom'), amcl: pick('amcl'),
    eo, ea, emax,
    plans: [...(run.plans || [])].sort((a, b) => a.t - b.t),
    PT: null,
    b: { x0: x0 - pad, x1: x1 + pad, y0: y0 - pad, y1: y1 + pad },
    tint: null, tintCol: '',
  };
}

// --- component ------------------------------------------------------------
export function mountReplay(el, { base = 'data/runs/', runs = ['r1', 'r2', 'r3'], initial = runs[runs.length - 1] } = {}) {
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const runName = id => `Run ${runs.indexOf(id) + 1}`;
  const on = { gt: true, odom: true, amcl: true, plan: true, map: true };

  el.classList.add('replay', 'card');
  el.tabIndex = 0;
  el.setAttribute('role', 'region');
  el.setAttribute('aria-label', 'Run replay. Space plays or pauses, left and right arrows step one second.');
  el.innerHTML = `
  <div class="rp-head">
    <div class="rp-seg" role="group" aria-label="Run">${runs.map(r => `<button type="button" data-run="${r}" aria-pressed="false">${runName(r)}</button>`).join('')}</div>
    <button type="button" class="btn rp-play">Play</button>
    <div class="rp-seg rp-speed" role="group" aria-label="Playback speed">${SPEEDS.map(s => `<button type="button" data-speed="${s}" aria-pressed="${s === 1}" aria-label="${s} times speed">${s}×</button>`).join('')}</div>
    <span class="rp-time num">t = 0.0 s / –</span>
  </div>
  <div class="rp-layers">
    <div class="rp-toggles" role="group" aria-label="Map layers">${LAYERS.map(([k, label, cls]) => `<button type="button" class="pill" data-layer="${k}" aria-pressed="true"><i class="rp-sw ${cls || ''}" style="background-color:var(--${k})"></i>${label}</button>`).join('')}</div>
    <div class="legend rp-glyphs" aria-label="Map symbols">
      <span><svg viewBox="-8 -6 16 12" width="16" height="12" aria-hidden="true"><path d="M6 0L-5 4.5L-2.5 0L-5 -4.5Z" style="fill:none;stroke:var(--gt);stroke-width:1.6;stroke-linejoin:round"/></svg>GT pose</span>
      <span><svg viewBox="-8 -6 16 12" width="16" height="12" aria-hidden="true"><path d="M6 0L-5 4.5L-2.5 0L-5 -4.5Z" style="fill:var(--odom)"/></svg>odom pose</span>
      <span><svg viewBox="-8 -6 16 12" width="16" height="12" aria-hidden="true"><circle r="4" style="fill:none;stroke:var(--amcl);stroke-width:1.6"/></svg>AMCL pose</span>
      <span><svg viewBox="-8 -6 16 12" width="16" height="12" aria-hidden="true"><circle r="4" style="fill:var(--good)"/></svg>A start</span>
      <span><svg viewBox="-8 -6 16 12" width="16" height="12" aria-hidden="true"><circle r="4" style="fill:var(--bad)"/></svg>B goal</span>
    </div>
  </div>
  <div class="rp-map"><canvas role="img" aria-label="Arena map"></canvas><div class="rp-status" role="status">Loading…</div></div>
  <input class="rp-scrub" type="range" min="0" max="1" step="0.1" value="0" aria-label="Replay time in seconds" disabled>
  <div class="rp-chart-head">
    <span class="eyebrow">Position error vs ground truth (cm)</span>
    <span class="legend"><span><i style="background:var(--odom)"></i>odometry <b class="num rp-eo">–</b></span><span><i style="background:var(--amcl)"></i>AMCL <b class="num rp-ea">–</b></span></span>
  </div>
  <div class="rp-chartbox"><canvas role="img" aria-label="Error chart"></canvas></div>
  <div class="kpis rp-kpis"></div>
  <p class="rp-note">Scored against simulator ground truth. Odometry is lidar-inertial, aligned to the world frame; AMCL is Nav2 localisation.</p>`;

  const $ = s => el.querySelector(s);
  const mapBox = $('.rp-map'), mapCv = $('.rp-map canvas'), status = $('.rp-status');
  const chartBox = $('.rp-chartbox'), chartCv = $('.rp-chartbox canvas');
  const scrub = $('.rp-scrub'), playBtn = $('.rp-play'), timeEl = $('.rp-time');
  const eoEl = $('.rp-eo'), eaEl = $('.rp-ea'), kpis = $('.rp-kpis');
  const mStatic = document.createElement('canvas'), cStatic = document.createElement('canvas');

  let D = null, t = 0, speed = 1, playing = false, last = 0, raf = 0, seq = 0, autoplay = !reduceMotion;
  let mctx = null, cctx = null, mw = 0, mh = 0, cw = 0, ch = 0;
  let s = 1, ox = 0, oy = 0; // world->css px
  const X = x => ox + (x - D.b.x0) * s, Y = y => oy + (D.b.y1 - y) * s;
  const CL = 40, CR = 10, CT = 8, CB = 20; // chart plot padding
  let ctop = 1;
  const C = {};
  const readColours = () => { const cs = getComputedStyle(document.documentElement); for (const k of TOKENS) C[k] = cs.getPropertyValue('--' + k).trim(); };
  readColours();
  const font = (px, w = 500) => `${w} ${px}px ${C.mono || 'monospace'}`;

  // --- static layers (rebuilt on resize / theme / run / map toggle) ---
  function buildMapStatic() {
    if (!mw) return;
    const c = sizeCanvas(mStatic, mw, mh);
    c.fillStyle = C.surface; c.fillRect(0, 0, mw, mh);
    if (!D) return;
    const { b, arena: A, meta } = D;
    s = Math.min((mw - 16) / (b.x1 - b.x0), (mh - 16) / (b.y1 - b.y0));
    ox = (mw - (b.x1 - b.x0) * s) / 2; oy = (mh - (b.y1 - b.y0) * s) / 2;
    // 1 m grid
    c.strokeStyle = C.line; c.lineWidth = 1; c.beginPath();
    for (let x = Math.ceil(b.x0); x <= b.x1; x++) { const px = Math.round(X(x)) + .5; c.moveTo(px, Y(b.y1)); c.lineTo(px, Y(b.y0)); }
    for (let y = Math.ceil(b.y0); y <= b.y1; y++) { const py = Math.round(Y(y)) + .5; c.moveTo(X(b.x0), py); c.lineTo(X(b.x1), py); }
    c.stroke();
    if (on.map && D.img) {
      if (!D.tint || D.tintCol !== C.map) { // recolour occupancy image with --map, keeping its alpha
        const tc = document.createElement('canvas'); tc.width = D.img.naturalWidth; tc.height = D.img.naturalHeight;
        const x = tc.getContext('2d'); x.drawImage(D.img, 0, 0); x.globalCompositeOperation = 'source-in'; x.fillStyle = C.map; x.fillRect(0, 0, tc.width, tc.height);
        D.tint = tc; D.tintCol = C.map;
      }
      const wM = meta.width * meta.resolution, hM = meta.height * meta.resolution;
      c.save(); c.globalAlpha = 0.9; c.imageSmoothingEnabled = false;
      c.drawImage(D.tint, X(meta.origin[0]), Y(meta.origin[1] + hM), wM * s, hM * s);
      c.restore();
    }
    const rect = ([cx, cy, sx, sy]) => [X(cx - sx / 2), Y(cy + sy / 2), sx * s, sy * s];
    c.fillStyle = C.ink;
    for (const w of A.walls) c.fillRect(...rect(w));
    c.lineWidth = 1.25; c.strokeStyle = C.muted; c.fillStyle = C.line;
    for (const bx of A.boxes) { c.fillRect(...rect(bx)); c.strokeRect(...rect(bx)); }
    for (const [cx, cy, r] of A.cylinders) { c.beginPath(); c.arc(X(cx), Y(cy), r * s, 0, 7); c.fill(); c.stroke(); }
    // scale bar, 1 m, inner bottom-left corner
    const sx0 = X(b.x0 + 0.5), sy0 = Y(b.y0 + 0.45);
    c.strokeStyle = C.muted; c.lineWidth = 1.5; c.beginPath();
    c.moveTo(sx0, sy0 - 4); c.lineTo(sx0, sy0); c.lineTo(sx0 + s, sy0); c.lineTo(sx0 + s, sy0 - 4); c.stroke();
    c.fillStyle = C.muted; c.font = font(11); c.textBaseline = 'bottom'; c.fillText('1 m', sx0 + 3, sy0 - 3);
    // A / B markers
    const mr = Math.max(5, 0.16 * s);
    for (const [p, col, lab] of [[A.A, C.good, 'A'], [A.B, C.bad, 'B']]) {
      c.beginPath(); c.arc(X(p[0]), Y(p[1]), mr * 1.9, 0, 7); c.strokeStyle = col; c.lineWidth = 1.5; c.stroke();
      c.beginPath(); c.arc(X(p[0]), Y(p[1]), mr * 0.7, 0, 7); c.fillStyle = col; c.fill();
      c.font = font(13, 600); c.textBaseline = 'middle'; c.fillText(lab, X(p[0]) + mr * 2.3, Y(p[1]) - mr * 1.6);
    }
  }

  const cx = v => CL + (v / D.dur) * (cw - CL - CR);
  const cy = v => CT + (ch - CT - CB) * (1 - v / ctop);
  function buildChartStatic() {
    if (!cw) return;
    const c = sizeCanvas(cStatic, cw, ch);
    c.fillStyle = C.surface; c.fillRect(0, 0, cw, ch);
    if (!D) return;
    const ecm = Math.max(D.emax * 100, 1);
    const step = [1, 2, 5, 10, 20, 50, 100].find(v => ecm / v <= 4) || 200;
    ctop = Math.ceil(ecm / step) * step / 100;
    c.font = font(11, 400); c.lineWidth = 1;
    c.textAlign = 'right'; c.textBaseline = 'middle';
    for (let v = 0; v <= ctop * 100 + 1e-6; v += step) {
      const py = Math.round(cy(v / 100)) + .5;
      c.strokeStyle = C.line; c.beginPath(); c.moveTo(CL, py); c.lineTo(cw - CR, py); c.stroke();
      c.fillStyle = C.muted; c.fillText(String(v), CL - 6, py);
    }
    const pw = cw - CL - CR;
    const xs = [5, 10, 20, 30, 60].find(v => pw / (D.dur / v) >= 56) || 120;
    c.textBaseline = 'top';
    for (let v = 0; v <= D.dur; v += xs) { c.textAlign = v ? 'center' : 'left'; c.fillText(`${v} s`, Math.min(cx(v), cw - CR - 14), ch - CB + 5); }
    c.lineJoin = 'round';
    for (const [arr, col] of [[D.ea, C.amcl], [D.eo, C.odom]]) {
      c.strokeStyle = col; c.lineWidth = 1.5; c.beginPath();
      let pen = false;
      for (let i = 0; i < arr.length; i++) {
        if (Number.isNaN(arr[i])) { pen = false; continue; }
        const px = cx(D.T[i]), py = cy(arr[i]);
        pen ? c.lineTo(px, py) : c.moveTo(px, py); pen = true;
      }
      c.stroke();
    }
  }
  const rebuild = () => { buildMapStatic(); buildChartStatic(); };

  // --- per-frame drawing ---
  function trail(c, arr, i, pose) {
    c.beginPath(); let pen = false;
    for (let k = 0; k <= i; k++) {
      const p = arr[k];
      if (!p) { pen = false; continue; }
      pen ? c.lineTo(X(p[0]), Y(p[1])) : c.moveTo(X(p[0]), Y(p[1])); pen = true;
    }
    if (pose && pen) c.lineTo(X(pose[0]), Y(pose[1]));
    c.stroke();
  }
  function arrow(c, p, len) {
    c.save(); c.translate(X(p[0]), Y(p[1])); c.rotate(-p[2]);
    c.beginPath(); c.moveTo(len * .6, 0); c.lineTo(-len * .4, len * .38); c.lineTo(-len * .18, 0); c.lineTo(-len * .4, -len * .38); c.closePath();
    c.restore();
  }
  let lastTime = '', lastErr = '';
  function draw() {
    if (!mctx || !cctx) return;
    mctx.drawImage(mStatic, 0, 0, mw, mh);
    cctx.drawImage(cStatic, 0, 0, cw, ch);
    if (!D) return;
    const { T } = D, i = bisect(T, t);
    const f = i < T.length - 1 ? Math.min(1, Math.max(0, (t - T[i]) / (T[i + 1] - T[i]))) : 0;
    const c = mctx;
    c.lineJoin = c.lineCap = 'round';
    if (on.plan && D.plans.length) {
      D.PT ??= Float64Array.from(D.plans, p => p.t);
      const pi = bisect(D.PT, t), plan = D.PT[pi] <= t ? D.plans[pi] : null;
      if (plan && plan.pts.length > 1) {
        c.setLineDash([6, 5]); c.strokeStyle = C.plan; c.lineWidth = 2; c.beginPath();
        plan.pts.forEach(([x, y], k) => k ? c.lineTo(X(x), Y(y)) : c.moveTo(X(x), Y(y)));
        c.stroke(); c.setLineDash([]);
      }
    }
    const pose = {};
    for (const [k, w] of [['gt', 2.5], ['amcl', 1.5], ['odom', 2]]) {
      pose[k] = poseAt(D[k], i, f);
      if (on[k]) { c.strokeStyle = C[k]; c.lineWidth = w; trail(c, D[k], i, pose[k]); }
    }
    const len = Math.max(15, 0.55 * s);
    if (on.amcl && pose.amcl) { c.beginPath(); c.arc(X(pose.amcl[0]), Y(pose.amcl[1]), len * .32, 0, 7); c.strokeStyle = C.amcl; c.lineWidth = 2; c.stroke(); }
    if (on.odom && pose.odom) { arrow(c, pose.odom, len); c.fillStyle = C.odom; c.fill(); c.strokeStyle = C.surface; c.lineWidth = 1; c.stroke(); }
    if (on.gt && pose.gt) { arrow(c, pose.gt, len * 1.2); c.strokeStyle = C.gt; c.lineWidth = 2; c.stroke(); }

    // chart cursor
    const px = Math.round(cx(t)) + .5, cc = cctx;
    cc.strokeStyle = C.ink; cc.lineWidth = 1; cc.beginPath(); cc.moveTo(px, CT); cc.lineTo(px, ch - CB); cc.stroke();
    for (const [arr, col] of [[D.eo, C.odom], [D.ea, C.amcl]]) {
      if (Number.isNaN(arr[i])) continue;
      cc.beginPath(); cc.arc(cx(T[i]), cy(arr[i]), 3, 0, 7); cc.fillStyle = col; cc.fill();
    }

    const ts = `t = ${t.toFixed(1)} s / ${D.dur.toFixed(1)} s`;
    if (ts !== lastTime) { lastTime = ts; timeEl.textContent = ts; scrub.value = t; scrub.setAttribute('aria-valuetext', `${t.toFixed(1)} seconds of ${D.dur.toFixed(1)}`); }
    const cm = v => Number.isNaN(v) ? '–' : `${(v * 100).toFixed(1)} cm`;
    const es = cm(D.eo[i]) + '|' + cm(D.ea[i]);
    if (es !== lastErr) { lastErr = es; eoEl.textContent = cm(D.eo[i]); eaEl.textContent = cm(D.ea[i]); }
  }

  // --- playback ---
  function frame(now) {
    raf = 0;
    if (playing && D) {
      const dt = last ? Math.min(0.25, (now - last) / 1000) : 0; // ponytail: clamp so a backgrounded tab does not jump
      last = now; t += dt * speed;
      if (t >= D.dur) { t = D.dur; setPlaying(false); }
    }
    draw();
    if (playing) invalidate();
  }
  function invalidate() { if (!raf) raf = requestAnimationFrame(frame); }
  function setPlaying(p) {
    if (p && D && t >= D.dur - 0.05) t = 0;
    playing = !!(p && D); last = 0;
    playBtn.textContent = playing ? 'Pause' : 'Play';
    invalidate();
  }
  function seek(v) { if (!D) return; t = Math.min(D.dur, Math.max(0, v)); invalidate(); }

  // --- data loading ---
  const cache = {};
  function load(id) {
    const u = base + id + '/';
    return cache[id] ??= Promise.all([getJSON(u + 'run.json'), getJSON(u + 'slam_map.json'), getImage(u + 'slam_map.png')])
      .then(([r, m, im]) => prep(r, m, im))
      .catch(e => { delete cache[id]; throw e; });
  }
  function fmtKpis(M) {
    const v = (x, d) => (x == null ? '–' : Number(x).toFixed(d)); // exported values, fixed decimals only
    const items = [['Path length', v(M.path_m, 2), 'm'], ['Duration (recorded)', v(M.duration_s, 1), 's'], ['Odometry ATE, aligned', v(M.odom_ate_aligned_cm ?? M.odom_ate_rmse_cm, 1), 'cm'], ['AMCL RMSE', v(M.amcl_rmse_cm, 1), 'cm'], ['End distance to B (tolerance 0.25)', v(M.goal_error_m, 2), 'm']];
    kpis.innerHTML = items.map(([k, val, u]) => `<div class="kpi"><div class="v">${val}<small> ${u}</small></div><div class="k">${k}</div></div>`).join('');
  }
  function selectRun(id) {
    const my = ++seq;
    for (const b of el.querySelectorAll('[data-run]')) b.setAttribute('aria-pressed', String(b.dataset.run === id));
    status.hidden = false; status.classList.remove('err'); status.setAttribute('role', 'status');
    status.textContent = `Loading ${runName(id)}…`;
    load(id).then(d => {
      if (my !== seq) return;
      D = d; t = Math.min(t, D.dur); lastTime = lastErr = '';
      status.hidden = true;
      scrub.disabled = false; scrub.max = D.dur;
      const bw = (D.b.x1 - D.b.x0).toFixed(1), bh = (D.b.y1 - D.b.y0).toFixed(1);
      mapBox.style.setProperty('--rp-aspect', `${bw} / ${bh}`);
      mapCv.setAttribute('aria-label', `Top-down map of the ${bw} by ${bh} metre arena for ${runName(id)}: walls, boxes and cylinders, start A and goal B, ground-truth, odometry and AMCL trajectories drawn up to the current time, the current Nav2 global plan, and the final slam_toolbox map.`);
      chartCv.setAttribute('aria-label', `Position error of odometry and AMCL against simulator ground truth over the ${D.dur.toFixed(1)} s of ${runName(id)}, in centimetres. Click to seek.`);
      fmtKpis(D.M);
      rebuild(); invalidate();
      if (autoplay) { autoplay = false; startWhenVisible(); }
    }).catch(e => {
      if (my !== seq) return;
      D = null; setPlaying(false); scrub.disabled = true; kpis.innerHTML = '';
      timeEl.textContent = 't = – / –';
      status.hidden = false; status.classList.add('err'); status.setAttribute('role', 'alert');
      status.textContent = `Could not load ${runName(id)}. ${e.message}`;
      rebuild(); invalidate();
    });
  }
  function startWhenVisible() {
    const io = new IntersectionObserver(es => { if (es.some(e => e.isIntersecting)) { io.disconnect(); if (!userTouched) setPlaying(true); } }, { threshold: 0.4 });
    io.observe(mapBox);
  }
  let userTouched = false;

  // --- events ---
  el.addEventListener('click', e => {
    const b = e.target.closest('button');
    if (!b || !el.contains(b)) return;
    userTouched = true;
    if (b === playBtn) setPlaying(!playing);
    else if (b.dataset.run) selectRun(b.dataset.run);
    else if (b.dataset.speed) {
      speed = +b.dataset.speed;
      for (const x of el.querySelectorAll('[data-speed]')) x.setAttribute('aria-pressed', String(x === b));
    } else if (b.dataset.layer) {
      const k = b.dataset.layer; on[k] = !on[k]; b.setAttribute('aria-pressed', String(on[k]));
      if (k === 'map') buildMapStatic();
      invalidate();
    }
  });
  el.addEventListener('keydown', e => {
    if (e.altKey || e.ctrlKey || e.metaKey) return;
    if (e.key === ' ' && e.target.tagName !== 'BUTTON') { e.preventDefault(); userTouched = true; setPlaying(!playing); }
    else if ((e.key === 'ArrowLeft' || e.key === 'ArrowRight') && D) { e.preventDefault(); userTouched = true; seek(t + (e.key === 'ArrowRight' ? 1 : -1)); }
  });
  scrub.addEventListener('input', () => { userTouched = true; seek(+scrub.value); });
  let dragging = false;
  const seekX = e => { if (!D) return; const r = chartBox.getBoundingClientRect(); seek((e.clientX - r.left - CL) / (cw - CL - CR) * D.dur); };
  chartBox.addEventListener('pointerdown', e => { dragging = true; userTouched = true; chartBox.setPointerCapture(e.pointerId); seekX(e); });
  chartBox.addEventListener('pointermove', e => { if (dragging) seekX(e); });
  chartBox.addEventListener('pointerup', () => { dragging = false; });
  chartBox.addEventListener('pointercancel', () => { dragging = false; });

  const ro = new ResizeObserver(() => {
    const mr = mapBox.getBoundingClientRect(), cr = chartBox.getBoundingClientRect();
    const nmw = Math.floor(mapBox.clientWidth), nmh = Math.floor(mapBox.clientHeight), ncw = Math.floor(chartBox.clientWidth), nch = Math.floor(chartBox.clientHeight);
    if (!mr.width || !cr.width || (nmw === mw && nmh === mh && ncw === cw && nch === ch && mctx)) return;
    mw = nmw; mh = nmh; cw = ncw; ch = nch;
    mctx = sizeCanvas(mapCv, mw, mh); cctx = sizeCanvas(chartCv, cw, ch);
    rebuild(); draw(); // canvas resize clears it, so paint synchronously
  });
  ro.observe(mapBox); ro.observe(chartBox);

  const retheme = () => { readColours(); rebuild(); invalidate(); };
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', retheme);
  new MutationObserver(retheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  document.fonts?.ready.then(() => { rebuild(); invalidate(); });

  selectRun(runs.includes(initial) ? initial : runs[0]);
}
