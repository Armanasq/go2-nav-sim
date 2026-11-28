// "Inside one lidar sweep": point cloud + RGB + depth for the r3 snapshots.
// Host page needs an import map for "three" (OrbitControls imports it by bare specifier):
// <script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js"}}</script>
const THREE_URL = 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js';
const ORBIT_URL = 'https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js';

const GROUND_Z = -0.33;          // ground plane in lidar frame (lidar 0.10 m above trunk centre)
const TRUNK_Z = -0.10;           // trunk centre in lidar frame
const Z_MIN = -0.35, Z_MAX = 1.05; // fixed height ramp so snapshots are comparable
const RINGS = [2, 4, 6, 8, 10];
const CHASE = { pos: [-5.5, 0, 3.2], target: [3, 0, 0] };

// Turbo colormap, polynomial approximation (A. Mikhailov, 2019). t in [0,1] -> [r,g,b] in [0,1].
function turbo(t) {
  t = Math.min(1, Math.max(0, t));
  const c = (a) => Math.min(1, Math.max(0, a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * (a[4] + t * a[5]))))));
  return [
    c([0.13572138, 4.6153926, -42.66032258, 132.13108234, -152.94239396, 59.28637943]),
    c([0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604]),
    c([0.1066733, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973]),
  ];
}
const turboCss = (reverse) => 'linear-gradient(90deg,' + Array.from({ length: 11 }, (_, i) => {
  const [r, g, b] = turbo(reverse ? 1 - i / 10 : i / 10).map((v) => Math.round(v * 255));
  return `rgb(${r},${g},${b}) ${i * 10}%`;
}).join(',') + ')';

const h = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) k in e && k !== 'style' ? (e[k] = v) : e.setAttribute(k, v);
  e.append(...kids);
  return e;
};

export async function mountCloud(el, { base = 'data/runs/r3/' } = {}) {
  const run = await (await fetch(base + 'run.json')).json();
  const snaps = run.snapshots;

  // ---- DOM ----
  const stage = h('div', { className: 'cloud-stage' });
  const reset = h('button', { type: 'button', className: 'btn cloud-reset', 'aria-label': 'Reset 3D view to chase camera' }, 'Reset view');
  const hint = h('p', { className: 'cloud-hint muted' }, 'Drag to orbit, right-drag or two fingers to pan, scroll or pinch to zoom. Rings at 2–10 m on the ground plane.');
  const hLegend = h('div', { className: 'cloud-legend' },
    h('span', { className: 'cloud-legend-k' }, 'Height z, lidar frame (m)'),
    h('span', { className: 'cloud-ramp', style: `background:${turboCss(false)}`, 'aria-hidden': 'true' }),
    h('span', { className: 'cloud-ticks num' }, h('span', {}, `${Z_MIN.toFixed(2)} ground`), h('span', {}, ((Z_MIN + Z_MAX) / 2).toFixed(2)), h('span', {}, Z_MAX.toFixed(2))));

  const picker = h('div', { className: 'cloud-picker', role: 'group', 'aria-label': 'Snapshot' });
  const btns = snaps.map((s, i) => {
    const b = h('button', { type: 'button', className: 'btn', 'aria-pressed': 'false', 'aria-label': `Snapshot ${i + 1}, t = ${s.t.toFixed(1)} seconds` },
      `t = ${s.t.toFixed(1)} s`);
    b.addEventListener('click', () => select(i));
    picker.append(b);
    return b;
  });
  const rgb = h('img', { width: 640, height: 360, alt: '', decoding: 'async' });
  const depth = h('img', { width: 640, height: 360, alt: '', decoding: 'async' });
  const facts = h('dl', { className: 'cloud-facts num' });
  const dLegend = h('div', { className: 'cloud-legend' },
    h('span', { className: 'cloud-ramp', style: `background:${turboCss(true)}`, 'aria-hidden': 'true' }),
    h('span', { className: 'cloud-ticks num' }, h('span', {}, '0.3 m near'), h('span', {}, '8 m far')));
  const side = h('aside', { className: 'cloud-side', 'aria-label': 'Snapshot details' },
    picker,
    h('figure', {}, rgb, h('figcaption', { className: 'muted' }, 'Onboard RGB, 640×360, 1.2 rad horizontal FOV')),
    h('figure', {}, depth, h('figcaption', { className: 'muted' }, 'Depth, TURBO colour map; dark = invalid or beyond 8 m'), dLegend),
    facts);
  el.replaceChildren(h('div', { className: 'cloud' }, h('div', { className: 'cloud-grid' }, h('div', { className: 'cloud-main' }, stage, hLegend, hint), side)));

  // ---- 3D (optional: images and facts still work without it) ----
  let gl = null;
  try {
    const [THREE, { OrbitControls }] = await Promise.all([import(THREE_URL), import(ORBIT_URL)]);
    gl = initScene(THREE, OrbitControls, stage);
    stage.append(reset);
    reset.addEventListener('click', gl.resetView);
  } catch (err) {
    console.warn('cloud: 3D view unavailable', err);
    stage.classList.add('is-fallback');
    stage.replaceChildren(h('p', { className: 'cloud-notice', role: 'status' }, '3D view unavailable (WebGL or three.js failed to load); camera images and facts below are still live.'));
    hLegend.hidden = hint.hidden = true;
  }

  let token = 0;
  async function select(i) {
    const s = snaps[i], my = ++token;
    btns.forEach((b, j) => b.setAttribute('aria-pressed', String(j === i)));
    rgb.src = `${base}snap_${i}.jpg`;
    rgb.alt = `Onboard RGB image at t = ${s.t.toFixed(1)} s`;
    depth.src = `${base}snap_${i}_depth.png`;
    depth.alt = `Colour-mapped depth image at t = ${s.t.toFixed(1)} s`;
    const [x, y, yaw] = s.pose;
    const row = (k, v) => [h('dt', {}, k), h('dd', {}, v)];
    const fill = (n) => facts.replaceChildren(...[
      row('Time', `${s.t.toFixed(2)} s`),
      row('Lidar points', n.toLocaleString('en-GB')),
      row('Depth valid', `${(s.depth_valid * 100).toFixed(1)} %`),
      row('Pose x, y', `${x.toFixed(2)}, ${y.toFixed(2)} m`),
      row('Heading', `${(yaw * 180 / Math.PI).toFixed(1)}°`),
    ].flat());
    fill(s.points);
    if (!gl) return;
    const res = await fetch(`${base}snap_${i}.bin`);
    if (!res.ok) throw new Error(`snap_${i}.bin: HTTP ${res.status}`);
    const buf = await res.arrayBuffer();
    if (my !== token) return; // a newer selection won
    // ponytail: Float32Array view assumes a little-endian host (every browser platform in practice).
    const pts = new Float32Array(buf, 0, Math.floor(buf.byteLength / 12) * 3);
    gl.setPoints(pts);
    fill(pts.length / 3);
  }
  await select(Math.min(2, snaps.length - 1));
  return { select };
}

function initScene(THREE, OrbitControls, stage) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true }); // throws without WebGL
  renderer.setClearColor(0x000000, 0); // stage background (var(--code-bg)) shows through
  const canvas = renderer.domElement;
  canvas.tabIndex = 0;
  canvas.setAttribute('role', 'img');
  canvas.setAttribute('aria-label', '3D lidar point cloud around the robot, coloured by height. Arrow keys pan when focused.');
  stage.append(canvas);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
  camera.up.set(0, 0, 1); // z-up lidar frame; must be set before OrbitControls is built
  const controls = new OrbitControls(camera, canvas);
  controls.maxDistance = 40;
  controls.minDistance = 0.5;
  controls.listenToKeyEvents(canvas);

  // range rings + labels on the ground plane
  const ringMat = new THREE.LineBasicMaterial({ color: 0x7a8392, transparent: true, opacity: 0.45 });
  for (const r of RINGS) {
    const pts = Array.from({ length: 129 }, (_, k) => {
      const a = (k / 128) * Math.PI * 2;
      return new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), GROUND_Z);
    });
    scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ringMat));
    scene.add(label(THREE, `${r} m`, r * Math.SQRT1_2, -r * Math.SQRT1_2, GROUND_Z));
  }
  const axes = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-10, 0, GROUND_Z), new THREE.Vector3(10, 0, GROUND_Z),
    new THREE.Vector3(0, -10, GROUND_Z), new THREE.Vector3(0, 10, GROUND_Z)]);
  scene.add(new THREE.LineSegments(axes, new THREE.LineBasicMaterial({ color: 0x7a8392, transparent: true, opacity: 0.25 })));

  // robot trunk (0.65 x 0.3 x 0.2 m) and forward arrow
  const body = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(0.65, 0.3, 0.2)),
    new THREE.LineBasicMaterial({ color: 0xffffff }));
  body.position.set(0, 0, TRUNK_Z);
  scene.add(body);
  scene.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.33, 0, TRUNK_Z), 0.9, 0x56c2ee, 0.25, 0.14));
  scene.add(label(THREE, '+x forward', 1.55, 0, TRUNK_Z + 0.05));

  const mat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
  let cloud = null;

  const render = () => renderer.render(scene, camera);
  controls.addEventListener('change', render);

  function resize() {
    const w = stage.clientWidth, hgt = stage.clientHeight;
    if (!w || !hgt) return;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(w, hgt, false);
    mat.size = 2 * renderer.getPixelRatio(); // size is in drawing-buffer pixels
    camera.aspect = w / hgt;
    camera.fov = camera.aspect < 1 ? 68 : 50; // portrait stages: widen so the chase view keeps the sides
    camera.updateProjectionMatrix();
    render();
  }
  new ResizeObserver(resize).observe(stage);

  function resetView() {
    camera.position.set(...CHASE.pos);
    controls.target.set(...CHASE.target);
    controls.update();
  }

  function setPoints(pts) {
    if (cloud) { scene.remove(cloud); cloud.geometry.dispose(); }
    const col = new Float32Array(pts.length);
    for (let i = 2; i < pts.length; i += 3) col.set(turbo((pts[i] - Z_MIN) / (Z_MAX - Z_MIN)), i - 2);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pts, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    cloud = new THREE.Points(g, mat);
    scene.add(cloud);
    render();
  }

  resetView();
  resize();
  return { setPoints, resetView };
}

function label(THREE, text, x, y, z) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const ctx = c.getContext('2d');
  ctx.font = '500 34px "IBM Plex Mono", ui-monospace, monospace';
  ctx.fillStyle = '#aab4c2';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 6, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false, sizeAttenuation: false }));
  s.scale.set(0.16, 0.04, 1);
  s.center.set(0, 0.5);
  s.position.set(x, y, z);
  return s;
}
