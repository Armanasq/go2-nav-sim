// Architecture explorer: data/architecture.json -> layered SVG diagram + detail panel (narrow screens: grouped button list).
// Usage: mountArch(el, { src, docBase, repoBase }). Vanilla ES module, no dependencies. Colours come from CSS (arch.css).

const SVGNS = 'http://www.w3.org/2000/svg';
const mk = (ns, tag, attrs = {}, ...kids) => {
  const e = ns ? document.createElementNS(ns, tag) : document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) if (v != null && v !== false) e.setAttribute(k, v === true ? '' : v);
  for (const k of kids.flat()) if (k != null && k !== false) e.append(k);
  return e;
};
const h = (t, a, ...k) => mk(null, t, a, ...k);
const s = (t, a, ...k) => mk(SVGNS, t, a, ...k);

// geometry (SVG user units == CSS px at full width)
const TARGET_W = 1056;           // 1120 column - gutters - stage padding
const H = 48, VSEP = 28, TOP = 44, LANE_MIN = 32;
const NODE_MIN_W = 124, WRAP_AT = 104, PAD_X = 36;  // PAD_X = left pad + glyph + gap + right pad
const NODE_MARGIN = 10, LANE_SEP = 14, LABEL_H = 15;
const F_NODE = '500 13px "IBM Plex Sans", system-ui, sans-serif';
const F_EDGE = '400 10.5px "IBM Plex Mono", ui-monospace, monospace';

const ctx = document.createElement('canvas').getContext('2d');
const tw = (text, font) => { ctx.font = font; return ctx.measureText(text).width; };

// Split a long label into two balanced lines, preferring spaces, then underscores.
function wrap(label) {
  if (tw(label, F_NODE) <= WRAP_AT) return [label];
  const cuts = [];
  for (let i = 1; i < label.length; i++) if (label[i] === ' ') cuts.push([label.slice(0, i), label.slice(i + 1)]);
  if (!cuts.length) for (let i = 1; i < label.length; i++) if (label[i - 1] === '_') cuts.push([label.slice(0, i), label.slice(i)]);
  let best = [label], bw = Infinity;
  for (const c of cuts) { const w = Math.max(tw(c[0], F_NODE), tw(c[1], F_NODE)); if (w < bw) { bw = w; best = c; } }
  return best;
}

let uid = 0;

export async function mountArch(el, { src = 'data/architecture.json', docBase = 'tutorial/', repoBase = 'https://github.com/Armanasq/go2-nav-sim/blob/main/' } = {}) {
  let data;
  try {
    const r = await fetch(src);
    if (!r.ok) throw new Error(`${r.status} ${r.statusText}`);
    data = await r.json();
  } catch (err) {
    el.replaceChildren(h('p', { class: 'callout warn' }, `Could not load the architecture data (${src}): ${err.message}`));
    console.error('[arch]', err);
    return null;
  }
  // measure with the real webfonts when they arrive; fall back after 1.5 s
  await Promise.race([
    Promise.all([document.fonts.load(F_NODE), document.fonts.load(F_EDGE)]).catch(() => {}),
    new Promise(r => setTimeout(r, 1500)),
  ]);

  const id = `arch${++uid}`;
  const cols = data.columns;
  const byId = new Map(data.nodes.map(n => [n.id, { ...n, lines: wrap(n.label) }]));
  const nodes = [...byId.values()];
  const edges = data.edges.filter(e => {
    const ok = byId.has(e.from) && byId.has(e.to);
    if (!ok) console.warn('[arch] edge with unknown endpoint skipped', e);
    return ok;
  }).map((e, i) => ({ ...e, i, a: byId.get(e.from), b: byId.get(e.to), tf: /\bTF$/.test(e.label) }));
  const byCol = cols.map((_, c) => nodes.filter(n => n.col === c));

  // ---- horizontal layout: per-column width from measured labels, gaps wide enough for adjacent-edge labels
  const colW = byCol.map(ns => Math.ceil(Math.max(NODE_MIN_W, ...ns.map(n => Math.max(...n.lines.map(l => tw(l, F_NODE))) + PAD_X))));
  for (const e of edges) e.w = Math.ceil(tw(e.label, F_EDGE)) + 10;
  const minGap = Math.max(48, ...edges.filter(e => Math.abs(e.a.col - e.b.col) === 1).map(e => e.w + 12));
  const gap = Math.max(minGap, (TARGET_W - colW.reduce((p, q) => p + q, 0)) / Math.max(1, cols.length - 1));
  const colX = []; colW.reduce((x, w, c) => (colX[c] = x, x + w + gap), 0);
  const W = Math.ceil(colX.at(-1) + colW.at(-1));
  for (const n of nodes) { n.x = colX[n.col]; n.w = colW[n.col]; }

  // ---- vertical layout: column 0 stacked, later columns pulled toward the mean of their left-hand neighbours
  const nbrs = n => edges.flatMap(e => e.a === n ? [e.b] : e.b === n ? [e.a] : []);
  byCol.forEach((ns, c) => {
    const want = ns.map(n => { const m = nbrs(n).filter(o => o.col < c); return m.length ? m.reduce((p, o) => p + o.y + H / 2, 0) / m.length : null; });
    let y = TOP - VSEP - H;
    ns.forEach((n, i) => { n.y = Math.max(TOP, y + H + VSEP, want[i] == null ? -Infinity : want[i] - H / 2); y = n.y; });
    const d = ns.map((n, i) => want[i] == null ? null : want[i] - H / 2 - n.y).filter(v => v != null);
    if (d.length && ns.length) {
      const shift = Math.max(TOP - ns[0].y, d.reduce((p, q) => p + q, 0) / d.length);
      for (const n of ns) n.y += shift;
    }
  });
  const cy = n => n.y + H / 2;

  // ---- routing. adjacent columns: S-curve in the gap; same column: vertical; longer spans: horizontal lane
  // through the intermediate columns at a y that is free of nodes and of other lanes.
  const lanes = [];
  const spanned = edges.filter(e => Math.abs(e.a.col - e.b.col) > 1).sort((p, q) => Math.abs(p.a.col - p.b.col) - Math.abs(q.a.col - q.b.col));
  for (const e of spanned) {
    const lo = Math.min(e.a.col, e.b.col) + 1, hi = Math.max(e.a.col, e.b.col) - 1;
    const blocks = [];
    for (let c = lo; c <= hi; c++) for (const n of byCol[c]) blocks.push([n.y - NODE_MARGIN, n.y + H + NODE_MARGIN]);
    for (const l of lanes) if (l.lo <= hi && l.hi >= lo) blocks.push([l.y - LANE_SEP, l.y + LANE_SEP]);
    const ys = cy(e.a), yt = cy(e.b), mid = (ys + yt) / 2;
    const free = y => y >= LANE_MIN && blocks.every(([b0, b1]) => y <= b0 || y >= b1);
    const cost = y => Math.abs(y - ys) + Math.abs(y - yt) + 0.01 * Math.abs(y - mid);
    const cands = [mid, ys, yt, ...blocks.flat()].filter(free);
    e.lane = cands.reduce((p, q) => cost(q) < cost(p) ? q : p);
    lanes.push({ lo, hi, y: e.lane });
  }
  // ports: distribute edge ends along each node side, ordered by where the edge heads next
  const sides = new Map();
  const port = (n, side, e, key) => { const k = n.id + side; if (!sides.has(k)) sides.set(k, []); sides.get(k).push({ e, n, key, end: n === e.a ? 'a' : 'b' }); };
  for (const e of edges) {
    if (e.a.col === e.b.col) continue;
    const fwd = e.b.col > e.a.col;
    port(e.a, fwd ? 'R' : 'L', e, e.lane ?? cy(e.b));
    port(e.b, fwd ? 'L' : 'R', e, e.lane ?? cy(e.a));
  }
  for (const list of sides.values()) {
    list.sort((p, q) => p.key - q.key);
    list.forEach((p, i) => { p.e[p.end + 'y'] = p.n.y + H * (i + 1) / (list.length + 1); });
  }
  const ARROW = 7;
  for (const e of edges) {
    const { a, b } = e;
    if (a.col === b.col) { // ponytail: straight vertical; assumes the two nodes are neighbours in their column
      const down = b.y > a.y, x = a.x + a.w / 2;
      const y1 = down ? a.y + H : a.y, y2 = down ? b.y : b.y + H, dir = down ? 1 : -1;
      e.d = `M${x} ${y1}V${y2 - dir * ARROW}`;
      e.tip = [x, y2, 0, dir];
      e.lx = x + 6 + e.w / 2; e.ly = (y1 + y2) / 2;
      continue;
    }
    const fwd = b.col > a.col, sgn = fwd ? 1 : -1;
    const x1 = fwd ? a.x + a.w : a.x, x2 = fwd ? b.x : b.x + b.w, y1 = e.ay, y2 = e.by;
    const x2s = x2 - sgn * ARROW;
    e.tip = [x2, y2, sgn, 0];
    if (e.lane == null) {
      const m = (x1 + x2s) / 2;
      e.d = `M${x1} ${y1}C${m} ${y1} ${m} ${y2} ${x2s} ${y2}`;
      e.lx = (x1 + x2) / 2; e.ly = (y1 + y2) / 2;
    } else {
      const ca = a.col + sgn, cb = b.col - sgn;
      const xa = fwd ? colX[ca] : colX[ca] + colW[ca], xb = fwd ? colX[cb] + colW[cb] : colX[cb];
      const m1 = (x1 + xa) / 2, m2 = (xb + x2s) / 2, L = e.lane;
      e.d = `M${x1} ${y1}C${m1} ${y1} ${m1} ${L} ${xa} ${L}H${xb}C${m2} ${L} ${m2} ${y2} ${x2s} ${y2}`;
      e.lx = (xa + xb) / 2; e.ly = L;
    }
  }
  const Hgt = Math.ceil(Math.max(...nodes.map(n => n.y + H), ...edges.map(e => (e.lane ?? 0) + LABEL_H / 2)) + 8);

  // ---- SVG
  const titleId = `${id}-t`, descId = `${id}-d`;
  const svg = s('svg', { class: 'arch-svg', viewBox: `0 0 ${W} ${Hgt}`, width: W, height: Hgt, role: 'group', 'aria-labelledby': titleId, 'aria-describedby': descId },
    s('title', { id: titleId }, 'go2-nav-sim architecture: data flow from simulator to evaluation'),
    s('desc', { id: descId }, 'Each node is a button; select one to show its topics, files and chapter. Arrow keys move between nodes.'));
  const gCols = s('g', { class: 'arch-cols', 'aria-hidden': 'true' });
  cols.forEach((c, i) => gCols.append(
    s('text', { class: 'arch-coltitle', x: colX[i], y: 14 }, c),
    s('line', { class: 'arch-colrule', x1: colX[i], x2: colX[i] + colW[i], y1: 22.5, y2: 22.5 })));
  const gEdges = s('g', { class: 'arch-edges', 'aria-hidden': 'true' });
  const gNodes = s('g', { class: 'arch-nodes' });
  const gLabels = s('g', { class: 'arch-elabels', 'aria-hidden': 'true' });
  for (const e of edges) {
    const [tx, ty, dx, dy] = e.tip, nx = -dy, ny = dx;
    const bx = tx - dx * ARROW, by = ty - dy * ARROW;
    e.g = s('g', { class: 'arch-edge' + (e.tf ? ' tf' : '') },
      s('path', { class: 'wire', d: e.d }),
      s('path', { class: 'head', d: `M${tx} ${ty}L${bx + nx * 3.5} ${by + ny * 3.5}L${bx - nx * 3.5} ${by - ny * 3.5}Z` }));
    e.lab = s('g', { class: 'arch-elabel' },
      s('rect', { x: -e.w / 2, y: -LABEL_H / 2, width: e.w, height: LABEL_H, rx: 3 }),
      s('text', { x: 0, y: 0, dy: '0.35em', 'text-anchor': 'middle' }, e.label));
    gEdges.append(e.g); gLabels.append(e.lab);
  }
  for (const n of nodes) {
    const lineY = n.lines.length === 1 ? [H / 2] : [H / 2 - 8, H / 2 + 8];
    n.g = s('g', { class: `arch-node ${n.kind}`, transform: `translate(${n.x},${n.y})`, tabindex: 0, role: 'button', 'aria-pressed': 'false', 'data-id': n.id,
      'aria-label': `${n.label} (${n.kind}, ${cols[n.col]})` },
      s('rect', { class: 'ring', x: -4, y: -4, width: n.w + 8, height: H + 8, rx: 9 }),
      s('rect', { class: 'box', width: n.w, height: H, rx: 6 }),
      n.kind === 'sensor' ? s('rect', { class: 'glyph', x: 12, y: H / 2 - 4, width: 8, height: 8, rx: 1 }) : s('circle', { class: 'glyph', cx: 16, cy: H / 2, r: 3.75 }),
      s('text', { class: 'arch-nlabel' }, n.lines.map((l, i) => s('tspan', { x: 28, y: lineY[i], dy: '0.35em' }, l))));
    gNodes.append(n.g);
  }
  svg.append(gCols, gEdges, gNodes, gLabels);

  // ---- narrow-screen list (same data, buttons grouped by column)
  const list = h('div', { class: 'arch-list' }, cols.map((c, i) => h('section', {},
    h('h4', { class: 'eyebrow' }, c),
    h('div', { class: 'arch-chips' }, byCol[i].map(n => (n.chip = h('button', { type: 'button', class: `arch-chip ${n.kind}`, 'aria-pressed': 'false', 'data-id': n.id },
      h('span', { class: 'glyph', 'aria-hidden': 'true' }), n.label)))))));

  const glyph = kind => kind === 'sensor'
    ? s('svg', { width: 12, height: 12, viewBox: '0 0 12 12', 'aria-hidden': 'true' }, s('rect', { class: 'glyph', x: 2, y: 2, width: 8, height: 8, rx: 1 }))
    : s('svg', { width: 12, height: 12, viewBox: '0 0 12 12', 'aria-hidden': 'true' }, s('circle', { class: 'glyph', cx: 6, cy: 6, r: 3.75 }));
  const wire = tf => s('svg', { width: 28, height: 10, viewBox: '0 0 28 10', 'aria-hidden': 'true', class: 'arch-edge' + (tf ? ' tf' : '') },
    s('path', { class: 'wire', d: 'M1 5H20' }), s('path', { class: 'head', d: 'M27 5L20 1.5L20 8.5Z' }));
  const legend = h('div', { class: 'arch-legend legend' },
    h('span', {}, glyph('sensor'), 'sensor'), h('span', {}, glyph('process'), 'process / node'),
    h('span', { class: 'arch-wireleg' }, wire(false), 'topic'), h('span', { class: 'arch-wireleg' }, wire(true), 'TF / transform'),
    h('span', { class: 'arch-hint arch-wireleg' }, 'Edge labels show for the hovered or selected node.'));

  const panel = h('div', { class: 'arch-panel', 'aria-live': 'polite' });
  const root = h('div', { class: 'arch' }, h('div', { class: 'arch-stage' }, svg), list, legend, panel);
  el.replaceChildren(root);

  // ---- state
  let sel = byId.has('lio') ? 'lio' : nodes[0]?.id, hov = null, foc = null;

  function placeLabels(on) {
    const placed = [];
    for (const e of edges) e.lab.classList.remove('on');
    for (const e of on.sort((p, q) => p.ly - q.ly)) {
      let y = e.ly;
      for (let moved = true, k = 0; moved && k < on.length; k++) {
        moved = false;
        for (const p of placed) if (Math.abs(p.x - e.lx) < (p.w + e.w) / 2 + 2 && Math.abs(p.y - y) < LABEL_H + 1) { y = p.y + LABEL_H + 1; moved = true; }
      }
      e.lab.setAttribute('transform', `translate(${e.lx},${y})`);
      e.lab.classList.add('on');
      placed.push({ x: e.lx, y, w: e.w });
    }
  }

  function paint() {
    const a = hov ?? foc ?? sel;
    const near = new Set([a]), on = [];
    for (const e of edges) {
      const hot = e.from === a || e.to === a;
      e.g.classList.toggle('hot', hot);
      if (hot) { near.add(e.from); near.add(e.to); on.push(e); gEdges.append(e.g); }
    }
    root.classList.toggle('arch-dim', (hov ?? foc) != null);
    for (const n of nodes) {
      n.g.classList.toggle('near', near.has(n.id));
      n.g.classList.toggle('sel', n.id === sel);
      n.g.setAttribute('aria-pressed', String(n.id === sel));
      n.chip.setAttribute('aria-pressed', String(n.id === sel));
    }
    placeLabels(on);
  }

  function renderPanel() {
    const n = byId.get(sel);
    if (!n) return;
    const ins = edges.filter(e => e.b === n), outs = edges.filter(e => e.a === n);
    const peer = (m) => h('button', { type: 'button', class: 'arch-peer', 'data-id': m.id }, m.label);
    const io = (title, list, other) => list.length ? h('div', { class: 'arch-io' }, h('span', { class: 'eyebrow' }, title),
      h('ul', {}, list.map(e => h('li', {}, h('code', {}, e.label), title === 'Inputs' ? ' from ' : ' to ', peer(other(e)))))) : null;
    const topics = n.topics?.length
      ? h('div', { class: 'table-wrap' }, h('table', {},
          h('thead', {}, h('tr', {}, ['Topic', 'Type', 'Rate', 'Frame'].map(t => h('th', { scope: 'col' }, t)))),
          h('tbody', {}, n.topics.map(t => h('tr', {}, h('td', {}, h('code', {}, t.name)), h('td', { class: 'mono', 'data-label': 'Type' }, t.type), h('td', { class: 'mono nowrap', 'data-label': 'Rate' }, t.rate), h('td', { class: 'mono', 'data-label': 'Frame' }, t.frame))))))
      : h('p', { class: 'muted arch-none' }, 'Publishes no topics of its own in this stack.');
    const files = n.files?.length ? h('div', {}, h('span', { class: 'eyebrow' }, 'Files'),
      h('ul', { class: 'arch-files' }, n.files.map(f => h('li', {}, h('a', { href: repoBase + f, target: '_blank', rel: 'noopener' }, f))))) : null;
    panel.replaceChildren(
      h('div', { class: 'arch-p-main' },
        h('p', { class: 'eyebrow' }, `${cols[n.col]} · ${n.kind}`),
        h('h3', {}, n.label),
        h('p', { class: 'arch-what' }, n.what),
        io('Inputs', ins, e => e.a), io('Outputs', outs, e => e.b),
        n.chapter ? h('p', { class: 'arch-chapter' }, h('a', { href: `${docBase}${n.chapter}.html` }, 'Read the chapter →')) : null),
      h('div', { class: 'arch-p-side' }, h('span', { class: 'eyebrow' }, 'Published topics'), topics, files));
  }

  function select(nid, { reveal = false } = {}) {
    if (!byId.has(nid)) return;
    sel = nid; paint(); renderPanel();
    if (reveal && panel.getBoundingClientRect().top > innerHeight - 80) panel.scrollIntoView({ block: 'start', behavior: matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
  }

  // keyboard: Up/Down within a column, Left/Right to the nearest node in the next non-empty column
  function neighbour(n, key) {
    const col = byCol[n.col], i = col.indexOf(n);
    if (key === 'ArrowUp') return col[i - 1];
    if (key === 'ArrowDown') return col[i + 1];
    const step = key === 'ArrowRight' ? 1 : -1;
    for (let c = n.col + step; c >= 0 && c < cols.length; c += step)
      if (byCol[c].length) return byCol[c].reduce((p, q) => Math.abs(cy(q) - cy(n)) < Math.abs(cy(p) - cy(n)) ? q : p);
  }
  for (const n of nodes) {
    n.g.addEventListener('pointerenter', () => { hov = n.id; paint(); });
    n.g.addEventListener('pointerleave', () => { hov = null; paint(); });
    n.g.addEventListener('focus', () => { if (n.g.matches(':focus-visible')) { foc = n.id; paint(); } });
    n.g.addEventListener('blur', () => { foc = null; paint(); });
    n.g.addEventListener('click', () => select(n.id));
    n.g.addEventListener('keydown', ev => {
      if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); select(n.id); }
      else if (ev.key.startsWith('Arrow')) { const m = neighbour(n, ev.key); ev.preventDefault(); m?.g.focus(); }
    });
    n.chip.addEventListener('click', () => select(n.id, { reveal: true }));
  }
  const chips = nodes.map(n => n.chip).sort((p, q) => p.compareDocumentPosition(q) & Node.DOCUMENT_POSITION_FOLLOWING ? -1 : 1);
  list.addEventListener('keydown', ev => {
    const i = chips.indexOf(ev.target);
    if (i < 0) return;
    const j = { ArrowDown: i + 1, ArrowRight: i + 1, ArrowUp: i - 1, ArrowLeft: i - 1 }[ev.key];
    if (j != null) { ev.preventDefault(); chips[Math.max(0, Math.min(chips.length - 1, j))].focus(); }
  });
  panel.addEventListener('click', ev => { const b = ev.target.closest('.arch-peer'); if (b) select(b.dataset.id); });

  select(sel);
  return { select, root };
}
