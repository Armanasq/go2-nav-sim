// Theme toggle (auto -> light -> dark), copy buttons on code blocks, and lazy mounting of the interactive instruments.
const root = document.documentElement;
const store = { get: () => { try { return localStorage.getItem('theme'); } catch { return null; } },
                set: (v) => { try { v ? localStorage.setItem('theme', v) : localStorage.removeItem('theme'); } catch { /* private mode */ } } };

function applyTheme(t) {
  if (t) root.setAttribute('data-theme', t); else root.removeAttribute('data-theme');
  const b = document.querySelector('.theme-toggle'); if (b) { b.textContent = t || 'auto'; b.setAttribute('aria-label', `Colour theme: ${t || 'auto'}`); }
}
applyTheme(store.get());
document.querySelector('.theme-toggle')?.addEventListener('click', () => {
  const next = { null: 'light', light: 'dark', dark: null }[String(store.get())] ?? null;
  store.set(next); applyTheme(next);
});

// Copy button sits in a wrapper, not in the <pre>, so it stays put when the code scrolls sideways.
for (const pre of document.querySelectorAll('pre')) {
  const box = document.createElement('div'); box.className = 'code'; pre.before(box); box.appendChild(pre);
  const btn = document.createElement('button'); btn.className = 'copy'; btn.type = 'button'; btn.textContent = 'Copy';
  btn.addEventListener('click', async () => {
    const text = pre.querySelector('code')?.innerText ?? pre.innerText;
    try { await navigator.clipboard.writeText(text); btn.textContent = 'Copied'; }
    catch { const r = document.createRange(); r.selectNodeContents(pre); const s = getSelection(); s.removeAllRanges(); s.addRange(r); btn.textContent = 'Selected'; }
    btn.classList.add('done');
    setTimeout(() => { btn.textContent = 'Copy'; btn.classList.remove('done'); }, 1500);
  });
  box.appendChild(btn);
}

// Sidebar: always open beside the text, folded to one line on phones (open in the HTML, so it works without JS).
const side = document.querySelector('details.side');
if (side) {
  const phone = matchMedia('(max-width: 860px)'); const sync = () => { side.open = !phone.matches; };
  phone.addEventListener('change', sync); sync();
}

// Sections of the current page in the sidebar: mark the one being read.
const toc = [...document.querySelectorAll('.side .sections a')];
if (toc.length) {
  const heads = toc.map((a) => document.getElementById(decodeURIComponent(a.hash.slice(1))));
  const spy = () => {
    const i = heads.findLastIndex((h) => h && h.getBoundingClientRect().top < 120);
    toc.forEach((a, j) => a.classList.toggle('is-current', j === i));
  };
  addEventListener('scroll', spy, { passive: true }); spy();
}

// <div data-mount="replay|cloud|arch" ...>: load the module only when the element is about to be visible.
const base = document.body.dataset.root || '';
const mod = (p) => new URL(`${base}${p}`, document.baseURI).href;   // dynamic import() needs a URL, not 'static/...' (a bare specifier)
const mounts = {
  replay: (el) => import(mod('static/js/replay.js')).then((m) => m.mountReplay(el, { base: `${base}data/runs/`, runs: (el.dataset.runs || 'r1,r2,r3').split(','), initial: el.dataset.initial || 'r3' })),
  cloud: (el) => import(mod('static/js/cloud.js')).then((m) => m.mountCloud(el, { base: `${base}data/runs/${el.dataset.run || 'r3'}/` })),
  arch: (el) => import(mod('static/js/arch.js')).then((m) => m.mountArch(el, { src: `${base}data/architecture.json`, docBase: `${base}tutorial/`, repoBase: 'https://github.com/Armanasq/go2-nav-sim/blob/main/' })),
};
const io = new IntersectionObserver((entries) => {
  for (const e of entries) {
    if (!e.isIntersecting) continue;
    io.unobserve(e.target);
    const css = document.createElement('link'); css.rel = 'stylesheet'; css.href = `${base}static/css/${e.target.dataset.mount}.css`; document.head.appendChild(css);
    mounts[e.target.dataset.mount]?.(e.target).catch((err) => { e.target.innerHTML = `<p class="callout warn">Could not load this view: ${err.message}</p>`; console.error(err); });
  }
}, { rootMargin: '600px' });
document.querySelectorAll('[data-mount]').forEach((el) => io.observe(el));
