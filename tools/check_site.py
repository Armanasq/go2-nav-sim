#!/usr/bin/env python3
"""Check the built site:  python3 tools/check_site.py [_site] [--chrome /usr/bin/google-chrome] [--shots DIR]
1. every internal href/src resolves to a file, and every #fragment to an id on the target page
2. every page loads in headless Chromium at 1280 and 390 px wide with no console errors, failed requests or HTTP >= 400,
   no horizontal scrolling, and the interactive views mounted; screenshots are saved
Writes a summary to stdout (and to site/evidence/sitecheck.txt with --write). Exit code 1 on any failure."""
import argparse, functools, http.server, pathlib, re, sys, threading, time, urllib.parse

ap = argparse.ArgumentParser(); ap.add_argument('site', nargs='?', default='_site'); ap.add_argument('--chrome'); ap.add_argument('--shots')
ap.add_argument('--write', action='store_true'); a = ap.parse_args()
SITE = pathlib.Path(a.site).resolve(); out, fails = [], 0


def log(s):
    out.append(s); print(s)


# 1. links -----------------------------------------------------------------------------------------------------------------
pages = sorted(SITE.rglob('*.html')); ids = {}
for p in pages:
    ids[p] = set(re.findall(r'\sid="([^"]+)"', p.read_text()))
broken = []
for p in pages:
    for ref in re.findall(r'(?:href|src)="([^"]+)"', p.read_text()):
        if re.match(r'(https?:|mailto:|data:|#$|\{)', ref):
            continue
        path, _, frag = ref.partition('#'); path = urllib.parse.unquote(path.split('?')[0])
        target = (p.parent / path).resolve() if path else p
        if target.is_dir():
            target = target / 'index.html'
        if not target.exists():
            broken.append(f'{p.relative_to(SITE)} -> {ref} (missing)')
        elif frag and target.suffix == '.html' and frag not in ids.get(target, set()):
            broken.append(f'{p.relative_to(SITE)} -> {ref} (no such id)')
log(f'links: {len(pages)} pages checked, {len(broken)} broken'); [log('  BROKEN ' + b) for b in broken]; fails += len(broken)

# 2. browser ---------------------------------------------------------------------------------------------------------------
from playwright.sync_api import sync_playwright  # noqa: E402
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(SITE)); handler.log_message = lambda *x: None
srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler); threading.Thread(target=srv.serve_forever, daemon=True).start()
base = f'http://127.0.0.1:{srv.server_address[1]}/'
shots = pathlib.Path(a.shots) if a.shots else None
if shots: shots.mkdir(parents=True, exist_ok=True)
with sync_playwright() as pw:
    kw = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
    if a.chrome: kw['executable_path'] = a.chrome
    br = pw.chromium.launch(**kw)
    for p in pages:
        rel = str(p.relative_to(SITE))
        for w, h in ((1280, 900), (390, 844)):
            ctx = br.new_context(viewport={'width': w, 'height': h}, device_scale_factor=1); pg = ctx.new_page(); errs = []
            pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
            pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
            pg.on('requestfailed', lambda r: errs.append(f'failed: {r.url} {r.failure}') if 'favicon' not in r.url and 'fonts.g' not in r.url else None)
            pg.on('response', lambda r: errs.append(f'HTTP {r.status}: {r.url}') if r.status >= 400 else None)
            pg.goto(base + rel, wait_until='networkidle')
            for y in range(0, pg.evaluate('document.body.scrollHeight'), 600):     # trigger lazy mounts
                pg.evaluate(f'window.scrollTo(0, {y})'); time.sleep(0.15)
            pg.wait_for_load_state('networkidle'); time.sleep(1.5)
            mounts = pg.evaluate("[...document.querySelectorAll('[data-mount]')].map(e => [e.dataset.mount, e.children.length > 0 && !e.querySelector('.callout.warn')])")
            overflow = pg.evaluate('document.documentElement.scrollWidth - window.innerWidth')
            bad = [m for m, ok in mounts if not ok]
            status = 'ok' if not errs and not bad and overflow <= 1 else 'FAIL'
            fails += status != 'ok'
            log(f'{status:4s} {rel:42s} {w:4d}px  mounted: {",".join(m for m, ok in mounts if ok) or "-"}'
                + (f'  not mounted: {bad}' if bad else '') + (f'  horizontal overflow {overflow}px' if overflow > 1 else ''))
            for e in errs[:5]:
                log('       ' + e[:160])
            if shots and (rel in ('index.html', 'evidence.html') or rel.startswith('tutorial/04')):
                pg.evaluate('window.scrollTo(0, 0)'); time.sleep(0.3)
                name = rel.replace('/', '_').replace('.html', '')
                pg.screenshot(path=str(shots / f'{name}-{w}.jpg'), full_page=True, type='jpeg', quality=82)
            ctx.close()
    br.close()
srv.shutdown()
log(f'result: {"PASS" if not fails else f"{fails} FAILURES"}')
if a.write:
    (pathlib.Path(__file__).resolve().parents[1] / 'site/evidence/sitecheck.txt').write_text('\n'.join(out) + '\n')
sys.exit(1 if fails else 0)
