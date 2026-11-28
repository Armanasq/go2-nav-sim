#!/usr/bin/env python3
"""Check the built site:  python3 tools/check_site.py [_site] [--chrome /usr/bin/google-chrome] [--shots DIR]
1. every internal href/src resolves to a file, and every #fragment to an id on the target page
2. every page loads in headless Chromium at 1280 and 390 px wide with no console errors, failed requests or HTTP >= 400,
   no KaTeX errors, no horizontal scrolling, and every embedded instrument mounted
3. every redirect page (a removed or moved URL) lands on its target
With --shots, the pages in SHOTS are also loaded in the dark theme, and screenshots of the top of each are saved in both
themes at both widths. Exit code 1 on any failure."""
import argparse, functools, http.server, pathlib, re, sys, threading, time, urllib.parse

ap = argparse.ArgumentParser(); ap.add_argument('site', nargs='?', default='_site'); ap.add_argument('--chrome'); ap.add_argument('--shots')
a = ap.parse_args()
SITE = pathlib.Path(a.site).resolve(); fails = 0
# Chrome cancels a video download once it has the metadata it needs; that is not an error (a missing file still fails the link check).
MEDIA_ABORT = lambda r: r.failure == 'net::ERR_ABORTED' and urllib.parse.urlparse(r.url).path.endswith(('.mp4', '.webm'))
SHOTS = ['index.html', 'tutorial/04-*.html', 'projects/*/index.html', 'tasks/index.html', 'tasks/s*/index.html', 'modules/index.html', 'reference.html']

# 1. links -----------------------------------------------------------------------------------------------------------------
pages = sorted(SITE.rglob('*.html')); ids = {}
for p in pages:
    ids[p] = set(re.findall(r'\sid="([^"]+)"', p.read_text()))
broken = []
for p in pages:
    for ref in re.findall(r'(?:href|src|poster)="([^"]+)"', p.read_text()):
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
print(f'links: {len(pages)} pages checked, {len(broken)} broken'); [print('  BROKEN ' + b) for b in broken]; fails += len(broken)

redirects = {p: re.search(r'http-equiv="refresh" content="0; url=([^"]+)"', p.read_text()) for p in pages}
redirects = {p: m.group(1) for p, m in redirects.items() if m}
shot_pages = [next(iter(sorted(SITE.glob(pat))), None) for pat in SHOTS] if a.shots else []
shot_pages = [p for p in shot_pages if p]

# 2. browser ---------------------------------------------------------------------------------------------------------------
from playwright.sync_api import sync_playwright  # noqa: E402
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(SITE)); handler.log_message = lambda *x: None
srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler); threading.Thread(target=srv.serve_forever, daemon=True).start()
base = f'http://127.0.0.1:{srv.server_address[1]}/'
shots = pathlib.Path(a.shots) if a.shots else None
if shots:
    shots.mkdir(parents=True, exist_ok=True)
runs = [(p, w, h, 'light') for p in pages if p not in redirects for w, h in ((1280, 900), (390, 844))]
runs += [(p, w, h, 'dark') for p in shot_pages for w, h in ((1280, 900), (390, 844))]
with sync_playwright() as pw:
    kw = {'args': ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']}
    if a.chrome:
        kw['executable_path'] = a.chrome
    br = pw.chromium.launch(**kw)
    for p, w, h, scheme in runs:
        rel = str(p.relative_to(SITE))
        ctx = br.new_context(viewport={'width': w, 'height': h}, device_scale_factor=1, color_scheme=scheme); pg = ctx.new_page(); errs = []
        pg.on('console', lambda m: errs.append('console: ' + m.text) if m.type == 'error' else None)
        pg.on('pageerror', lambda e: errs.append('pageerror: ' + str(e)))
        pg.on('requestfailed', lambda r: errs.append(f'failed: {r.url} {r.failure}') if not ('favicon' in r.url or 'fonts.g' in r.url or MEDIA_ABORT(r)) else None)
        pg.on('response', lambda r: errs.append(f'HTTP {r.status}: {r.url}') if r.status >= 400 else None)
        pg.goto(base + rel, wait_until='load')        # not networkidle: a video's paused download keeps the network busy
        y = 0
        while y < pg.evaluate('document.body.scrollHeight'):                   # trigger lazy mounts and lazy images; the page grows as they load
            pg.evaluate(f'window.scrollTo(0, {y})'); time.sleep(0.1); y += 700
        pg.wait_for_load_state('load'); time.sleep(1.0)
        # late-loading images above an instrument can push it out of view after the scroll; bring each one into view as a reader would
        pg.evaluate("document.querySelectorAll('[data-mount]').forEach(e => e.scrollIntoView())"); time.sleep(0.5)
        try:                                                                   # instruments fetch their data after they scroll into view
            pg.wait_for_function("[...document.querySelectorAll('[data-mount]')].every(e => e.children.length)", timeout=20000)
        except Exception:
            pass
        mounts = pg.evaluate("[...document.querySelectorAll('[data-mount]')].map(e => [e.dataset.mount, e.children.length > 0 && !e.querySelector('.callout.warn')])")
        overflow = pg.evaluate('document.documentElement.scrollWidth - window.innerWidth')
        katex = pg.evaluate("[!!document.querySelector('script[src*=katex]'), document.querySelectorAll('.katex').length, document.querySelectorAll('.katex-error').length]")
        if katex[0] and katex[1] == 0:
            errs.append('KaTeX loaded but rendered nothing')
        if katex[2]:
            errs.append(f'{katex[2]} KaTeX errors: ' + pg.evaluate("[...document.querySelectorAll('.katex-error')].slice(0, 3).map(e => e.title || e.textContent).join(' | ')"))
        bad = [m for m, ok in mounts if not ok]
        status = 'ok' if not errs and not bad and overflow <= 1 else 'FAIL'
        fails += status != 'ok'
        print(f'{status:4s} {rel:48s} {w:4d}px {scheme:5s}  math: {katex[1]:3d}  mounted: {",".join(m for m, ok in mounts if ok) or "-"}'
              + (f'  not mounted: {bad}' if bad else '') + (f'  horizontal overflow {overflow}px' if overflow > 1 else ''))
        for e in errs[:5]:
            print('       ' + e[:200])
        if p in shot_pages:
            pg.evaluate('window.scrollTo(0, 0)'); time.sleep(0.3)
            name = rel.replace('/', '_').replace('.html', '')
            pg.screenshot(path=str(shots / f'{name}-{w}-{scheme}.jpg'), type='jpeg', quality=85)
        ctx.close()

    # 3. redirects ---------------------------------------------------------------------------------------------------------
    ctx = br.new_context(); pg = ctx.new_page()
    for p, to in redirects.items():
        rel = str(p.relative_to(SITE)); pg.goto(base + rel); pg.wait_for_load_state('load')
        want = urllib.parse.urljoin(base + rel, to)
        ok = pg.url.split('#')[0] == want.split('#')[0]
        fails += not ok
        print(f'{"ok" if ok else "FAIL":4s} {rel:48s} redirects to {pg.url[len(base):]}' + ('' if ok else f' (expected {to})'))
    ctx.close(); br.close()
srv.shutdown()
print(f'result: {"PASS" if not fails else f"{fails} FAILURES"}')
sys.exit(1 if fails else 0)
