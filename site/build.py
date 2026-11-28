#!/usr/bin/env python3
"""Build the static site into _site/.   python3 site/build.py [--out _site]

Four sections. Learn: chapters (docs/NN-*.md), projects (projects/pNN_*/README.md) and practical-task stages
(tasks/sNN_*/README.md) in the order a learner does them. Tasks: every task in curriculum/spec.md with its status from
tasks/sNN_*/results/<id>.json. Deep dives: modules/mNN_*/README.md with site/data/roadmap.json. Reference: topics, frames,
parameters, commands and troubleshooting (docs/12). Whatever markdown exists at build time is rendered; numbers are read from
result files, so the site cannot disagree with the repository. URLs of removed pages get a small redirect page."""
import argparse, html, json, os, pathlib, re, shutil
import markdown
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]; SITE = ROOT / 'site'; OUT = ROOT / '_site'
REPO = 'https://github.com/Armanasq/go2-nav-sim'; BLOB = REPO + '/blob/main/'; TREE = REPO + '/tree/main/'
CUR = ' aria-current="page"'
NAV = [('Learn', 'tutorial/00-overview.html'), ('Tasks', 'tasks/index.html'), ('Deep dives', 'modules/index.html'), ('Reference', 'reference.html')]
# The learner's order from docs/00-overview.md ("Plan your route"), with each task stage placed after the chapter that teaches
# its tools. Keys: docs/NN chapter, pNN project, sNN stage. Missing items are skipped.
PATH = [('Start', ['docs/00', 'docs/01', 'docs/02', 'docs/03', 'docs/04', 'p01']),
        ('The robot', ['docs/05', 'docs/06', 's01', 's02', 's03', 'docs/07', 's04', 's05']),
        ('Move it', ['p02', 'p03', 'p04', 's08']),
        ('Sense and avoid', ['docs/08', 's06', 'p05', 'p06']),
        ('Estimate and map', ['docs/09', 's07', 'docs/11', 'docs/10', 's09']),
        ('Navigate', ['s10', 'p11', 'p12', 'p13', 's11']),
        ('Beyond simulation', ['p14', 'docs/13'])]
MEDIA = ('.png', '.svg', '.jpg', '.jpeg', '.gif', '.mp4', '.webm')
STILLS = ['tasks/s02_*/figures/2.png', 'projects/p03_*/figures/shapes.png', 'projects/p05_*/figures/scan_and_depth.png',
          'projects/p13_*/figures/patrol_path.png', 'projects/p*/figures/*.png']
ROUTES = {}          # repo path -> site path, filled before any page is rendered so every page can link to every other

KATEX = ('<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css">'
         '<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.js"></script>'
         '<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/contrib/auto-render.min.js" '
         'onload="renderMathInElement(document.querySelector(\'main\'),{delimiters:[{left:\'$$\',right:\'$$\',display:true},'
         '{left:\'$\',right:\'$\',display:false}],throwOnError:false})"></script>')
THREE = ('<script type="importmap">{"imports": {"three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js", '
         '"three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>')
esc = html.escape


# --- page shell ------------------------------------------------------------------------------------------------------------
def url(target, frm):
    """Relative URL from site page `frm` to site path `target` (which may carry a #fragment)."""
    path, _, frag = target.partition('#')
    return os.path.relpath(path, os.path.dirname(frm) or '.') + ('#' + frag if frag else '')


def write_page(rel, title, body, current='', desc='', head=''):
    root = '../' * rel.count('/')
    nav = ''.join(f'<a href="{root}{href}"{CUR if name == current else ""}>{name}</a>' for name, href in NAV)
    desc = desc or 'Learn to move, control, map and navigate a simulated Unitree Go2 in ROS 2 Humble.'
    p = OUT / rel; p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>{esc(title)}</title>
<meta name="description" content="{esc(desc[:160])}">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<link rel="stylesheet" href="{root}static/css/site.css">
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'%3E%3Ccircle cx='8' cy='8' r='5' fill='%230a6a9c'/%3E%3C/svg%3E">
{head}
</head>
<body data-root="{root}">
<a class="skip" href="#main">Skip to content</a>
<header class="topbar"><div class="inner">
<a class="brand" href="{root}index.html">go2-nav-sim</a>
<nav aria-label="Main">{nav}</nav>
<button class="theme-toggle" type="button" aria-label="Colour theme: auto">auto</button>
</div></header>
<main id="main">
{body}
</main>
<footer class="site"><div class="wrap">Simulation only. Source and issues on <a href="{REPO}">GitHub</a>.</div></footer>
<script type="module" src="{root}static/js/site.js"></script>
</body>
</html>
''')


def doc(label, side, article):
    """Sidebar and one text column. The sidebar is a <details>, open beside the text, folded to one line on phones."""
    return (f'<div class="wrap doc"><details class="side" open><summary>{esc(label)}</summary><nav aria-label="Contents">{side}</nav></details>'
            f'<article class="prose">{article}</article></div>')


def redirect(rel, target, keep_hash=True, hashes=None):
    to = url(target, rel)
    js = (f'var m={json.dumps({k: url(v, rel) for k, v in (hashes or {}).items()})},h=location.hash;'
          f'location.replace(m[h]||({str(keep_hash).lower()}&&h?{json.dumps(to.split("#")[0])}+h:{json.dumps(to)}));')
    (OUT / rel).parent.mkdir(parents=True, exist_ok=True)
    (OUT / rel).write_text(f'<!doctype html>\n<html lang="en"><head><meta charset="utf-8"><title>Moved</title>'
                           f'<meta http-equiv="refresh" content="0; url={to}"><link rel="canonical" href="{to}"><script>{js}</script></head>'
                           f'<body><p>This page moved to <a href="{to}">{to}</a>.</p></body></html>\n')


# --- markdown --------------------------------------------------------------------------------------------------------------
def md_convert(text):
    """Markdown to HTML without letting Markdown touch LaTeX: code is set aside first, then $$..$$ and $..$ math, then both are
    restored (code before conversion, math after). Returns (html, has_math, md)."""
    md = markdown.Markdown(extensions=['tables', 'fenced_code', 'toc', 'sane_lists'])
    code, math = [], []
    def keep(store, tag):
        return lambda m: (store.append(m.group(0)), f'{tag}{len(store) - 1}X')[1]
    t = re.sub(r'```.*?```', keep(code, 'CODEPH'), text, flags=re.S)
    t = re.sub(r'`[^`\n]+`', keep(code, 'CODEPH'), t)
    t = re.sub(r'\$\$.+?\$\$', keep(math, 'MATHPH'), t, flags=re.S)
    t = re.sub(r'(?<![\\$\w])\$(?=[^\s${])([^$\n]+?)(?<=[^\s\\])\$(?!\w)', keep(math, 'MATHPH'), t)
    t = re.sub(r'CODEPH(\d+)X', lambda m: code[int(m.group(1))], t)
    h = md.convert(t)
    # LaTeX accepts q_\min (one token); KaTeX wants q_{\min}. ponytail: fixed list of operator names, extend it if KaTeX reports another
    math = [re.sub(r'([_^])\\(min|max|inf|sup|lim|log|exp|det|arg)\b', r'\1{\\\2}', x) for x in math]
    h = re.sub(r'MATHPH(\d+)X', lambda m: esc(math[int(m.group(1))], quote=False), h)
    return h, bool(math), md


def route(rel):
    """Site path for a repository path, or None when the site does not serve it."""
    rel = rel.rstrip('/')
    if rel in ROUTES:
        return ROUTES[rel]
    if rel.startswith('docs/img/'):
        rel = 'img/' + rel[len('docs/img/'):]
    return rel if (OUT / rel).is_file() else None


def fix_links(h, src_dir, out_rel):
    """Rewrite every relative href/src written for GitHub (relative to the markdown file) to the page it becomes on the site;
    anything the site does not serve links to GitHub."""
    def fix(m):
        attr, path, frag = m.group(1), m.group(2), m.group(3) or ''
        if not path or re.match(r'[a-z]+:', path):
            return m.group(0)
        rel = os.path.normpath(os.path.join(src_dir, path)).replace(os.sep, '/')
        if rel.startswith('..'):
            return m.group(0)
        target = route(rel)
        if target is None and attr != 'href' and not (ROOT / rel).exists():
            # an image or video the source names but that is not there: try its folder's figures/ and media/, and say so
            moved = next((f'{src_dir}/{d}/{os.path.basename(rel)}' for d in ('figures', 'media') if route(f'{src_dir}/{d}/{os.path.basename(rel)}')), None)
            print(f'warning: a page in {src_dir} names missing file {rel}' + (f'; using {moved}' if moved else ''))
            target = moved and route(moved)
        if target is None:
            return f'{attr}="{(TREE if (ROOT / rel).is_dir() else BLOB) + rel + frag}"'
        if frag:
            target = target.split('#')[0] + frag
        return f'{attr}="{url(target, out_rel)}"'
    return re.sub(r'\b(href|src|poster)="([^"#]*)(#[^"]*)?"', fix, h)


def toc_list(tokens):
    """Flatten markdown's nested toc tokens into [(level, id, name)]."""
    out = []
    for t in tokens:
        out.append((t['level'], t['id'], t['name'])); out += toc_list(t['children'])
    return out


def render(src, out_rel):
    """One markdown file to an HTML fragment for page `out_rel`: title split off, a trailing "Next: ..." line dropped (the
    layout has prev/next), links rewritten, tables wrapped so they scroll on a phone."""
    text = src.read_text()
    title, body = text.split('\n', 1) if text.startswith('#') else ('', text)
    body = re.sub(r'\n+Next: \[.*?\]\(.*?\)\s*$', '\n', body)
    h, has_math, md = md_convert(body)
    h = fix_links(h, str(src.parent.relative_to(ROOT)), out_rel)
    h = h.replace('<table>', '<div class="table-wrap"><table>').replace('</table>', '</table></div>').replace('<img ', '<img loading="lazy" ')
    return dict(title=title.lstrip('#').strip(), html=h, toc=toc_list(md.toc_tokens), math=has_math)


def copy_assets(d, dst):
    """figures/, media/ and images or videos at the top of a content folder, copied next to its page so relative links work."""
    dst.mkdir(parents=True, exist_ok=True)
    for sub in ('figures', 'media'):
        if (d / sub).is_dir():
            shutil.copytree(d / sub, dst / sub, dirs_exist_ok=True, ignore=shutil.ignore_patterns('*.py', '__pycache__'))
    for f in d.iterdir():
        if f.is_file() and f.suffix.lower() in MEDIA:
            shutil.copy2(f, dst / f.name)


# --- data read at build time -----------------------------------------------------------------------------------------------
def read_json(p):
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError) as e:            # another team may be writing the file right now
        print(f'warning: skipped {p.relative_to(ROOT)}: {e}')
        return None


def stage_dir(n):
    return next((d for d in sorted(ROOT.glob(f'tasks/s{n:02d}_*')) if d.is_dir()), None)


def spec_stages():
    """Stages and task ids from curriculum/spec.md, with each task's status from its result file (none = spec)."""
    stages = []
    for m in re.finditer(r'^## Stage (\d+): (.+?)(?: \(.*\))?\n(.+)$', (ROOT / 'curriculum/spec.md').read_text(), re.M):
        n = int(m.group(1)); d = stage_dir(n); tasks = []
        for t in m.group(3).split(' | '):
            tm = re.match(r'(\d+\.\d+) (.+?)(?: \((.+)\))?$', t.strip())
            if not tm:
                continue
            r = read_json(d / 'results' / f'{tm.group(1)}.json') if d and (d / 'results' / f'{tm.group(1)}.json').exists() else None
            st = (r or {}).get('status') or {True: 'verified', False: 'failed'}.get((r or {}).get('pass'), 'spec')
            tasks.append(dict(id=tm.group(1), name=(r or {}).get('title') or tm.group(2)[0].upper() + tm.group(2)[1:],
                              shows=tm.group(3) or '', status=st, result=r))
        stages.append(dict(n=n, title=m.group(2)[0].upper() + m.group(2)[1:], dir=d, tasks=tasks))
    return stages


def runs():
    return [dict(id=p.stem[4:], **json.loads(p.read_text())) for p in sorted((ROOT / 'results').glob('run_*.json'))]


def missions():
    """RESULT lines of scripts/run_mission.sh, stored in site/evidence/missions.json by tools/collect_evidence.py."""
    p = SITE / 'evidence/missions.json'
    return json.loads(p.read_text()) if p.exists() else []


# --- learning path ---------------------------------------------------------------------------------------------------------
def find_item(key, stages):
    if key.startswith('docs/'):
        src = next(iter(sorted(ROOT.glob(f'docs/{key[5:]}-*.md'))), None)
        if not src:
            return None
        title = src.read_text().split('\n', 1)[0].lstrip('# ')
        num, name = title.split('. ', 1) if '. ' in title else ('', title)
        return dict(key=key, src=src, out=f'tutorial/{src.stem}.html', num=num, kind=f'Chapter {num}', name=name, label=name)
    kind, n = key[0], int(key[1:])
    d = stage_dir(n) if kind == 's' else next((d for d in sorted(ROOT.glob(f'projects/p{n:02d}_*')) if d.is_dir()), None)
    if not d or not (d / 'README.md').exists():
        return None
    first = (d / 'README.md').read_text().split('\n', 1)[0]
    m = re.match(r'#\s+(?:Project|Stage)?\s*\d*[.:]?\s*(.*)', first); name = m.group(1).strip() if m else d.name
    name = name[:1].upper() + name[1:]
    if kind == 's' and not name:
        name = next(s['title'] for s in stages if s['n'] == n)
    return dict(key=key, src=d / 'README.md', dir=d, out=f'{d.parent.name}/{d.name}/index.html', num=f'{kind.upper()}{n}',
                kind=f'{"Project" if kind == "p" else "Stage"} {n}' + (' tasks' if kind == 's' else ''), name=name, label=name.split(':')[0])


def learn_path(stages):
    parts = [(part, [it for it in (find_item(k, stages) for k in keys) if it]) for part, keys in PATH]
    listed = {k for _, keys in PATH for k in keys}
    extra = [find_item(f'{d.name[0]}{d.name[1:3]}', stages) for d in sorted(ROOT.glob('projects/p[0-9][0-9]_*')) + sorted(ROOT.glob('tasks/s[0-9][0-9]_*'))
             if f'{d.name[0]}{d.name[1:3]}' not in listed]
    parts[-2][1].extend(it for it in extra if it)     # ponytail: unplaced projects/stages join "Navigate"; add them to PATH when written
    return [(p, items) for p, items in parts if items]


def side_learn(parts, cur, toc):
    out = ''
    for part, items in parts:
        if any(it is cur for it in items):
            lis = ''
            for it in items:
                lis += f'<li><a href="{url(it["out"], cur["out"])}"{CUR if it is cur else ""}><span class="n">{it["num"]}</span><span>{esc(it["label"])}</span></a>'
                lis += (sections(toc) if it is cur else '') + '</li>'
            out += f'<li class="open"><p class="part">{part}</p><ol>{lis}</ol></li>'
        else:
            out += f'<li><a class="part" href="{url(items[0]["out"], cur["out"])}">{part}</a></li>'
    return f'<ol class="parts">{out}</ol>'


def sections(toc):
    items = ''.join(f'<li><a href="#{i}">{esc(html.unescape(n))}</a></li>' for lvl, i, n in toc if lvl == 2)
    return f'<ul class="sections">{items}</ul>' if items else ''


def prevnext(seq, i, frm):
    a = f'<a class="pn" href="{url(seq[i-1]["out"], frm)}"><span>Previous</span>{esc(seq[i-1]["name"])}</a>' if i else '<span></span>'
    b = f'<a class="pn next" href="{url(seq[i+1]["out"], frm)}"><span>Next</span>{esc(seq[i+1]["name"])}</a>' if i + 1 < len(seq) else '<span></span>'
    return f'<nav class="prevnext" aria-label="Previous and next">{a}{b}</nav>'


def insert_section(h, toc, sid, title, inner):
    """Add a site-only section (an instrument or measured results) before the chapter's Exercise, or at the end."""
    block = f'<h2 id="{sid}">{esc(title)}</h2>\n{inner}\n'
    at = h.find('<h2 id="exercise"')
    toc_at = next((k for k, (lvl, i, _) in enumerate(toc) if i == 'exercise'), len(toc))
    toc.insert(toc_at, (2, sid, title))
    return h[:at] + block + h[at:] if at >= 0 else h + block


def embeds():
    """What each chapter gains on the site: the instruments where they teach, and the measured runs where scoring is taught."""
    rs, ms = runs(), missions(); byid = {m['run']: m for m in ms}; rows = ''
    for r in rs:
        m = byid.get(r['id'], {}); ok = m.get('result') == 'SUCCEEDED'
        rows += (f'<tr><td>{r["id"]}</td><td class="{"good" if ok else "bad"}">{esc(m.get("result", "not recorded").lower())}</td>'
                 f'<td class="n">{m.get("sim_time_s", "")}</td><td class="n">{r["path_m"]}</td><td class="n">{r["goal_error_m"]}</td>'
                 f'<td class="n">{r["odom_ate_aligned_cm"]}</td><td class="n">{r["odom_rpe_1m_pct"]}</td><td class="n">{r["amcl_rmse_cm"]}</td></tr>')
    tol = rs[0].get('goal_tolerance_m', 0.25) if rs else 0.25
    table = ('<div class="table-wrap"><table><thead><tr><th>Run</th><th>Nav2</th><th class="n">Time (s)</th><th class="n">Path (m)</th>'
             '<th class="n">To B (m)</th><th class="n">ATE (cm)</th><th class="n">RPE (%)</th><th class="n">AMCL (cm)</th>'
             f'</tr></thead><tbody>{rows}</tbody></table></div>'
             f'<p>{len(rs)} consecutive runs of <code>scripts/run_mission.sh</code> from a fresh start, scored by <code>bag_metrics.py</code> '
             f'(<a href="{BLOB}results">results/run_*.json</a>). Every estimate is paired with ground truth interpolated at its own timestamp. '
             'ATE is the RMSE of odometry position after the best rigid SE(2) alignment; RPE is its translational error over 1 m of path; '
             'AMCL is the RMSE of AMCL position, scored only at AMCL\'s own updates. To B is where the robot stopped, measured from B. '
             f'Nav2 stops once its own estimate is within {tol} m of B, so that distance mostly reflects the tolerance and the AMCL error at that moment.</p>')
    return {
        '00-overview': ('the-stack-node-by-node', 'The stack, node by node',
                        '<p>Select a node for what it does, the topics it publishes and the files that configure it.</p><div class="instrument" data-mount="arch"></div>'),
        '08-robot-and-sensors': ('one-lidar-sweep-in-3d', 'One lidar sweep in 3D',
                                 '<p>Six moments from run w2. Drag to orbit; the RGB and depth images were taken at the same instant as the sweep.</p>'
                                 '<div class="instrument" data-mount="cloud" data-run="w2"></div>'),
        '10-mapping-and-navigation': ('replay-a-recorded-run', 'Replay a recorded run',
                                      '<p>Ground truth, odometry and AMCL from the bags of runs w1 to w3 at 10 Hz. Scrub the timeline or click the error chart.</p>'
                                      '<div class="instrument" data-mount="replay" data-runs="w1,w2,w3" data-initial="w2"></div>'),
        '11-record-and-visualise': ('measured-runs', 'Measured runs', table) if rs else None,
    }


def build_learn(parts):
    seq = [it for _, items in parts for it in items]; part_of = {it['key']: p for p, items in parts for it in items}
    extra = embeds()
    for i, it in enumerate(seq):
        r = it['rendered']; head = KATEX if r['math'] else ''
        e = extra.get(it['src'].stem) if it['key'].startswith('docs/') else None
        if e:
            r['html'] = insert_section(r['html'], r['toc'], *e)
            head += THREE if 'data-mount="cloud"' in e[2] else ''
        src = it['src'].relative_to(ROOT)
        article = (f'<p class="where">{part_of[it["key"]]} · {it["kind"]}</p><h1>{esc(it["name"])}</h1>{r["html"]}'
                   f'{prevnext(seq, i, it["out"])}<p class="source">Source: <a href="{BLOB}{src}">{src}</a></p>')
        write_page(it['out'], f'{it["name"]} · go2-nav-sim', doc(it['label'], side_learn(parts, it, r['toc']), article), 'Learn', it['name'], head)
    return seq


# --- tasks -----------------------------------------------------------------------------------------------------------------
def stage_page(s, item):
    """A stage with a README is a Learn page (status added to each task heading); a stage with results only gets a page of
    its acceptance results; a stage with neither has no page."""
    if item:
        r = item['rendered']
        for t in s['tasks']:
            t['anchor'] = next((i for _, i, n in r['toc'] if re.match(rf'(?:Task\s+)?{re.escape(t["id"])}\b', n)), None)
            if t['anchor']:
                r['html'] = re.sub(rf'(<h\d id="{re.escape(t["anchor"])}">.*?)(</h\d>)',
                                   lambda m: f'{m.group(1)} <span class="st {t["status"]}">{t["status"]}</span>{m.group(2)}', r['html'], count=1)
        return item['out']
    done = [t for t in s['tasks'] if t['result']]
    if not done:
        return None
    rel = f'tasks/{s["dir"].name}/index.html'; copy_assets(s['dir'], OUT / 'tasks' / s['dir'].name); body = ''
    for t in done:
        t['anchor'] = 'task-' + t['id'].replace('.', '-'); res = t['result']
        raw = res.get('checks') or {}
        raw = raw if isinstance(raw, dict) else {str(i): c for i, c in enumerate(raw)}
        # stage agents write checks either as {"value", "rule", "passed"} or as a bare boolean/number
        norm = {k: c if isinstance(c, dict) else {'value': c, 'passed': c is True} for k, c in raw.items()}
        checks = ''.join(f'<tr><td><code>{esc(k)}</code></td><td class="n">{fmt(c.get("value"))}</td><td>{esc(str(c.get("rule", "")))}</td>'
                         f'<td class="{"good" if c.get("passed") else "bad"}">{"pass" if c.get("passed") else "fail"}</td></tr>' for k, c in norm.items())
        figs = ''.join(f'<img loading="lazy" src="{o}" alt="Result figure of task {t["id"]}, {esc(t["name"])}">' for o in res.get('outputs', [])
                       if o.lower().endswith(MEDIA[:4]) and (s['dir'] / o).is_file())
        body += (f'<h2 id="{t["anchor"]}">{t["id"]} {esc(t["name"])} <span class="st {t["status"]}">{t["status"]}</span></h2>'
                 + (f'<div class="table-wrap"><table><thead><tr><th>Check</th><th class="n">Measured</th><th>Rule</th><th>Result</th></tr></thead><tbody>{checks}</tbody></table></div>' if checks else '')
                 + figs + (f'<p class="source">Run with <code>{esc(res["command"])}</code> in <code>tasks/{s["dir"].name}</code>.</p>' if res.get('command') else ''))
    article = (f'<p class="where">Tasks · Stage {s["n"]}</p><h1>{esc(s["title"])}</h1><p>The write-up of this stage is not published yet. '
               f'These are the acceptance results recorded so far, read from <a href="{TREE}tasks/{s["dir"].name}/results">tasks/{s["dir"].name}/results</a>.</p>{body}')
    write_page(rel, f'Stage {s["n"]}. {s["title"]} · go2-nav-sim', doc(f'Stage {s["n"]}', stage_side(s), article), 'Tasks')
    return rel


def fmt(v):
    return f'{v:.4g}' if isinstance(v, float) else esc(str(v))


def stage_side(s):
    return '<ol class="parts"><li class="open"><a class="part" href="../index.html">All tasks</a><ol>' + ''.join(
        f'<li><a href="#{t["anchor"]}"><span class="n">{t["id"]}</span><span>{esc(t["name"])}</span></a></li>' for t in s['tasks'] if t.get('anchor')) + '</ol></li></ol>'


def build_tasks(stages, items):
    body, side, pages = '', '', {}
    for s in stages:
        page = stage_page(s, items.get(f's{s["n"]:02d}'))
        if page:
            pages[f'tasks/{s["dir"].name}'] = dict(out=page, name=s['title'], label=s['title'])
        counts = ' · '.join(f'{sum(t["status"] == k for t in s["tasks"])} {k}' for k in ('verified', 'failed', 'spec'))
        rows = ''
        for t in s['tasks']:
            name = f'<a href="{url(page + "#" + t["anchor"], "tasks/index.html")}">{esc(t["name"])}</a>' if page and t.get('anchor') else esc(t['name'])
            rows += f'<tr><td class="id">{t["id"]}</td><td>{name}</td><td class="muted">{esc(t["shows"])}</td><td class="st {t["status"]}">{t["status"]}</td></tr>'
        link = f' · <a href="{url(page, "tasks/index.html")}">open the stage</a>' if page else ''
        body += (f'<h2 id="stage-{s["n"]}">Stage {s["n"]}. {esc(s["title"])}</h2><p class="meta">{counts}{link}</p>'
                 f'<div class="table-wrap"><table class="tasks"><thead><tr><th>Id</th><th>Task</th><th>Shows</th><th>Status</th></tr></thead><tbody>{rows}</tbody></table></div>')
        side += f'<li><a href="#stage-{s["n"]}"><span class="n">{s["n"]}</span><span>{esc(s["title"])}</span></a></li>'
    article = ('<h1>Tasks</h1><p>Every practical task of the course: one capability each, with an acceptance test. A task is '
               '<span class="st verified">verified</span> when its test ran and passed, <span class="st failed">failed</span> when it ran and '
               'did not pass, and <span class="st spec">spec</span> when it is not built yet.</p>' + body)
    write_page('tasks/index.html', 'Tasks · go2-nav-sim', doc('Stages', f'<ol class="parts"><li class="open"><ol>{side}</ol></li></ol>', article),
               'Tasks', 'Every practical task of the Go2 course with its acceptance status.')
    return pages


# --- deep dives ------------------------------------------------------------------------------------------------------------
def build_deep_dives(mods):
    """modules/index.html: the roadmap's modules in dependency order, each linking to its page when the module is written."""
    r = json.loads((SITE / 'data/roadmap.json').read_text()); have = {d.name[1:3]: (d, it) for d, it in mods}
    entries = r['modules'] + [dict(id=k, title=it['name'], status='', levels=[], prereq=[], here='', next='', refs=[]) for k, (d, it) in have.items()
                              if k not in {m['id'] for m in r['modules']}]
    rows = ''
    for m in entries:
        d, it = have.get(m['id'], (None, None))
        ls = m.get('level_status', ['planned'] * 3)
        levels = ''.join(f'<li><span class="muted">{k}</span> {esc(v)} <span class="st {s}">{s}</span></li>' for k, v, s in zip(('Fundamental', 'Intermediate', 'Advanced'), m['levels'], ls))
        chs = ' · '.join(f'<a href="{url(ROUTES[f"docs/{c}.md"], "modules/index.html")}">chapter {int(c[:2])}</a>' for c in m.get('chapters') or [] if f'docs/{c}.md' in ROUTES)
        pre = ', '.join(f'<a href="#m{p}">{p}</a>' for p in m['prereq']) or 'none'
        read = f'<a class="read" href="{d.name}/index.html">Read</a>' if d else '<span></span>'
        dl = ''.join(f'<dt>{k}</dt><dd>{v}</dd>' for k, v in (('Needs', pre), ('In this repo', esc(m['here']) + (f'<br>{chs}' if chs else '')),
                                                              ('Next', esc(m['next']))) if v)
        refs = ''.join(f'<li>{esc(x)}</li>' for x in m['refs'])
        rows += (f'<li id="m{m["id"]}"><details><summary><span class="n">{m["id"]}</span><span class="t">{esc(m["title"])}</span>'
                 f'<span class="st {m["status"]}">{m["status"]}</span></summary><div class="body"><ul class="levels">{levels}</ul><dl>{dl}</dl>'
                 f'{"<p class=muted>References</p><ul class=refs>" + refs + "</ul>" if refs else ""}</div></details>{read}</li>')
    legend = ''.join(f'<li><span class="st {k}">{k}</span>: {esc(v)}</li>' for k, v in r['status_legend'].items())
    article = (f'<h1>Deep dives</h1><p>Each module derives one piece of a legged robot from first principles and checks it with code and '
               f'measurements, on the Go2 in MuJoCo. No chapter depends on them. Modules are listed in dependency order.</p>'
               f'<ul class="legend-list">{legend}</ul><ol class="modlist">{rows}</ol>')
    write_page('modules/index.html', 'Deep dives · go2-nav-sim', f'<div class="wrap narrow"><article class="prose">{article}</article></div>',
               'Deep dives', 'First-principles modules on the Unitree Go2: maths, anatomy, kinematics, state estimation.')
    side = '<ol class="parts"><li class="open"><a class="part" href="../index.html">All deep dives</a><ol>{}</ol></li></ol>'
    for d, it in mods:
        r_ = it['rendered']
        lis = ''.join(f'<li><a href="../{d2.name}/index.html"{CUR if d2 is d else ""}><span class="n">{d2.name[1:3]}</span><span>{esc(it2["label"])}</span></a>'
                      + (sections(r_['toc']) if d2 is d else '') + '</li>' for d2, it2 in mods)
        article = (f'<p class="where">Deep dive · module {d.name[1:3]}</p><h1>{esc(it["name"])}</h1>{r_["html"]}'
                   f'<p class="source">Source: <a href="{BLOB}modules/{d.name}/README.md">modules/{d.name}/README.md</a></p>')
        write_page(it['out'], f'{it["name"]} · go2-nav-sim', doc(it['label'], side.format(lis), article), 'Deep dives', it['name'], KATEX if r_['math'] else '')


# --- reference -------------------------------------------------------------------------------------------------------------
def table(cols, rows, code_cols=0):
    return ('<div class="table-wrap"><table><thead><tr>' + ''.join(f'<th>{c}</th>' for c in cols) + '</tr></thead><tbody>'
            + ''.join('<tr>' + ''.join(f'<td><code>{esc(str(v))}</code></td>' if i < code_cols else f'<td>{esc(str(v))}</td>' for i, v in enumerate(r)) + '</tr>' for r in rows)
            + '</tbody></table></div>')


def parameter_tables():
    """Parameters read from the files the robot actually uses."""
    cfg = ROOT / 'src/go2_nav_sim/config'; out = []
    lo = (ROOT / 'src/go2_nav_sim/scripts/lidar_odom.py').read_text()
    out.append(('lidar_odom.py', 'src/go2_nav_sim/scripts/lidar_odom.py', ['Parameter', 'Default', 'Note'],
                [(n, d, (c or '').strip(' #')) for n, d, c in re.findall(r"P\('(\w+)', ([^)]+)\)\.value[^\n#]*(#[^\n]*)?", lo)]))
    s = yaml.safe_load((cfg / 'scan.yaml').read_text())['pointcloud_to_laserscan']['ros__parameters']
    out.append(('pointcloud_to_laserscan', 'src/go2_nav_sim/config/scan.yaml', ['Parameter', 'Value'], list(s.items())))
    e = yaml.safe_load((cfg / 'ekf.yaml').read_text())['odom_ekf']['ros__parameters']
    out.append(('EKF (robot_localization)', 'src/go2_nav_sim/config/ekf.yaml', ['Parameter', 'Value'], list(e.items())))
    n = yaml.safe_load((cfg / 'nav2_params.yaml').read_text())
    fp = n['controller_server']['ros__parameters']['FollowPath']; lc = n['local_costmap']['local_costmap']['ros__parameters']
    vs = n['velocity_smoother']['ros__parameters']; am = n['amcl']['ros__parameters']; gb = n['planner_server']['ros__parameters']['GridBased']
    out.append(('Nav2, changes from the Humble defaults', 'src/go2_nav_sim/scripts/make_nav2_params.py', ['Parameter', 'Value'], [
        ('controller', fp['plugin'].split('::')[-1]), ('desired_linear_vel', fp['desired_linear_vel']), ('lookahead_dist', fp['lookahead_dist']),
        ('use_rotate_to_heading', fp['use_rotate_to_heading']), ('robot_radius', lc['robot_radius']),
        ('inflation_radius', lc['inflation_layer']['inflation_radius']), ('cost_scaling_factor', lc['inflation_layer']['cost_scaling_factor']),
        ('planner', gb['plugin']), ('use_astar', gb['use_astar']), ('velocity_smoother max_velocity', vs['max_velocity']),
        ('amcl laser_max_range', am['laser_max_range']), ('amcl max_beams', am['max_beams'])]))
    return ''.join(f'<h3>{esc(t)}</h3><p class="source">From <a href="{BLOB}{src}">{src}</a>.</p>{table(cols, rows, 2)}' for t, src, cols, rows in out)


COMMANDS = [('scripts/setup.sh', 'fetch upstream at the pinned commit, apply the patch, plugin overlay if needed, build'),
            ('source env.sh', 'ROS 2, workspace, Cyclone DDS on loopback, plugin paths, GPU offload, clean PATH'),
            ('scripts/up.sh', 'sim, estimation, then Nav2, with readiness checks; retries the Nav2 bring-up'),
            ('scripts/stop.sh [nav]', 'stop everything (or only Nav2), escalating to SIGKILL'),
            ('scripts/run_mission.sh bags/run1', 'fresh stack, record a bag, drive A to B, stop recording'),
            ('python3 src/go2_nav_sim/scripts/go_to_goal.py [x y yaw]', 'localise AMCL, send a goal, report the result against ground truth'),
            ('python3 src/go2_nav_sim/scripts/bag_metrics.py BAG', 'odometry ATE, drift, AMCL error, goal error'),
            ('python3 src/go2_nav_sim/scripts/render_dashboard.py BAG OUT.mp4 --cards', '1080p dashboard video from a bag'),
            ('python3 src/go2_nav_sim/scripts/export_run.py BAG DIR', 'compact JSON, map and snapshots for this site'),
            ('python3 -m pytest -q src/go2_nav_sim/test', 'unit tests, no ROS needed'),
            ('python3 site/build.py', 'build this site into _site/')]
TF_TREE = '''map                      AMCL (map -> odom)
└── odom                 EKF (odom -> base_footprint), from lidar odometry + gyro
    └── base_footprint   on the ground plane
        └── base_link    CHAMP base_to_footprint: here a constant (0, 0, 0.426), no tilt (real height ~0.217 m)
            ├── trunk, legs          robot_state_publisher
            ├── lidar_link           +0.10 m z
            ├── camera_link          +0.28 m x, +0.03 m z
            │   └── camera_optical   rpy (-pi/2, 0, -pi/2)
            └── chase_cam            video only
slam_map                 slam_toolbox, separate tree; never connected to map
world                    ground truth (/odom_gt), evaluation only'''


def build_reference(trouble):
    a = json.loads((SITE / 'data/architecture.json').read_text())
    topics = [(t['name'], t['type'], t['rate'], t['frame'], n['label']) for n in a['nodes'] for t in n.get('topics', [])]
    tb = trouble['html'] if trouble else ''
    tb = re.sub(r'<(/?)h([2-5])', lambda m: f'<{m.group(1)}h{int(m.group(2)) + 1}', tb)          # its sections sit under one h2 here
    secs = [('topics', 'Topics', table(['Topic', 'Type', 'Rate', 'Frame', 'Published by'], topics, 1)),
            ('frames', 'TF tree', f'<pre><code>{esc(TF_TREE)}</code></pre>'),
            ('parameters', 'Parameters', parameter_tables()),
            ('commands', 'Commands', table(['Command', 'What it does'], COMMANDS, 1))]
    if trouble:
        secs.append(('troubleshooting', 'Troubleshooting', tb + f'<p class="source">Source: <a href="{BLOB}docs/12-troubleshooting.md">docs/12-troubleshooting.md</a></p>'))
    side = '<ol class="parts"><li class="open"><ol>' + ''.join(f'<li><a href="#{i}"><span>{t}</span></a></li>' for i, t, _ in secs) + '</ol></li></ol>'
    article = ('<h1>Reference</h1><p>Parameter tables are read from the configuration files when the site is built.</p>'
               + ''.join(f'<h2 id="{i}">{t}</h2>{h}' for i, t, h in secs))
    write_page('reference.html', 'Reference · go2-nav-sim', doc('Reference', side, article), 'Reference',
               'Topics, frames, parameters, commands and troubleshooting for the go2-nav-sim stack.', KATEX if trouble and trouble['math'] else '')


# --- home ------------------------------------------------------------------------------------------------------------------
def build_home(parts, pages):
    stills, seen = [], set()
    for pat in STILLS:
        for f in sorted(ROOT.glob(pat)):
            owner = f.parents[1]
            if owner in seen or f'{owner.parent.name}/{owner.name}' not in pages or len(stills) == 3:
                continue
            seen.add(owner); page = pages[f'{owner.parent.name}/{owner.name}']
            stills.append(f'<a href="{page["out"]}"><img src="{f.relative_to(ROOT)}" alt="Figure from {esc(page["name"])}" loading="lazy">'
                          f'<span>{esc(page["label"])}</span></a>')
    path = ''.join(f'<li><a href="{items[0]["out"]}">{part}</a><span>{" · ".join(esc(it["label"]) for it in items)}</span></li>' for part, items in parts)
    body = (f'<div class="wrap home"><h1>Learn ROS 2 on a simulated Unitree Go2, from an empty terminal to a robot that walks, '
            f'stops for obstacles, builds a map and navigates on its own.</h1>'
            f'<p><a class="btn" href="{parts[0][1][0]["out"]}">Start with chapter 0</a></p>'
            f'<div class="stills">{"".join(stills)}</div>'
            f'<h2>The path</h2><ol class="path">{path}</ol></div>')
    write_page('index.html', 'go2-nav-sim · learn ROS 2 on a simulated Unitree Go2', body)


# --- main ------------------------------------------------------------------------------------------------------------------
def main():
    global OUT
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default=str(OUT)); OUT = pathlib.Path(ap.parse_args().out)
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    for src, dst in ((SITE / 'static', 'static'), (SITE / 'data', 'data'), (ROOT / 'media', 'media'), (ROOT / 'docs/img', 'img')):
        shutil.copytree(src, OUT / dst, ignore=shutil.ignore_patterns('dev', '*.py', '__pycache__'))

    stages = spec_stages(); parts = learn_path(stages)
    learn = [it for _, items in parts for it in items]
    mods = [(d, dict(src=d / 'README.md', dir=d, out=f'modules/{d.name}/index.html')) for d in sorted(ROOT.glob('modules/m[0-9][0-9]_*')) if (d / 'README.md').exists()]
    trouble_src = next(iter(sorted(ROOT.glob('docs/12-*.md'))), None)
    # routes first, so a page can link to any other page; then copy assets, so links to figures resolve
    for it in learn + [it for _, it in mods]:
        ROUTES[str(it['src'].relative_to(ROOT))] = it['out']
        if 'dir' in it:
            ROUTES[str(it['dir'].relative_to(ROOT))] = it['out']; copy_assets(it['dir'], OUT / pathlib.Path(it['out']).parent)
    for s in stages:
        if s['dir'] and f'tasks/{s["dir"].name}' not in ROUTES and any(t['result'] for t in s['tasks']):
            ROUTES[f'tasks/{s["dir"].name}'] = f'tasks/{s["dir"].name}/index.html'
    ROUTES['modules'] = ROUTES['modules/README.md'] = 'modules/index.html'
    if trouble_src:
        ROUTES[str(trouble_src.relative_to(ROOT))] = 'reference.html#troubleshooting'

    for it in learn + [it for _, it in mods]:
        it['rendered'] = render(it['src'], it['out'])
        if it['src'].parent.parent.name == 'modules':
            title = it['rendered']['title']; it['name'] = re.sub(r'^(?:Module\s+)?\d+\s*[.:·]?\s*', '', title) or it['dir'].name; it['label'] = it['name'].split(':')[0]
    stage_pages = build_tasks(stages, {it['key']: it for it in learn if it['key'][0] == 's'})      # adds task status to stage headings first
    build_learn(parts)
    build_deep_dives(mods)
    build_reference(render(trouble_src, 'reference.html') if trouble_src else None)
    build_home(parts, {**stage_pages, **{f'{it["dir"].parent.name}/{it["dir"].name}': it for it in learn if 'dir' in it}})

    # old URLs: every removed or merged page forwards to where its content went
    redirect('evidence.html', 'tutorial/11-record-and-visualise.html#measured-runs', keep_hash=False)
    redirect('explore.html', 'tutorial/08-robot-and-sensors.html#one-lidar-sweep-in-3d', keep_hash=False,
             hashes={'#architecture': 'tutorial/00-overview.html#the-stack-node-by-node'})
    redirect('roadmap.html', 'modules/index.html')
    if trouble_src:
        redirect(f'tutorial/{trouble_src.stem}.html', 'reference.html#troubleshooting')
    (OUT / '.nojekyll').write_text('')
    print(f'built {OUT}: {len(learn)} learn pages in {len(parts)} parts, {len(mods)} deep dives, '
          f'{sum(len(s["tasks"]) for s in stages)} tasks in {len(stages)} stages, 4 redirects')


if __name__ == '__main__':
    main()
