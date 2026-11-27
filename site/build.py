#!/usr/bin/env python3
"""Build the static site into _site/.   python3 site/build.py [--out _site]

Sources: docs/*.md (tutorial chapters, the same files GitHub shows), site/pages/*.html (hand-written pages), site/static,
site/data (exported runs), media/, docs/img/, results/*.json and site/evidence/*. Numbers shown on the site are read from the
result files at build time, so the site cannot disagree with the repository."""
import argparse, html, json, pathlib, re, shutil
import markdown
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]; SITE = ROOT / 'site'
CUR = ' aria-current="page"'
REPO = 'https://github.com/Armanasq/go2-nav-sim'; BLOB = REPO + '/blob/main/'
NAV = [('Roadmap', 'roadmap.html'), ('Tutorial', 'tutorial/00-overview.html'), ('Explore', 'explore.html'), ('Evidence', 'evidence.html'), ('Reference', 'reference.html')]
PARTS = [('Foundations', range(0, 8)), ('Tasks', range(8, 12)), ('Help', range(12, 100))]


def part_of(num):
    return next(name for name, r in PARTS if int(num or 0) in r)


def page(title, body, root, current='', desc='', extra_head=''):
    nav = ''.join(f'<a href="{root}{href}"{CUR if current == name else ""}>{name}</a>' for name, href in NAV) + f'<a class="ext" href="{REPO}">GitHub</a>'
    return f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>{html.escape(title)}</title>
<meta name="description" content="{html.escape(desc or 'Unitree Go2 navigation in Gazebo with ROS 2 Humble: sensors, lidar-inertial odometry, Nav2, and recorded evidence.')}">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<link rel="stylesheet" href="{root}static/css/base.css"><link rel="stylesheet" href="{root}static/css/site.css">
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'%3E%3Ccircle cx='8' cy='8' r='5' fill='%230b74a8'/%3E%3C/svg%3E">
<script type="importmap">{{"imports": {{"three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js", "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}}}</script>
{extra_head}
</head>
<body data-root="{root}">
<a class="skip visually-hidden" href="#main">Skip to content</a>
<header class="topbar"><div class="inner">
<a class="brand" href="{root}index.html">go2-nav-sim</a>
<nav aria-label="Main">{nav}</nav>
<button class="theme-toggle" type="button" aria-label="Colour theme: auto">auto</button>
</div></header>
<main id="main">
{body}
</main>
<footer class="site"><div class="wrap">Built from <a href="{REPO}">{REPO.split('//')[1]}</a>. Simulation only. Ground truth initialises AMCL once and is otherwise used only for scoring.</div></footer>
<script type="module" src="{root}static/js/site.js"></script>
</body>
</html>
'''


# ---------------------------------------------------------------------------------------------------------------------------
def chapters():
    out = []
    for p in sorted((ROOT / 'docs').glob('[0-9][0-9]-*.md')):
        text = p.read_text(); title = re.match(r'#\s+(.*)', text).group(1)
        num, name = title.split('. ', 1) if '. ' in title else ('', title)
        first = next((l for l in text.split('\n\n')[1:] if l and not l.startswith(('#', '|', '```', '-', '!'))), '')
        out.append(dict(slug=p.stem, title=title, num=num, name=name, md=text, summary=re.sub(r'[`*\[\]]|\(.*?\)', '', first).strip()))
    return out


def rewrite_links(h, root):
    h = re.sub(r'href="(\d\d-[\w-]+)\.md(#[\w-]+)?"', lambda m: f'href="{m.group(1)}.html{m.group(2) or ""}"', h)
    h = re.sub(r'src="img/', f'src="{root}img/', h)
    h = re.sub(r'href="\.\./((?:src|scripts|patches|results|media|config)[^"]*)"', lambda m: f'href="{BLOB}{m.group(1)}"', h)
    h = re.sub(r'<table>', '<div class="table-wrap"><table>', h).replace('</table>', '</table></div>')
    return h


def build_chapters(out):
    chs = chapters(); (out / 'tutorial').mkdir(parents=True, exist_ok=True)
    for i, c in enumerate(chs):
        md = markdown.Markdown(extensions=['tables', 'fenced_code', 'toc', 'sane_lists'])
        body_md = re.sub(r'\n+Next: \[.*?\]\(.*?\)\s*$', '\n', c['md'].split('\n', 1)[1])          # title and "Next:" handled by the layout
        content = rewrite_links(md.convert(body_md), '../')
        toc = ''.join(f'<li><a href="#{t["id"]}">{html.escape(t["name"])}</a></li>' for t in md.toc_tokens if t['level'] == 2)
        side = ''
        for pname, rng in PARTS:
            items = ''.join(f'<li><a href="{x["slug"]}.html"{CUR if x is c else ""}><span class="num">{x["num"]}</span>{html.escape(x["name"])}</a></li>' for x in chs if int(x['num'] or 0) in rng)
            side += f'<li class="part"><p class="part-name">{pname}</p><ol>{items}</ol></li>' if items else ''
        prev = f'<a class="pn prev" href="{chs[i-1]["slug"]}.html"><span class="muted">Previous</span>{html.escape(chs[i-1]["title"])}</a>' if i else '<span></span>'
        nxt = f'<a class="pn next" href="{chs[i+1]["slug"]}.html"><span class="muted">Next</span>{html.escape(chs[i+1]["title"])}</a>' if i + 1 < len(chs) else '<span></span>'
        body = f'''<div class="wrap chapter">
<details class="chapters" open><summary><span class="muted">Chapters</span><span>{c["num"]}. {html.escape(c["name"])}</span></summary>
<nav aria-label="Chapters"><ol class="parts">{side}</ol></nav></details>
<article class="prose-col">
<p class="where">{part_of(c["num"])} · Step {c["num"]} of {chs[-1]["num"]}</p><h1>{html.escape(c["name"])}</h1>
{content}
<nav class="prevnext" aria-label="Chapter">{prev}{nxt}</nav>
<p class="source">Source: <a href="{BLOB}docs/{c["slug"]}.md">docs/{c["slug"]}.md</a></p>
</article>
<aside class="onpage" aria-label="On this page">{"<p class=onpage-title>On this page</p><ul>" + toc + "</ul>" if toc else ""}</aside>
</div>'''
        (out / 'tutorial' / f'{c["slug"]}.html').write_text(page(f'{c["title"]} · go2-nav-sim', body, '../', 'Tutorial', c['summary'][:150]))
    return chs


# ---------------------------------------------------------------------------------------------------------------------------
def runs():
    rs = []
    for p in sorted((ROOT / 'results').glob('run_*.json')):
        rs.append(dict(id=p.stem.replace('run_', ''), **json.loads(p.read_text())))
    return rs


def fmt_range(vals, unit, nd=1):
    lo, hi = min(vals), max(vals)
    return f'{lo:.{nd}f}{unit}' if abs(hi - lo) < 10 ** -nd else f'{lo:.{nd}f}–{hi:.{nd}f}{unit}'


def kpis(rs, missions):
    ok = sum(1 for m in missions if m['result'] == 'SUCCEEDED')
    cells = [(f'{ok}/{len(missions)}', 'consecutive missions reached the goal'),
             (fmt_range([r['odom_ate_aligned_cm'] for r in rs], ' cm'), 'odometry ATE, SE(2)-aligned RMSE'),
             (fmt_range([r['amcl_rmse_cm'] for r in rs], ' cm'), 'AMCL error at its updates (RMSE)'),
             (fmt_range([r['goal_error_m'] for r in rs], ' m', 2), 'end distance to the goal (Nav2 tolerance 0.25 m)')]
    return '<div class="kpis">' + ''.join(f'<div class="kpi"><div class="v">{v}</div><div class="k">{k}</div></div>' for v, k in cells) + '</div>'


def missions():
    """RESULT lines captured from scripts/run_mission.sh, stored in site/evidence/missions.json by tools/collect_evidence.py."""
    p = SITE / 'evidence/missions.json'
    return json.loads(p.read_text()) if p.exists() else []


def runs_table(rs, ms):
    byid = {m['run']: m for m in ms}; rows = ''
    for r in rs:
        m = byid.get(r['id'], {})
        res = f'<span class="result good">succeeded</span>' if m.get('result') == 'SUCCEEDED' else f'<span class="result bad">{html.escape(m.get("result", "?").lower())}</span>'
        rows += (f'<tr><td>{r["id"]}</td><td>{res}</td><td class="n">{m.get("sim_time_s", "")}</td><td class="n">{r["path_m"]}</td>'
                 f'<td class="n">{r["goal_error_m"]}</td><td class="n">{r["odom_ate_aligned_cm"]}</td><td class="n">{r["odom_ate_anchored_cm"]}</td><td class="n">{r["odom_rpe_1m_pct"]}</td>'
                 f'<td class="n">{r["amcl_rmse_cm"]}</td><td class="n">{r["amcl_max_cm"]}</td>'
                 f'<td><a href="data/runs/{r["id"]}/run.json">run.json</a> · <a href="{BLOB}results/run_{r["id"]}.json">metrics</a></td></tr>')
    return ('<div class="table-wrap"><table><thead><tr><th>Run</th><th>Nav2 result</th><th class="n">Time to goal, s</th><th class="n">Path, m</th>'
            '<th class="n">End distance to B, m</th><th class="n">Odom ATE aligned, cm</th><th class="n">Odom ATE anchored, cm</th><th class="n">RPE per 1 m, %</th><th class="n">AMCL RMSE, cm</th>'
            '<th class="n">AMCL max, cm</th><th>Data</th></tr></thead><tbody>' + rows + '</tbody></table></div>')


def reference_tables():
    """Parameters read from the files the robot actually uses."""
    cfg = ROOT / 'src/go2_nav_sim/config'; out = []
    lo = (ROOT / 'src/go2_nav_sim/scripts/lidar_odom.py').read_text()
    rows = [(n, d, c.strip(' #')) for n, d, c in re.findall(r"P\('(\w+)', ([^)]+)\)\.value[^\n#]*(#[^\n]*)?", lo)]
    out.append(('lidar_odom.py parameters', 'src/go2_nav_sim/scripts/lidar_odom.py', ['parameter', 'default', 'note'], rows))
    s = yaml.safe_load((cfg / 'scan.yaml').read_text())['pointcloud_to_laserscan']['ros__parameters']
    out.append(('pointcloud_to_laserscan', 'src/go2_nav_sim/config/scan.yaml', ['parameter', 'value'], [(k, v) for k, v in s.items()]))
    e = yaml.safe_load((cfg / 'ekf.yaml').read_text())['odom_ekf']['ros__parameters']
    out.append(('EKF (robot_localization)', 'src/go2_nav_sim/config/ekf.yaml', ['parameter', 'value'], [(k, v) for k, v in e.items()]))
    n = yaml.safe_load((cfg / 'nav2_params.yaml').read_text())
    fp = n['controller_server']['ros__parameters']['FollowPath']; lc = n['local_costmap']['local_costmap']['ros__parameters']
    vs = n['velocity_smoother']['ros__parameters']; am = n['amcl']['ros__parameters']
    rows = [('controller', fp['plugin'].split('::')[-1]), ('desired_linear_vel', fp['desired_linear_vel']), ('lookahead_dist', fp['lookahead_dist']),
            ('use_rotate_to_heading', fp['use_rotate_to_heading']), ('robot_radius', lc['robot_radius']),
            ('inflation_radius', lc['inflation_layer']['inflation_radius']), ('cost_scaling_factor', lc['inflation_layer']['cost_scaling_factor']),
            ('planner', n['planner_server']['ros__parameters']['GridBased']['plugin']), ('use_astar', n['planner_server']['ros__parameters']['GridBased']['use_astar']),
            ('velocity_smoother max_velocity', vs['max_velocity']), ('amcl laser_max_range', am['laser_max_range']), ('amcl max_beams', am['max_beams'])]
    out.append(('Nav2 (deltas from Humble defaults)', 'src/go2_nav_sim/scripts/make_nav2_params.py', ['parameter', 'value'], rows))
    h = ''
    for title, src, cols, rows in out:
        h += f'<h3>{html.escape(title)}</h3><p class="muted small">Read from <a href="{BLOB}{src}">{src}</a> at build time.</p><div class="table-wrap"><table><thead><tr>'
        h += ''.join(f'<th>{c}</th>' for c in cols) + '</tr></thead><tbody>'
        h += ''.join('<tr>' + ''.join(f'<td><code>{html.escape(str(v))}</code></td>' if i < 2 else f'<td>{html.escape(str(v))}</td>' for i, v in enumerate(r)) + '</tr>' for r in rows)
        h += '</tbody></table></div>'
    return h


def topics_table():
    a = json.loads((SITE / 'data/architecture.json').read_text()); rows = ''
    for n in a['nodes']:
        for t in n.get('topics', []):
            rows += f'<tr><td><code>{t["name"]}</code></td><td>{html.escape(t["type"])}</td><td>{html.escape(t["rate"])}</td><td><code>{html.escape(t["frame"])}</code></td><td>{html.escape(n["label"])}</td></tr>'
    return '<div class="table-wrap"><table><thead><tr><th>Topic</th><th>Type</th><th>Rate</th><th>Frame</th><th>Published by</th></tr></thead><tbody>' + rows + '</tbody></table></div>'


def roadmap_html(chs):
    r = json.loads((SITE / 'data/roadmap.json').read_text()); slugs = {c['slug']: c for c in chs}
    count = {s: sum(m['status'] == s for m in r['modules']) for s in r['status_legend']}
    h = f'<p class="lead">{html.escape(r["intro"])}</p>'
    h += '<ul class="status-legend">' + ''.join(f'<li><span class="status {s}">{s}</span><span class="count">{count[s]} of {len(r["modules"])}</span><span class="desc">{html.escape(d)}</span></li>' for s, d in r['status_legend'].items()) + '</ul>'
    h += '<h2 id="order">Order of work</h2><ol class="stages">' + ''.join(
        f'<li><span class="num">{st["n"]}</span><span>{html.escape(st["title"])}</span><span class="mods">' +
        ' '.join(f'<a href="#m{m}" aria-label="Module {m}">{m}</a>' for m in st['modules']) + '</span></li>' for st in r['stages']) + '</ol>'
    h += f'<h2 id="modules">Modules</h2><p class="prose muted">Every module is written to the same template: {", ".join(r["module_template"]).lower()}.</p><div class="modules">'
    for m in r['modules']:
        pre = ', '.join(f'<a href="#m{p}">{p}</a>' for p in m['prereq']) or 'none'
        ch = ' · '.join(f'<a href="tutorial/{s}.html">{html.escape(slugs[s]["title"])}</a>' for s in m.get('chapters', []) if s in slugs)
        ev = f'<a href="{m["evidence"]}">evidence</a>' if m.get('evidence') else ''
        ls = m.get('level_status', ['planned'] * 3)
        levels = ''.join(f'<div class="lvl-{s}"><span class="lvl">{k} · <span class="status {s}">{s}</span></span>{html.escape(v)}</div>' for k, v, s in zip(('Fundamental', 'Intermediate', 'Advanced'), m['levels'], ls))
        refs = ''.join(f'<li>{html.escape(x)}</li>' for x in m['refs'])
        h += (f'<details class="module" id="m{m["id"]}"><summary><span class="num">{m["id"]}</span><span class="t">{html.escape(m["title"])}</span>'
              f'<span class="status {m["status"]}">{m["status"]}</span></summary><div class="body">'
              f'<div class="levels">{levels}</div><dl><dt>Needs</dt><dd>{pre}</dd><dt>In this repo</dt><dd>{html.escape(m["here"])}'
              f'{"<br>" + ch if ch else ""}{" · " + ev if ev else ""}</dd>{"<dt>Current practice</dt><dd>" + html.escape(m["current"]) + "</dd>" if m.get("current") else ""}<dt>Next</dt><dd>{html.escape(m["next"])}</dd></dl>'
              f'{"<p class=refs-title>References</p><ul class=refs>" + refs + "</ul>" if refs else ""}</div></details>')
    return h + '</div>'


def evidence_block(name):
    p = SITE / 'evidence' / name
    return html.escape(p.read_text()) if p.exists() else '(not collected yet: run tools/collect_evidence.py)'


# ---------------------------------------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--out', default=str(ROOT / '_site')); out = pathlib.Path(ap.parse_args().out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    for src, dst in ((SITE / 'static', 'static'), (SITE / 'data', 'data'), (ROOT / 'media', 'media'), (ROOT / 'docs/img', 'img')):
        shutil.copytree(src, out / dst, ignore=shutil.ignore_patterns('dev', '*.py'))
    if (SITE / 'evidence').exists():
        shutil.copytree(SITE / 'evidence', out / 'evidence')
    chs = build_chapters(out); rs = runs(); ms = missions()
    steps = ''
    for pname, rng in PARTS:
        rows = ''.join(f'<li><a href="tutorial/{c["slug"]}.html"><span class="num">{c["num"]}</span><strong>{html.escape(c["name"])}</strong></a></li>' for c in chs if int(c['num'] or 0) in rng)
        steps += f'<div class="part"><h3 class="part-name">{pname}</h3><ol class="steps">{rows}</ol></div>' if rows else ''
    subs = {'{{KPIS}}': kpis(rs, ms) if rs and ms else '', '{{STEPS}}': steps, '{{RUNS_TABLE}}': runs_table(rs, ms), '{{REFERENCE}}': reference_tables(),
            '{{TOPICS}}': topics_table(), '{{PYTEST}}': evidence_block('pytest.txt'), '{{MISSIONS}}': evidence_block('missions.txt'),
            '{{SITECHECK}}': evidence_block('sitecheck.txt'), '{{SETUP}}': evidence_block('setup.txt'), '{{REPO}}': REPO, '{{BLOB}}': BLOB, '{{ROADMAP}}': roadmap_html(chs)}
    for p in sorted((SITE / 'pages').glob('*.html')):
        text = p.read_text(); m = re.match(r'<!--\s*title:\s*(.*?)\s*\|\s*nav:\s*(.*?)\s*\|\s*desc:\s*(.*?)\s*-->\n', text)
        title, nav, desc = m.groups(); body = text[m.end():]
        for k, v in subs.items():
            body = body.replace(k, v)
        (out / p.name).write_text(page(title, body, '', nav, desc))
    (out / '.nojekyll').write_text('')
    print(f'built {out}: {len(chs)} chapters, {len(list((SITE / "pages").glob("*.html")))} pages, {len(rs)} runs')


if __name__ == '__main__':
    main()
