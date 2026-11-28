#!/usr/bin/env python3
"""Stage 10 acceptance tests. Each task starts the stack it needs (up.sh, a fresh stack per run) and stops it at the end.
usage: python3 run.py <task-id>|all          e.g. python3 run.py 10.04
Writes results/<id>.json, and per run results/runs/<id>_<k>.json + .csv (true track). Thresholds: acceptance.json.
With VIDEO=1 the first run of 10.08 is also recorded from the spectator camera to media/ (wrap it in flock /tmp/go2_render.lock)."""
import json, math, os, pathlib, re, signal, subprocess, sys, time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE / 'worlds'))
import make_worlds as mw

ACC = json.loads((HERE / 'acceptance.json').read_text())
RES, RUNS, LOGS = HERE / 'results', HERE / 'results/runs', HERE / 'logs'
PY = '/usr/bin/python3'
ARENA = (str(ROOT / 'src/go2_nav_sim/worlds/arena.world'), str(ROOT / 'src/go2_nav_sim/maps/arena.yaml'))
WAYPOINTS = ['-4.5,-0.8,0.0', '-0.4,-3.8,1.5708', ','.join(map(str, mw.WORLDS['obstacles']['goal']))]
P11, P13 = ROOT / 'projects/p11_command-arbitration', ROOT / 'projects/p13_behaviour-trees'
TASKS = {}


def task(*ids):
    def wrap(f):
        for i in ids:
            TASKS[i] = f
        return f
    return wrap


def sh(cmd, timeout=600, env=None):
    p = subprocess.run(cmd, shell=True, cwd=ROOT, capture_output=True, text=True, timeout=timeout,
                       env={**os.environ, **(env or {})}, executable='/bin/bash')
    return p.returncode, p.stdout + p.stderr


def up(world, map_=None, nav=True):
    for attempt in range(2):
        rc, out = sh(f'bash {HERE}/up.sh {world} {map_ or ""}', env=None if nav else {'NAV': '0'})
        if rc == 0:
            return
        print(out[-400:])
    sys.exit(f'stack did not come up on {world}')


def stop():
    sh('bash scripts/stop.sh')


def mission(tid, k, mode, world, *args):
    out = RUNS / f'{tid}_{k}.json'
    rc, txt = sh(f'{PY} {HERE}/nav.py {mode} {" ".join(map(str, args))} --world {world} --out {out}', timeout=900)
    print(f'  {tid} run {k}: ' + ' | '.join(l for l in txt.splitlines() if l.startswith('leg ')))
    if not out.exists():
        print(txt[-1500:])
        return None
    return json.loads(out.read_text())


def ok(leg):
    return leg['result'] == 'SUCCEEDED' and leg['final_error_m'] <= ACC['_common']['final_error_m']


def summary(runs, key='legs'):
    """Per-run rows and the aggregate numbers the checks use."""
    legs = [l for r in runs if r for l in r[key]]
    rows = [{k: l.get(k) for k in ('status_name', 'sim_time_s', 'final_error_m', 'path_m', 'straight_m', 'min_clearance_m',
                                   'amcl_rmse_m')} for l in legs]
    return dict(runs=rows, n_runs=len(runs),
                success_rate=round(sum(bool(r) and all(ok(l) for l in r[key]) for r in runs) / len(runs), 3),
                max_final_error_m=max(l['final_error_m'] for l in legs) if legs else None,
                min_clearance_m=min(l['min_clearance_m'] for l in legs) if legs else None,
                mean_sim_time_s=round(sum(l['sim_time_s'] for l in legs) / len(legs), 1) if legs else None)


def judge(tid, m, command):
    checks, verdicts = ACC[tid]['checks'], {}
    for k, c in checks.items():
        v = m.get(k)
        verdicts[k] = v is not None and (v <= c['max'] if 'max' in c else v >= c['min'] if 'min' in c else v == c['equals'])
    text = '; '.join(f"{k} {'<=' if 'max' in c else '>=' if 'min' in c else '=='} {c.get('max', c.get('min', c.get('equals')))}"
                     for k, c in checks.items())
    status = 'verified' if all(verdicts.values()) else 'failed'
    out = dict(id=tid, status=status, metrics=m, acceptance=text, checks=verdicts, command=command)
    (RES / f'{tid}.json').write_text(json.dumps(out, indent=1))
    print(f'{tid} {status}: ' + ', '.join(f'{k}={m.get(k)}' for k in checks))
    return status


def keep_warnings(tid, k):
    """Nav2's warnings and errors of a run (nav.log is overwritten by the next up.sh)."""
    lines = [l for l in (LOGS / 'nav.log').read_text(errors='replace').splitlines() if '[WARN]' in l or '[ERROR]' in l]
    (RUNS / f'{tid}_{k}_nav_warnings.log').write_text('\n'.join(lines) + '\n')


def runs_of(tid, mode, world, args, map_=None):
    runs = []
    for k in range(ACC[tid]['runs']):
        up(world, map_)
        runs.append(mission(tid, k, mode, world, *args))
        keep_warnings(tid, k)
        stop()
    return runs


@task('10.01', '10.02', '10.03', '10.04', '10.05', '10.06', '10.07', '10.15')
def a_to_b(tid):
    world, map_ = ACC[tid]['world'], ACC[tid].get('map')
    goal = mw.WORLDS[world]['goal']
    runs = runs_of(tid, 'goto', world, goal, map_)
    return judge(tid, summary(runs), f'python3 run.py {tid}  (up.sh {world} {map_ or ""}; nav.py goto {" ".join(map(str, goal))}) x {len(runs)}')


@task('10.08')
def replan(tid):
    runs = []
    for k in range(ACC[tid]['runs']):
        up('room')
        rec = None
        if k == 0 and os.environ.get('VIDEO') == '1':
            raw = LOGS / 'replan_raw.mp4'
            rec = subprocess.Popen([PY, str(ROOT / 'projects/common/record_cam.py'), str(raw), '--topic', '/spectator/cam/image_raw'],
                                   cwd=ROOT, start_new_session=True)
        runs.append(mission(tid, k, 'replan', 'room'))
        if rec:
            rec.send_signal(signal.SIGINT); rec.wait(timeout=30)
            speedup(raw, HERE / 'media/10.08_replan.mp4')
        stop()
    m = summary(runs)
    boxes = [r['box'] for r in runs if r and 'box' in r]
    m.update(boxes=boxes, all_plans_blocked_at_spawn=len(boxes) == len(runs) and all(b['plan_blocked_at_spawn'] for b in boxes),
             max_replan_s=max((b['replan_s'] for b in boxes), default=None) if all(b['replan_s'] is not None for b in boxes) else None,
             min_box_clearance_m=min((b['min_box_clearance_m'] for b in boxes if b['min_box_clearance_m'] is not None), default=None))
    return judge(tid, m, f'python3 run.py {tid}  (up.sh room; nav.py replan) x {len(runs)}')


def speedup(raw, out, max_s=18.0):
    """record_cam.py writes real-time (simulated) video; speed it up to at most max_s seconds, H.264."""
    d = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', str(raw)],
                             capture_output=True, text=True).stdout or 0)
    k = max(1.0, math.ceil(d / max_s * 2) / 2)
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', str(raw), '-vf', f'setpts=PTS/{k},scale=960:-2', '-r', '20', '-an',
                    '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '28', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)],
                   check=True)
    print(f'  video {out.name}: {d:.0f} s at x{k}')


@task('10.09')
def reject(tid):
    w = mw.WORLDS['one_obstacle']; bx, by = w['blocks'][0][:2]
    up('one_obstacle')
    r = mission(tid, 0, 'reject', 'one_obstacle', bx, by, 0.0, 8.0, 8.0, 0.0)
    nav_log = (LOGS / 'nav.log').read_text(errors='replace')
    stop()
    legs = r['legs'] if r else []
    pattern = r'(Failed to create plan[^\n]*|off the global costmap[^\n]*|GridBased plugin failed[^\n]*|Goal failed[^\n]*|Aborting handle[^\n]*)'
    m = dict(goals=[l['goal'] for l in legs], status=[l['status'] for l in legs], status_name=[l['status_name'] for l in legs],
             time_to_abort_s=[l['sim_time_s'] for l in legs], displacement_m=[l['max_displacement_m'] for l in legs],
             all_aborted=len(legs) == 2 and all(l['status'] == 6 for l in legs),
             max_time_to_abort_s=max((l['sim_time_s'] for l in legs), default=None),
             max_displacement_m=max((l['max_displacement_m'] for l in legs), default=None),
             planner_log=sorted(set(re.sub(r'^.*?\]: ', '', s) for s in re.findall(pattern, nav_log)))[:6])
    return judge(tid, m, f'python3 run.py {tid}  (up.sh one_obstacle; nav.py reject {bx} {by} 0 8 8 0)')


@task('10.10')
def cancel(tid):
    up('room')
    r = mission(tid, 0, 'cancel', 'room', '--n', ACC[tid]['runs'])
    stop()
    legs = r['legs'] if r else []
    m = dict(trials=[{k: l.get(k) for k in ('status_name', 'stop_s', 'walked_after_cancel_m')} for l in legs],
             all_canceled=len(legs) == ACC[tid]['runs'] and all(l['status'] == 5 for l in legs),
             max_stop_s=max((l['stop_s'] for l in legs), default=None) if all(l.get('stop_s') is not None for l in legs) else None,
             max_walked_after_cancel_m=max((l['walked_after_cancel_m'] for l in legs), default=None))
    return judge(tid, m, f'python3 run.py {tid}  (up.sh room; nav.py cancel --n 3)')


def launch_bg(cmd, log, n_active):
    """Start a launch file in its own session; wait until n lifecycle managers report active."""
    f = open(log, 'w')
    p = subprocess.Popen(cmd, shell=True, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, executable='/bin/bash')
    for _ in range(90):
        time.sleep(2)
        if pathlib.Path(log).read_text(errors='replace').count('Managed nodes are active') >= n_active:
            return p
    return p


def end_bg(p):
    """Stop only the process group this script started (its launch file and children), then the rest of the stack."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            break
        time.sleep(3)
    stop()


@task('10.11')
def override(tid):
    up(*ARENA, nav=False)
    p = launch_bg(f'ros2 launch {P11}/arbitration.launch.py', LOGS / 'arbitration.log', 3)
    csv = RUNS / f'{tid}_override.csv'
    rc, txt = sh(f'{PY} {P11}/measure.py override --csv {csv}', timeout=900)
    end_bg(p)
    (RUNS / f'{tid}_override.txt').write_text(txt)
    lat = dict(re.findall(r'key -> (\S+)\s+(\d+) ms', txt))
    back = re.search(r'Nav2 back on cmd_vel ([\d.]+) s', txt)
    res = re.search(r'nav: (RESULT .*)', txt)
    m = dict(key_latency_ms={k: int(v) for k, v in lat.items()},
             key_to_cmd_vel_ms=int(lat['cmd_vel']) if 'cmd_vel' in lat else None,
             nav2_back_s=float(back.group(1)) if back else None,
             nav2_result=res.group(1) if res else None,
             goal_succeeded=bool(res and 'SUCCEEDED' in res.group(1)))
    return judge(tid, m, f'python3 run.py {tid}  (up.sh arena, NAV=0; ros2 launch {P11.relative_to(ROOT)}/arbitration.launch.py; '
                         f'{P11.relative_to(ROOT)}/measure.py override)')


@task('10.12')
def waypoints(tid):
    runs = runs_of(tid, 'waypoints', 'obstacles', WAYPOINTS)
    m = summary(runs)
    errs, missed = [], []
    for r in runs:
        if not r:
            continue
        l = r['legs'][0]
        wps = l['waypoints']
        errs += [math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(l['reached_at'], wps)] + [l['final_error_m']]
        missed.append(l['missed'])
    m.update(missed_waypoints=missed, max_waypoint_error_m=round(max(errs), 3) if errs else None,
             success_rate=round(sum(bool(r) and ok(r['legs'][0]) and not r['legs'][0]['missed'] for r in runs) / len(runs), 3))
    return judge(tid, m, f'python3 run.py {tid}  (up.sh obstacles; nav.py waypoints {" ".join(WAYPOINTS)}) x {len(runs)}')


@task('10.13')
def patrol(tid):
    up(*ARENA, nav=False)
    p = launch_bg(f'ros2 launch {P13}/nav_bt.launch.py bt_xml:={P13}/trees/patrol.xml', LOGS / 'nav_bt.log', 2)
    bt = open(RUNS / f'{tid}_bt.log', 'w')
    btp = subprocess.Popen([PY, str(P13 / 'bt_log.py'), '--gt', str(RUNS / f'{tid}_gt.csv')], cwd=ROOT, stdout=bt,
                           stderr=subprocess.STDOUT, start_new_session=True)
    rc, txt = sh(f'{PY} src/go2_nav_sim/scripts/go_to_goal.py -4.5 -3.5 -2.03', timeout=900)
    time.sleep(2); btp.send_signal(signal.SIGINT); btp.wait(timeout=20); bt.close()
    end_bg(p)
    log = (RUNS / f'{tid}_bt.log').read_text()
    res = re.search(r'RESULT (\S+)\s+sim_time ([\d.]+) s\s+ground-truth final error ([\d.]+) m', txt)
    legs = re.findall(r'^\s*([\d.]+)\s+(Leg\w+|At\w+|Repeat)\s+RUNNING\s+-> (\w+)', log, re.M)
    m = dict(go_to_goal=res.group(0) if res else txt[-300:], tree_events=[' '.join(x) for x in legs],
             patrol_succeeded=any(n == 'Repeat' and s == 'SUCCESS' for _, n, s in legs) and bool(res and res.group(1) == 'SUCCEEDED'),
             sim_time_s=float(res.group(2)) if res else None, final_error_m=float(res.group(3)) if res else None)
    return judge(tid, m, f'python3 run.py {tid}  (up.sh arena, NAV=0; ros2 launch {P13.relative_to(ROOT)}/nav_bt.launch.py '
                         f'bt_xml:=.../trees/patrol.xml; bt_log.py; go_to_goal.py -4.5 -3.5 -2.03)')


@task('10.14')
def go_return(tid):
    goal = mw.WORLDS['obstacles']['goal']
    runs = runs_of(tid, 'return', 'obstacles', goal)
    m = summary(runs)
    m['max_return_error_m'] = max((r['legs'][1]['final_error_m'] for r in runs if r and len(r['legs']) > 1), default=None)
    return judge(tid, m, f'python3 run.py {tid}  (up.sh obstacles; nav.py return {" ".join(map(str, goal))}) x {len(runs)}')


def main():
    RUNS.mkdir(parents=True, exist_ok=True); LOGS.mkdir(exist_ok=True)
    ids = sorted(TASKS, key=lambda s: float(s)) if sys.argv[1:] == ['all'] else sys.argv[1:]
    if not ids or any(i not in TASKS for i in ids):
        sys.exit(__doc__)
    for tid in ids:
        TASKS[tid](tid)


if __name__ == '__main__':
    main()
