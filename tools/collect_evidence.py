#!/usr/bin/env python3
"""Collect evidence files for the site:  python3 tools/collect_evidence.py --runs w1 w2 w3 [--setup-log PATH]
  site/evidence/pytest.txt      unit test output (run now)
  site/evidence/missions.txt    RESULT lines of each mission log (logs/<run>.log), plus missions.json for the tables
  site/evidence/setup.txt       tail of a scripts/setup.sh log, if given"""
import argparse, datetime, json, pathlib, re, subprocess, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]; EV = ROOT / 'site/evidence'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--runs', nargs='+', required=True); ap.add_argument('--setup-log'); a = ap.parse_args()
    EV.mkdir(parents=True, exist_ok=True); stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    t = subprocess.run([sys.executable, '-m', 'pytest', '-v', '--no-header', 'src/go2_nav_sim/test', 'modules'], cwd=ROOT, capture_output=True, text=True)
    (EV / 'pytest.txt').write_text(f'$ python -m pytest -v src/go2_nav_sim/test modules        # {stamp}\n' + re.sub(r'\n{2,}', '\n', t.stdout).strip() + '\n')
    lines, ms = [f'# {stamp}, one machine, final code, consecutive runs'], []
    for run in a.runs:
        log = (ROOT / 'logs' / f'{run}.log').read_text()
        res = re.search(r'RESULT (\w+)\s+sim_time ([\d.]+) s\s+ground-truth final error ([\d.]+) m', log)
        retries = log.count('retrying')
        lines.append(f'$ scripts/run_mission.sh bags/{run}')
        lines += [l for l in log.splitlines() if re.search(r'stack up|retrying|goal ->|distance remaining|RESULT', l)]
        ms.append({'run': run, 'result': res.group(1) if res else 'NO RESULT', 'sim_time_s': float(res.group(2)) if res else None,
                   'final_error_m': float(res.group(3)) if res else None, 'nav2_retries': retries})
    (EV / 'missions.txt').write_text('\n'.join(re.sub(r'\[INFO\] \[\d+\.\d+\] \[basic_navigator\]: ', '  ', l) for l in lines) + '\n')
    (EV / 'missions.json').write_text(json.dumps(ms, indent=2))
    if a.setup_log:
        s = pathlib.Path(a.setup_log).read_text().splitlines()
        keep = [l for l in s if re.search(r'patch|plugins|Summary|Finished <<<|Failed|done\.|error', l, re.I)]
        (EV / 'setup.txt').write_text(f'# scripts/setup.sh in a fresh clone, {stamp}\n' + '\n'.join(keep[-40:]) + '\n')
    print('pytest:', 'passed' if t.returncode == 0 else 'FAILED', '| missions:', [m['result'] for m in ms])


if __name__ == '__main__':
    main()
