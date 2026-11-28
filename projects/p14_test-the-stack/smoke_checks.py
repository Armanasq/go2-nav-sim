"""The pass/fail rules of the smoke test, kept free of ROS so they can be unit-tested (tests/test_smoke_checks.py)."""

# topic -> rate in Hz that the configuration asks for (sensors.xacro, the upstream IMU plugin, ekf.yaml `frequency`)
EXPECTED = {'/scan': 10.0, '/imu/data': 100.0, '/odom': 50.0}
TOLERANCE = 0.10          # +-10 % of the expected rate
DEADLINE = 120.0          # s of wall-clock time from launch to the first message (and to the first map -> base_footprint)
WINDOW = 5.0              # s of simulated time over which each rate is measured


def rate(stamps):
    """Messages per second of simulated time from header stamps (s). Under load Gazebo runs slower than real time, so a
    wall-clock rate would fail a healthy stack; the stamps say how often the data was produced in the simulation."""
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def check(first_seen, stamps, tf_seen, expected=EXPECTED, deadline=DEADLINE, tol=TOLERANCE):
    """first_seen: topic -> wall s to first message (missing = never); stamps: topic -> header stamps in s;
    tf_seen: wall s to the first map -> base_footprint lookup that worked, or None. Returns failure strings, [] = pass."""
    fails = []
    for topic, hz in expected.items():
        t = first_seen.get(topic)
        if t is None or t > deadline:
            fails.append(f'{topic}: no message within {deadline:.0f} s')
            continue
        r = rate(stamps.get(topic, []))
        if abs(r - hz) > tol * hz:
            fails.append(f'{topic}: {r:.1f} Hz, expected {hz:.0f} Hz +- {tol:.0%}')
    if tf_seen is None or tf_seen > deadline:
        fails.append(f'map -> base_footprint: no transform within {deadline:.0f} s')
    return fails
