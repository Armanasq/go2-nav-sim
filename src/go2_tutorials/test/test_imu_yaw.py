import math

from go2_tutorials.imu_yaw import YawIntegrator


def test_constant_rate():
    y = YawIntegrator()
    for i in range(201):                   # 100 Hz for 2 s at 0.5 rad/s
        y.add(i * 0.01, 0.5)
    assert math.isclose(y.yaw, 1.0, abs_tol=1e-9)


def test_gap_and_out_of_order_are_skipped():
    y = YawIntegrator(max_gap=0.5)
    y.add(0.0, 1.0)
    y.add(0.1, 1.0)                        # +0.1
    y.add(0.05, 1.0)                       # older stamp: ignored
    y.add(2.0, 1.0)                        # 1.9 s gap: skipped
    y.add(2.1, 1.0)                        # +0.1
    assert math.isclose(y.yaw, 0.2, abs_tol=1e-9)
