"""The stage files agree with the numbers run.py assumes (no ROS, no simulator)."""
import json
import pathlib
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parents[1]


def test_acceptance_has_every_task_with_valid_rules():
    acc = json.loads((HERE / 'acceptance.json').read_text())
    assert [k for k in acc if k[0].isdigit()] == [f'1.{i:02d}' for i in range(1, 11)]
    for tid in (k for k in acc if k[0].isdigit()):
        assert acc[tid]['checks'] and acc[tid]['basis']
        for rule in acc[tid]['checks'].values():
            assert set(rule) <= {'max', 'min', 'equals'} and rule


def test_obstacle_box_face_is_where_run_py_expects():
    box = ET.parse(HERE / 'worlds/obstacles.world').getroot().find(".//model[@name='box']")
    assert box is not None
    pose, size = box.findtext('pose'), box.findtext('link/collision/geometry/box/size')
    assert pose is not None and size is not None
    x = float(pose.split()[0])
    depth = float(size.split()[0])
    assert abs((x - depth / 2) - (-3.0)) < 1e-9           # run.py 1.09: face_x = -3.0, 1.5 m ahead of the spawn x = -4.5
    assert box.findtext('static') == '1'


def test_empty_world_has_only_the_ground():
    w = ET.parse(HERE / 'worlds/empty.world').getroot()
    assert [m.get('name') for m in w.iter('model')] == ['ground_plane']
    assert w.findtext('.//physics/max_step_size') == '0.001'


def test_extra_xacro_adds_rear_cam_once():
    text = (HERE / 'urdf/go2_extra.xacro').read_text()
    assert text.count('<link name="rear_cam_link">') == 1 and '<origin xyz="-0.25 0 0.06" rpy="0 0 3.14159"/>' in text


def test_extra_xacro_has_no_colon_space():
    # ros2 run ... -p robot_description:="$(xacro ...)" parses the value as YAML; a ': ' anywhere aborts robot_state_publisher.
    assert ': ' not in (HERE / 'urdf/go2_extra.xacro').read_text()
