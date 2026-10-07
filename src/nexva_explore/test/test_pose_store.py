import math
import os

from nexva_explore import pose_store


def test_round_trip(tmp_path):
    path = str(tmp_path / 'm.pose.yaml')
    pose_store.save_pose(path, 1.5, -2.25, math.pi / 2, 'm', source='hardware')
    pose = pose_store.load_pose(path)
    assert pose['x'] == 1.5 and pose['y'] == -2.25
    assert abs(pose['yaw_rad'] - math.pi / 2) < 1e-4
    assert pose['yaw_deg'] == 90.0
    assert pose['map_name'] == 'm'
    assert [f for f in os.listdir(tmp_path)] == ['m.pose.yaml']   # no temp left


def test_missing_and_corrupt(tmp_path):
    assert pose_store.load_pose(str(tmp_path / 'none.yaml')) is None
    bad = tmp_path / 'bad.yaml'
    bad.write_text('x: 1\ny: [unclosed')
    assert pose_store.load_pose(str(bad)) is None
    bad.write_text('x: 1\ny: 2\n')            # no yaw
    assert pose_store.load_pose(str(bad)) is None
    bad.write_text('x: .nan\ny: 2\nyaw_rad: 0\n')
    assert pose_store.load_pose(str(bad)) is None


def test_refuses_nan(tmp_path):
    try:
        pose_store.save_pose(str(tmp_path / 'p.yaml'), float('nan'), 0, 0, 'm')
    except ValueError:
        return
    raise AssertionError('NaN was saved')


def test_guard(tmp_path):
    path = str(tmp_path / 'm.pose.yaml')
    map_yaml = tmp_path / 'm.yaml'
    map_yaml.write_text('x')
    pose_store.save_pose(path, 0, 0, 0, 'm', source='hardware')
    pose = pose_store.load_pose(path)
    assert pose_store.check_pose_for_map(pose, 'm', str(map_yaml), 'hardware')[0]
    assert not pose_store.check_pose_for_map(pose, 'other', str(map_yaml))[0]
    assert not pose_store.check_pose_for_map(pose, 'm', str(map_yaml), 'sim')[0]
    assert not pose_store.check_pose_for_map(None, 'm')[0]
    os.utime(map_yaml, (pose['saved_unix'] + 7200,) * 2)     # map re-saved later
    assert not pose_store.check_pose_for_map(pose, 'm', str(map_yaml))[0]
