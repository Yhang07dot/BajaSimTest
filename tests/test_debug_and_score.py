from pathlib import Path

import pytest
import yaml

from baja_perception_sim.score_display import format_score, marker_text

ROOT = Path(__file__).resolve().parents[1]


def test_debug_view_has_sensor_cloud_without_truth_or_map():
    cfg = yaml.safe_load((ROOT / 'src/baja_perception_sim/config/debug.rviz').read_text())
    manager = cfg['Visualization Manager']
    topics = [d.get('Topic', {}).get('Value', '') for d in manager['Displays']]
    assert '/sensors/lidar/points' in topics
    assert '/sensors/front_camera/image_raw' in topics
    assert all(not t.startswith('/ground_truth/') for t in topics)
    assert manager['Global Options']['Fixed Frame'] == 'base_link'


@pytest.mark.parametrize('name', ['debug.rviz', 'perception.rviz'])
def test_rviz_starts_with_mouse_camera_tool(name):
    cfg = yaml.safe_load((ROOT / 'src/baja_perception_sim/config' / name).read_text())
    manager = cfg['Visualization Manager']
    assert manager['Tools'][0]['Class'] == 'rviz_default_plugins/MoveCamera'
    assert manager['Views']['Current']['Class'] == 'rviz_default_plugins/Orbit'
    assert any(v['Name'] == 'Reset view' for v in manager['Views']['Saved'])


def test_score_waiting_is_not_displayed_as_a_measurement():
    report = {'status': 'waiting_for_algorithm', 'boundary': {'status': 'waiting_for_algorithm'}}
    assert '尚未评分' in format_score(report)
    assert '0.0 / 100' not in format_score(report)
    assert 'waiting for algorithm' in marker_text(report)


def test_score_renders_metrics_and_missing_messages():
    item = {'score_100': 80., 'precision': .8, 'recall': .8, 'f1': .8, 'tp': 8, 'fp': 2, 'fn': 2}
    report = {'status': 'running', 'objectness': item, 'semantic': item, 'frames': 10,
              'missing_frames': 2, 'boundary': {'status': 'waiting_for_algorithm'}}
    text = format_score(report)
    assert '80.0 / 100' in text and 'Recall 80.0%' in text
    assert '缺帧 2' in text
    assert '80.0 / 100' in marker_text(report)
