"""Readable score presentation; works without ROS or a running simulator."""


def percentage(value):
    return '--' if value is None else f'{value * 100:.1f}%'


def points(value):
    return '--' if value is None else f'{value:.1f} / 100'


def format_score(result):
    lines = ['感知算法评分', '============']
    if result.get('status') == 'waiting_for_algorithm':
        lines.append('三维检测：等待算法输出 /perception/objects_3d，尚未评分。')
    else:
        for key, label in [('objectness', '障碍检出'), ('semantic', '类别识别')]:
            item = result.get(key, {})
            lines += [f'{label}：{points(item.get("score_100"))}',
                      f'  Precision {percentage(item.get("precision"))}  Recall {percentage(item.get("recall"))}  F1 {percentage(item.get("f1"))}',
                      f'  正确 TP={item.get("tp", 0)}  误检 FP={item.get("fp", 0)}  漏检 FN={item.get("fn", 0)}']
    boundary = result.get('boundary', {})
    if boundary.get('status') == 'waiting_for_algorithm':
        lines.append('道路边界：等待算法输出 /perception/road_boundaries。')
    else:
        error = boundary.get('mean_error_on_covered_points_m')
        error_text = '--' if error is None else f'{error:.3f} m'
        lines.append(f'道路边界：覆盖率 {percentage(boundary.get("coverage_within_0_5m"))}，已覆盖点平均误差 {error_text}')
    lines += [f'已评分帧 {result.get("frames", 0)}  缺帧 {result.get("missing_frames", 0)}  不合法消息 {result.get("invalid_messages", 0)}',
              '范围：前方 ROI 内有激光返回的三维目标；二维和特殊路面暂未计分。']
    return '\n'.join(lines)


def marker_text(result):
    if result.get('status') == 'waiting_for_algorithm':
        objects = '3D objects: waiting for algorithm'
    else:
        objects = (f'Object detection: {points(result["objectness"]["score_100"])}\n'
                   f'Classification: {points(result["semantic"]["score_100"])}')
    boundary = result.get('boundary', {})
    boundary_text = ('waiting for algorithm' if boundary.get('status') == 'waiting_for_algorithm'
                     else percentage(boundary.get('coverage_within_0_5m')))
    return (f'PERCEPTION SCORE (F1 x 100)\n{objects}\n'
            f'Boundary coverage: {boundary_text}\n'
            f'Frames: {result.get("frames", 0)}   Missing: {result.get("missing_frames", 0)}')
