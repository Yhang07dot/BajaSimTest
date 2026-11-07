"""ROS-independent geometry used by generation, control and scoring."""
import math
from collections import deque

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation, Slerp


def track_length(cfg):
    return 2 * (cfg['straight_x'] + cfg['straight_y']) + 2 * math.pi * cfg['radius']


def pursuit_yaw_rate(speed, target):
    """Limit path curvature, preserving steering authority as speed changes.

    0.30/m is inside the model's tan(26deg)/1.43m physical steering limit.
    A fixed 0.3rad/s limit cannot correct a 10m-radius turn at 3m/s.
    """
    curvature = 2 * target[1] / max(target[0]**2 + target[1]**2, 0.1)
    return float(speed * np.clip(curvature, -0.30, 0.30))


def track_pose(s, cfg):
    s %= track_length(cfg)
    x = y = yaw = 0.0
    r = cfg['radius']
    for length in [cfg['straight_x'], cfg['straight_y']] * 2:
        ds = min(s, length)
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        s -= ds
        if s <= 1e-10:
            return x, y, yaw
        ds = min(s, math.pi * r / 2)
        angle = ds / r
        x += r * (math.sin(yaw + angle) - math.sin(yaw))
        y -= r * (math.cos(yaw + angle) - math.cos(yaw))
        yaw += angle
        s -= ds
        if s <= 1e-10:
            return x, y, yaw
    return x, y, yaw


def track_samples(cfg):
    n = math.ceil(track_length(cfg) / cfg['sample_step'])
    return np.array([(*track_pose(s, cfg), s) for s in
                     np.linspace(0, track_length(cfg), n, endpoint=False)])


def rotation(q):
    return Rotation.from_quat(q).as_matrix()


def to_local(points, position, quaternion):
    return (np.asarray(points) - position) @ rotation(quaternion)


def points_in_box(points, obj, tolerance=0.035):
    p = np.asarray(points) - obj['position']
    c, s = math.cos(obj['yaw']), math.sin(obj['yaw'])
    local = p @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.all(np.abs(local) <= np.asarray(obj['size']) / 2 + tolerance, axis=1)


class PoseHistory:
    def __init__(self, seconds=8.0):
        self.items = deque()
        self.seconds = seconds

    def add(self, t, position, quaternion):
        if self.items and t < self.items[-1][0]:
            self.items.clear()
        self.items.append((t, np.array(position), np.array(quaternion)))
        while self.items and t - self.items[0][0] > self.seconds:
            self.items.popleft()

    def at(self, t, tolerance=0.05):
        if not self.items or t < self.items[0][0] - tolerance or t > self.items[-1][0] + tolerance:
            return None
        ts = [a[0] for a in self.items]
        i = int(np.searchsorted(ts, t))
        if i == 0:
            return self.items[0][1:]
        if i == len(ts):
            return self.items[-1][1:]
        a, b = self.items[i-1], self.items[i]
        f = (t - a[0]) / (b[0] - a[0])
        q = Slerp([a[0], b[0]], Rotation.from_quat([a[2], b[2]]))([t]).as_quat()[0]
        return a[1] * (1-f) + b[1] * f, q


def match_centres(pred, truth, gate):
    """Maximum-cardinality one-to-one matching, then minimum distance."""
    if not pred or not truth:
        return []
    d = np.linalg.norm(np.array(pred)[:, None, :] - np.array(truth)[None, :, :], axis=2)
    penalty = (min(len(pred), len(truth)) + 1) * (gate + 1)
    rows, cols = linear_sum_assignment(np.where(d <= gate, d, penalty))
    return [(int(i), int(j), float(d[i, j])) for i, j in zip(rows, cols) if d[i, j] <= gate]


def polyline_distances(points, segments):
    """Distance to finite segments; no extrapolation across unobserved gaps."""
    p = np.asarray(points)
    result = np.full(len(p), np.inf)
    for line in segments:
        line = np.asarray(line)
        for a, b in zip(line[:-1], line[1:]):
            v = b - a
            t = np.clip((p-a) @ v / max(float(v@v), 1e-12), 0, 1)
            result = np.minimum(result, np.linalg.norm(p - a - t[:, None]*v, axis=1))
    return result
