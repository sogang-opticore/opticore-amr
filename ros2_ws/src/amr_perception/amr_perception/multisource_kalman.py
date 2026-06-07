#!/usr/bin/env python3
"""
multisource_kalman.py — P-7 multi-source 칼만 + association + 클래스 라벨

ROS 비의존 순수 모듈 (lidar_clustering.py 패턴). fused_tracker가 _tick에서 소비.
- FusedKalmanTrack: object_tracker.KalmanTrack 확장. source별 R 차등 + tentative/confirmed.
- associate(): 위치 근접 + 클래스 일관성 greedy 매칭.
위치는 칼만이 공분산으로 자동 가중(LiDAR R 작게 → 더 신뢰). 클래스는 YOLO 관측이 채움.
생성 강화: 새 트랙은 tentative, min_hits 관측 누적돼야 confirmed(=publish 대상).
"""

from dataclasses import dataclass
import numpy as np

# source 비트마스크 — amr_msgs/TrackedObject 상수와 반드시 일치
SOURCE_LIDAR = 1
SOURCE_ROBOT_CAM = 2
SOURCE_CCTV = 4

# source별 관측 노이즈 R (2x2, m^2). [미확정 - 튜닝 필요] 출발값.
#   LiDAR: 클러스터 centroid 위치 정밀 → 작게
#   robot_cam: ground-projection, depth 부정확 → 크게
#   CCTV: 고정 extrinsic, 중간 (P-10에서 검증)
# NOTE: 지금은 isotropic. depth 방향만 더 크게(anisotropic) 하려면
#       update()에 robot→obs bearing으로 회전한 2x2 R을 넘기도록 확장.
R_BY_SOURCE = {
    SOURCE_LIDAR:     np.diag([0.05, 0.05]),
    SOURCE_ROBOT_CAM: np.diag([0.80, 0.80]),
    SOURCE_CCTV:      np.diag([0.50, 0.50]),
}
R_DEFAULT = np.eye(2) * 0.5


@dataclass
class Observation:
    """단일 소스 관측. fused_tracker가 LiDAR/YOLO를 이 형태로 정규화해 넘긴다."""
    x: float
    y: float
    source: int                 # SOURCE_* 단일 값 (관측은 한 소스에서만 옴)
    class_name: str = None      # YOLO 관측만 채움. LiDAR=None
    confidence: float = 0.0


class FusedKalmanTrack:
    """단일 융합 트랙. 등속 모델 [x, y, vx, vy], source별 R로 update."""

    _next_id = 0

    def __init__(self, obs: 'Observation', dt: float, min_hits: int = 3):
        self.track_id = FusedKalmanTrack._next_id
        FusedKalmanTrack._next_id += 1
        self.dt = dt
        self.miss_count = 0

        # 생성 강화: min_hits 관측 누적돼야 confirmed. 그 전엔 tentative(미발행).
        self.min_hits = max(1, int(min_hits))
        self.hit_count = 1
        self.confirmed = self.hit_count >= self.min_hits

        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=float)
        self.Q = np.eye(4) * 0.1
        self.x = np.array([obs.x, obs.y, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 1.0

        # 클래스: LiDAR-only면 unknown, YOLO면 즉시 라벨
        self.class_name = obs.class_name or 'unknown'
        self.class_confidence = obs.confidence if obs.class_name else 0.0
        self.source = obs.source  # 비트마스크 OR 누적

    def _F(self, dt: float):
        return np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

    def predict(self, dt: float = None):
        dt = self.dt if dt is None else dt
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, obs: 'Observation'):
        R = R_BY_SOURCE.get(obs.source, R_DEFAULT)
        z = np.array([obs.x, obs.y], dtype=float)
        innov = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.miss_count = 0
        self.source |= obs.source
        self._update_class(obs)

        self.hit_count += 1
        if self.hit_count >= self.min_hits:
            self.confirmed = True

    def _update_class(self, obs: 'Observation'):
        # YOLO 관측만 클래스 갱신. unknown은 언제든 덮어쓰고, 그 외엔 conf 높은 쪽 우선.
        if not obs.class_name:
            return
        if self.class_name == 'unknown' or obs.confidence >= self.class_confidence:
            self.class_name = obs.class_name
            self.class_confidence = obs.confidence

    @property
    def position(self):
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self):
        return float(self.x[2]), float(self.x[3])

    @property
    def position_covariance(self):
        # 2x2 row-major [xx, xy, yx, yy] — amr_msgs float32[4]
        c = self.P[0:2, 0:2]
        return [float(c[0, 0]), float(c[0, 1]), float(c[1, 0]), float(c[1, 1])]


def _class_compatible(track_cls: str, obs_cls: str) -> bool:
    # 둘 다 확정 클래스이고 다르면 매칭 금지. unknown/None은 아무거나 호환.
    if not obs_cls or obs_cls == 'unknown':
        return True
    if not track_cls or track_cls == 'unknown':
        return True
    return track_cls == obs_cls


def associate(tracks, observations, max_dist: float):
    """
    greedy nearest-neighbor 매칭. 게이트: 거리 <= max_dist AND 클래스 일관성.
    한 source 그룹 단위로 호출 → 트랙이 source별로 최대 1개씩 흡수(LiDAR+YOLO 동시 융합).
    반환: (matches[(t_idx, o_idx)], unmatched_obs_idx, unmatched_track_idx)
    """
    pairs = []
    for ti, t in enumerate(tracks):
        tx, ty = t.position
        for oi, o in enumerate(observations):
            if not _class_compatible(t.class_name, o.class_name):
                continue
            d = np.hypot(o.x - tx, o.y - ty)
            if d <= max_dist:
                pairs.append((d, ti, oi))
    pairs.sort(key=lambda p: p[0])

    matched_t, matched_o, matches = set(), set(), []
    for d, ti, oi in pairs:
        if ti in matched_t or oi in matched_o:
            continue
        matched_t.add(ti)
        matched_o.add(oi)
        matches.append((ti, oi))

    unmatched_obs = [oi for oi in range(len(observations)) if oi not in matched_o]
    unmatched_tracks = [ti for ti in range(len(tracks)) if ti not in matched_t]
    return matches, unmatched_obs, unmatched_tracks
