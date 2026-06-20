"""cctv_projection 단위테스트 — EXPLORE 실측 랜드마크로 투영 코어 검증.

실행: colcon test  또는  python3 -m pytest test/test_cctv_projection.py
GT(실측): 1n 광축(320,240)→world(14.5,16.41); foot(359,341)→world(13.93,18.03) (person_1 GT(14,18)).
"""
import math

from amr_perception.cctv_projection import (
    rot_rpy,
    project_pixel_to_ground,
    world_to_map,
    map_to_cell,
    pixel_to_cell,
)

# camera_info 실측
K = (293.244, 293.244, 320.0, 240.0)
# warehouse.world extrinsics (world frame)
POSE_1N = (14.5, 18.5, 4.49, 0.0, 1.1345, -1.5708)
POSE_1S = (14.5, 5.5, 4.49, 0.0, 1.1345, 1.5708)
# 실측 world→map 오프셋, /map 정보
OFFSET = (-2.96, -15.12)
ORIGIN = (-3.18, -16.9)
RES = 0.05
W, H = 1207, 840


def test_rot_identity():
    R = rot_rpy(0.0, 0.0, 0.0)
    for i in range(3):
        for j in range(3):
            assert abs(R[i][j] - (1.0 if i == j else 0.0)) < 1e-12


def test_1n_optical_center_hits_floor():
    # 광축(이미지 중앙)은 1n 직하 전방 바닥 ~ (14.5, 16.41)
    p = project_pixel_to_ground(320.0, 240.0, POSE_1N, K)
    assert p is not None
    assert abs(p[0] - 14.5) < 0.05
    assert abs(p[1] - 16.41) < 0.10


def test_1n_person_foot_projection():
    # person_1 발 픽셀 → world, GT(14,18) 와 <0.5m (실측 0.07m)
    p = project_pixel_to_ground(359.0, 341.0, POSE_1N, K)
    assert p is not None
    err = math.hypot(p[0] - 14.0, p[1] - 18.0)
    assert err < 0.5, f"projection error {err:.3f}m too large"
    assert err < 0.15  # 실측 회귀 가드


def test_horizontal_camera_returns_none():
    # 수평(+X)으로 보는 카메라: 광축이 바닥평면을 안 만남 → None
    horiz = (0.0, 0.0, 2.0, 0.0, 0.0, 0.0)
    assert project_pixel_to_ground(320.0, 240.0, horiz, K) is None


def test_world_to_map_and_cell():
    mx, my = world_to_map(13.93, 18.03, OFFSET[0], OFFSET[1])
    assert abs(mx - 10.97) < 1e-6
    assert abs(my - 2.91) < 1e-6
    cell = map_to_cell(mx, my, ORIGIN[0], ORIGIN[1], RES)
    # floor 경계 부동소수 민감(14.15/0.05=282.9999) — ±1 셀 허용(디스크 반경 18셀이라 무의미).
    assert abs(cell[0] - 283) <= 1 and abs(cell[1] - 396) <= 1


def test_pixel_to_cell_full_pipeline():
    res = pixel_to_cell(359.0, 341.0, POSE_1N, K,
                        OFFSET[0], OFFSET[1], ORIGIN[0], ORIGIN[1], RES, W, H)
    assert res is not None
    col, row, mx, my = res
    # person_1 은 좌핀치 복도 col 범위(약 277~312) 내부
    assert 270 <= col <= 320
    assert 380 <= row <= 410


def test_1s_far_clip_geometry():
    # 1s 는 person 까지 ~13m (>10m far-clip). 투영 자체는 되지만 1n 과 다른(먼) 점.
    p = project_pixel_to_ground(320.0, 240.0, POSE_1S, K)
    assert p is not None
    assert p[1] < 9.5  # 1s 광축 바닥점은 복도 입구(Y9.5) 남쪽


if __name__ == "__main__":
    import sys
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fails = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            fails += 1
            print(f"FAIL {fn.__name__}: {e}")
    sys.exit(1 if fails else 0)
