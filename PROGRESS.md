# PROGRESS — CCTV No-Go Overlay (Path B) · `feature/cctv-nogo-overlay`

## Morning Summary (2026-06-20)

### 빌드
`cd /workspace/ros2_ws && colcon build --packages-select amr_perception` → **PASS** (일반 빌드, `--symlink-install` 미사용 §3). 단위테스트 `test_cctv_projection` **7/7 PASS**.

### GATE 결과 (정량)
| Gate | 결과 | 근거(실측) |
|---|---|---|
| C0 sensor live | **PASS** | `/cctv/*` 4대 ~15.6Hz |
| C1 검출 | **PASS** | 1n BG-sub로 person_1 안정 검출 |
| C2 투영 <0.5m | **PASS** | foot→world **(13.93,18.03)** vs GT(14,18) = **0.074m** |
| C3 오버레이 발행+A\* 소비 | **PASS** | 오버레이 1009셀 발행 → `astar: dynamic obstacle layer overlay active: 1009 cells` |
| **C4 리루트 delta** | **🔴 BLOCKED** | A\*가 좌핀치를 애초에 안 씀(clearance 설정) → ON/OFF 경로 동일(서측 x7.21). **amr_navigation 이슈, 스코프 밖.** → `BLOCKED.md` |
| C5 decay/재개방 | **PASS** | person 제거 → 1009 → **0** (~2s) → 0 유지 |
| C6 false-positive 0 | **PASS** | 빈복도 + **이동 로봇**서도 spurious **0** (range-gate + robot-exclusion 후) |

**헤드라인 C4만 미달 — 원인은 A\* clearance 설정(좁은 1.40m 핀치 회피), perception 버그 아님.** A\*가 no-go를 plan grid에 정상 합성함은 증명됨(1009셀). 자세한 원인·증거·추천 → **`BLOCKED.md`**.

### 검출기 락
**BG-subtraction** (빈 복도 frozen 배경 abs-diff). 근거: YOLOv8n은 25° top-down·23px person_1을 **"bird" 0.42로 오분류**(person 0건). BG-sub는 안정 검출 + 투영 0.07m. (좌핀치 데모는 **1n 전담**: 1s는 person까지 >10m far-clip 미검 + 로봇 start 직하라 자기오검.)

### world→map 오프셋 (실측)
spec 가설 (3.18,16.9)는 **오류**(그건 grid origin). 실측 **map = world − (2.96,15.12)** (≈fleet 검증값 3.0,15.0). 1대 스폰+회전→AMCL TF vs ground_truth로 확정.

### dynamic 쇼케이스 (5b)
**inconclusive/보류** — 리루트(C4)에 의존하므로 C4 BLOCKED와 함께 보류. perception 측(사람 입장→오버레이→퇴장→소멸)은 C1+C5로 정상 입증.

### RTF / 환경
ign 단일프로세스 RTF ≈ 0.3–0.5 (정상, §3). GPU 렌더 정상(util>0, libEGL 정상). NumPy 2.x 충돌 회피 위해 이미지 디코드는 `np.frombuffer`(cv_bridge 미사용).

### 산출물
- 커밋(amr_perception만): `cctv_projection.py`(+test), `cctv_detector.py`, `cctv_nogo_overlay.py`, `config/cctv_nogo_params.yaml`, `launch/cctv.launch.py`, `setup.py`.
- 증거 bag: `/workspace/validation/cctv/bags/cctv_evidence`(비커밋) — detections 68 + dynamic_obstacle_layer 36(~4.6Hz).
- 프레임: `/workspace/validation/cctv/frames/`(비커밋) — `corridor_1n_yolo.png`(bird 오분류), `map_annotated.png`.
- 검증 스크립트/런치: `/workspace/validation/cctv/`(비커밋).

### `git diff --stat` (이번 런 amr_perception)
PLAN/feat 커밋 2건 + 게이트 수정(if-k 크래시, 1n-only, range-gate, robot-exclusion): `cctv_nogo_overlay.py` +37, `cctv_nogo_params.yaml` +8.

### Open blocker
**C4 리루트** — `BLOCKED.md` 참조. A\* clearance(amr_navigation) 또는 map2 핀치폭(US) 변경 필요(스코프 밖). 변경 즉시 본 파이프라인 무수정 리루트 가능.

### HU 재현 명령어
```bash
source /workspace/env.sh
# 1) 시뮬(헤드리스, mover off)
bash /workspace/validation/cctv/sim_gazebo_static.sh &        # gazebo + world
# 2) 단일로봇(검증용, robot.launch.py 버그 회피)
bash /workspace/validation/cctv/scratch/launch_single.sh 14.5 5.0 true &   # robot+nav+cctv bridge
python3 /workspace/validation/cctv/scratch/seed_initialpose.py 11.54 -10.12 1.5708
# 3) CCTV 검출+오버레이 (정식 launch)
ros2 launch amr_perception cctv.launch.py use_bridge:=false map_topic:=/amr1/map layer_topic:=/amr1/dynamic_obstacle_layer
# 게이트 1런: bash /workspace/validation/cctv/scratch/run_gate.sh on  (또는 off)
# C5/C6: python3 /workspace/validation/cctv/scratch/c5c6_check.py
```

### 다음 추천
1. **C4 unblock**(HU/소유팀): `BLOCKED.md` 추천 1~3 중 택1(A\* clearance 완화 / map2 핀치 확장 / 우회로 차단). 그 후 `run_gate.sh on`/`off`로 delta + bag.
2. unblock 시 dynamic 쇼케이스(mover ON) 녹화(Foxglove 머니샷, `/cctv/corridor_1n/debug` + `/amr1/dynamic_obstacle_layer`).
