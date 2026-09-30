# OptiCore AMR-Lite Phase 0

ROS2와 Gazebo 없이 실행되는 경량 2D AMR 로컬 플래닝 기술 스파이크다. 동일한 관측·행동 계약에서 `lite_rule_baseline`, Behavior Cloning(BC), PPO, BC+DAgger-lite를 실행하고, 공통 Safety Shield의 효과를 paired evaluation으로 비교한다.

이 디렉터리는 기존 `ros2_ws`와 독립적이다. 기존 `dwa_node.py`를 복사하거나 import하지 않으며, ROS 플래너의 제약값과 설계만 참고했다.

## 구현 범위

- differential-drive 운동 모델과 물리 substep 충돌 검사
- 직사각형 정적 장애물 및 scripted 원형 동적 장애물
- 72-ray, 270° 가상 LiDAR와 최근 4-frame stack
- 정적 지도 A* 전역 경로, path projection, arc-length local goal
- Pure Pursuit 기반 `lite_rule_baseline`
- 공통 normalized action 및 Safety Shield
- episode 단위 teacher NPZ, BC, DAgger-lite
- 외부 RL 라이브러리 없이 동작하는 소형 PyTorch PPO
- shield OFF/ON paired evaluation, CSV, 그래프, 최종-frame 데모

## 요구 환경

- Python 3.10 이상
- NumPy, PyYAML, PyTorch, Matplotlib
- 테스트에는 pytest

현재 저장소에서 확인한 `python` 3.11 환경에는 위 패키지가 설치되어 있다. 새 환경에서는 다음과 같이 프로젝트 전용 가상환경을 사용할 수 있다.

```bash
cd amr_lite_spike
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
```

설치하지 않고 저장소에서 직접 실행할 때는 아래 명령처럼 `PYTHONPATH=.`을 사용한다.

## 가장 짧은 실행

다음 한 줄은 teacher 데이터 생성 → BC smoke → PPO smoke → paired evaluation → 그래프와 데모 생성을 순서대로 실행한다.

```bash
cd amr_lite_spike
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.train
```

Smoke는 코드 경로와 checkpoint 입출력을 검증하기 위한 실행이다. BC/PPO 수렴을 보장하거나 주장하지 않는다. Smoke 평가·그래프·데모는 `artifacts/smoke/`에 저장되어 전체 평가 결과를 덮어쓰지 않는다.

## 단계별 실행

테스트:

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib pytest -q
```

Teacher dataset:

```bash
PYTHONPATH=. python -m amr_lite.learning.collect_teacher \
  --episodes 8 --output artifacts/datasets/teacher
```

BC smoke/full:

```bash
PYTHONPATH=. python -m amr_lite.learning.train_bc
PYTHONPATH=. python -m amr_lite.learning.train_bc --full \
  --output artifacts/checkpoints/bc_full.pt
```

DAgger-lite 1회와 재학습:

```bash
PYTHONPATH=. python -m amr_lite.learning.collect_dagger \
  --checkpoint artifacts/checkpoints/bc_smoke.pt
PYTHONPATH=. python -m amr_lite.learning.train_bc \
  --dataset artifacts/datasets/teacher artifacts/datasets/dagger \
  --output artifacts/checkpoints/bc_dagger_smoke.pt
```

PPO smoke/full:

```bash
PYTHONPATH=. python -m amr_lite.learning.train_ppo
PYTHONPATH=. python -m amr_lite.learning.train_ppo --full \
  --output artifacts/checkpoints/ppo_full.pt
```

BC warm start, 고정 평가 callback, best checkpoint를 포함한 PPO 실험:

```bash
PYTHONPATH=. python -m amr_lite.learning.train_ppo --full \
  --warm-start-dataset artifacts/datasets/teacher \
  --output artifacts/checkpoints/ppo_bc_best_full.pt
```

최종 step은 `ppo_bc_best_full.pt`, 학습 중 평가가 가장 좋았던 모델은 `ppo_bc_best_full_best.pt`에 저장된다. 현재 실험에서 best는 RL 업데이트 전 timestep 0이므로, 성능을 `PPO가 향상시켰다`고 해석하면 안 된다.

권장 stable PPO는 actor/critic encoder 분리, value normalization, KL early stop을 추가한다.

```bash
PYTHONPATH=. python -m amr_lite.learning.train_ppo --stable --full \
  --warm-start-dataset artifacts/datasets/teacher \
  --output artifacts/checkpoints/ppo_stable_full.pt
```

현재 stable best는 51,200 timestep에서 선택됐으며 실제 PPO 업데이트 후 모델이다.

고정 seed paired evaluation:

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.evaluate
```

빠른 세 시나리오 평가에는 `--quick`을 추가한다. `configs/evaluation.yaml`의 `shield_modes`가 `[false, true]`이므로 모든 정책의 raw 결과와 shield 적용 결과가 함께 생성된다.

데모 스크린샷:

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.demo \
  --policy rule --scenario S3 --output artifacts/videos/rule_S3.png
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.demo \
  --policy bc --checkpoint artifacts/checkpoints/bc_dagger_smoke.pt \
  --scenario D0 --output artifacts/videos/bc_dagger_D0.png
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.demo \
  --policy ppo --checkpoint artifacts/checkpoints/ppo_smoke.pt \
  --scenario D0 --output artifacts/videos/ppo_D0.png
```

`--show`를 추가하면 실행 중 화면을 갱신한다. 헤드리스 환경에서는 최종-frame PNG를 사용한다.

## 관측·행동 계약

관측은 항상 `float32[296]`이다.

```text
scan_t-3[72], scan_t-2[72], scan_t-1[72], scan_t[72],
normalized local-goal distance, sin/cos local-goal angle,
normalized v/w,
normalized signed cross-track error, sin/cos path-heading error
```

LiDAR history 순서는 oldest → newest이며 거리는 `[0, 1]`로 정규화된다. 행동은 `[a_v, a_w] ∈ [-1, 1]^2`이고 다음과 같이 명령으로 변환된다.

```text
v_target = (a_v + 1) / 2 × v_max
w_target = a_w × w_max
```

그 뒤 모든 정책에 같은 가속도 제한과 Safety Shield를 적용한다.

## 시나리오

- `S0` 빈 직선, `S1` 90° 코너, `S2` 좁은 통로
- `S3` 전역 경로 위 정적 장애물, `S4` 장애물 옆 통과
- `S5` 경로에서 벗어난 시작, `S6` 부분 폐쇄 통로, `S7` no-path
- `D0` 횡단, `D1` 정면 접근, `D2` 같은 방향 저속 이동
- `D3` 통로 정지, `D4` 이동 후 갑작스러운 정지

## 설정

주요 설정은 모두 `configs/`에 있다.

- `env.yaml`: 로봇, 물리, LiDAR, 관측, reward, shield
- `baseline.yaml`: Pure Pursuit와 clearance 기반 감속
- `bc.yaml`: MLP, split, batch, epoch
- `ppo.yaml`: smoke/full timestep과 PPO hyperparameter
- `evaluation.yaml`: 시나리오, paired seed, shield mode

기본 로봇 반경 0.36 m, 제어율 20 Hz, lookahead 0.65 m는 기존 ROS 설정을 참고했다.

## 결과물

```text
artifacts/datasets/       episode/chunk NPZ
artifacts/checkpoints/    BC/PPO checkpoint와 학습 로그
artifacts/results/        episodes.csv, summary.csv, config.json
artifacts/plots/          success/collision/time/training 그래프
artifacts/videos/         주행 최종-frame PNG
artifacts/smoke/          통합 smoke 명령 전용 결과
artifacts/full/           full BC/PPO paired evaluation과 데모
artifacts/reward_v2/       path-progress reward PPO 실험
artifacts/ppo_best/        BC warm-start PPO best-checkpoint 평가
artifacts/ppo_stable/      안정화 PPO final/best 평가와 데모
```

측정 결과와 해석은 [docs/RESULTS.md](docs/RESULTS.md), 설계 결정은 [docs/DECISIONS.md](docs/DECISIONS.md), 한계는 [docs/KNOWN_LIMITATIONS.md](docs/KNOWN_LIMITATIONS.md)를 참고한다. M1 재현성 재평가 결과는 [docs/M1_RESULTS.md](docs/M1_RESULTS.md)에 별도로 정리했다.

현재 코드의 실행 가능 여부와 M1 산출물을 단계별로 확인할 때는
[실행·검증 런북](docs/RUNBOOK.md)을 따른다.

## M1 재현 가능한 기준선

다음 명령은 Stable PPO를 학습 seed 5개로 반복하고, 분리된 selection/test
seed에서 best와 final checkpoint를 평가한다.

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.m1
```

결과는 `artifacts/m1/m1-reproducible-baseline-v1/` 아래에 episode 원자료,
학습-seed·시나리오별 요약, 95% CI, paired comparison, manifest와 보고서로
저장된다. 완료된 seed checkpoint는 config와 timestep을 검증한 뒤 재사용한다.

## 해석 시 주의

`lite_rule_baseline`은 기존 ROS rule-based hybrid local planner의 축소판이며 동등 구현이 아니다. PPO smoke는 256 timestep이며 실행 경로 확인용이다. 초기 shared-encoder 100,000-timestep PPO는 작은 원을 도는 정책으로 붕괴했지만, stable PPO best는 51,200 timestep에서 전체 평가 성공률 87.5%를 기록했다. 다만 final checkpoint는 50%로 저하됐고 `D1` 정면 접근 실패가 남아 있으므로 수렴·일반화를 주장하지 않는다. 자세한 분석은 결과와 한계 문서에 명시했다.
