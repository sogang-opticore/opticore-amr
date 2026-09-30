# AMR-Lite 실행·검증 런북

이 문서는 현재 AMR-Lite 코드가 실행 가능한지, 기존 M1 결과가 보존되어 있는지,
새 변경이 기준선을 깨뜨리지 않았는지를 같은 절차로 확인하기 위한 운영 문서다.

명령 모음은 `README.md`, 결과 해석은 `RESULTS.md`와 `M1_RESULTS.md`에도 있지만,
실행 전 점검부터 합격 판정과 장애 대응까지 한 번에 이어지는 문서는 이 런북을
기준으로 한다.

## 1. 현재 기준선

- 기준일: 2026-09-30
- 코드 기준 커밋: `16e0a41` (`feat(amr-lite): add predictive safety shield core`)
- Python: 3.10 이상, 현재 확인 환경은 Python 3.11
- 단위테스트 기준: `27 passed`
- 기본 Safety Shield: `legacy`
- M1 학습 seed: `7, 17, 29, 43, 59`
- M1 최종 평가: 4,160 episode
- M1 custom Stable PPO best: 성공률 85.4%, 95% CI 84.0–86.5%
- Rule baseline: 성공률 86.2%, 95% CI 84.4–87.5%

예측형 shield 코어와 테스트는 들어가 있지만 `configs/env.yaml`의 기본값은
`shield_strategy: legacy`다. 따라서 predictive shield는 아직 M1 기준선의
승인된 기본 동작으로 간주하지 않는다.

## 2. 검증 수준

| 수준 | 목적 | 현재 환경 실측 | 실행 시점 |
|---|---|---:|---|
| Gate 0 | 경로·버전·의존성 확인 | 1분 이내 | 모든 작업 시작 전 |
| Gate 1 | 순수 로직 회귀검사 | 약 6초 | 모든 코드 변경 후 |
| Gate 2 | 데이터→BC/PPO→평가→그림 전체 smoke | 약 28초 | 기능 변경 후 |
| Gate 3 | M1 학습·평가 오케스트레이션 smoke | 약 12초 | 실험 코드 변경 후 |
| Gate 4 | 기존 M1 산출물 감사 | 1~2분 | 결과 인용·보고 전 |
| Gate 5 | 전체 M1 재현 | 장시간, 장비 의존 | 릴리스 또는 기준선 재생성 시 |

Gate 1~3의 시간은 2026-09-30 현재 개발 환경에서 측정한 참고값이다. 다른 CPU,
PyTorch 버전, 파일시스템에서는 달라질 수 있으며, 시간 자체는 합격 기준이 아니다.

## 3. Gate 0 — 실행 전 점검

저장소 루트에서 AMR-Lite 디렉터리로 이동한다.

```bash
cd amr_lite_spike
python --version
python -c "import numpy, yaml, torch, matplotlib; print('dependencies: OK')"
git status --short -- .
git log -1 --oneline
```

합격 기준:

- Python 3.10 이상이다.
- `dependencies: OK`가 출력된다.
- `amr_lite_spike` 아래에 의도하지 않은 수정이 없다.
- 작업하려는 커밋과 `git log -1` 결과가 일치한다.

저장소 전체의 `docs/`에 untracked 문서가 있어도 AMR-Lite 코드 검증에는 직접 영향을
주지 않는다. 단, `git status --short -- .`에 표시되는 AMR-Lite 변경은 반드시
누가 만든 변경인지 확인하고 보존한다.

의존성이 없다면 프로젝트 전용 환경을 만든다.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
```

## 4. Gate 1 — 단위테스트

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m pytest -q
```

현재 기준 예상 결과:

```text
...........................                                              [100%]
27 passed
```

합격 기준:

- process exit code가 0이다.
- 27개 테스트가 모두 통과한다.
- skip, failure, error가 없다.

predictive shield만 빠르게 확인할 때는 다음을 사용한다.

```bash
PYTHONPATH=. python -m pytest -q tests/test_safety_shield.py
```

여기서 predictive 테스트가 통과하더라도 predictive shield의 전체 성능이 승인된 것은
아니다. 현재 테스트는 속도 추정과 정면 위험에서 회피 또는 후진 후보를 선택하는지를
검사한다.

## 5. Gate 2 — Phase 0 전체 smoke

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.train
```

정상 실행은 다음 다섯 단계를 순서대로 출력한다.

```text
[1/5] Collecting teacher episodes
[2/5] Training BC smoke checkpoint
[3/5] Training PPO smoke checkpoint
[4/5] Evaluating smoke policies
[5/5] Rendering smoke demos
```

필수 산출물:

```text
artifacts/datasets/teacher/
artifacts/checkpoints/bc_smoke.pt
artifacts/checkpoints/ppo_smoke.pt
artifacts/checkpoints/ppo_smoke.training.csv
artifacts/smoke/results/episodes.csv
artifacts/smoke/results/summary.csv
artifacts/smoke/plots/training_curve.png
artifacts/smoke/videos/lite_rule_baseline_S3.png
artifacts/smoke/videos/bc_S3.png
artifacts/smoke/videos/ppo_S3.png
```

합격 기준:

- 다섯 단계가 모두 끝나고 exit code가 0이다.
- `dataset_chunks`가 6이다.
- BC/PPO checkpoint, CSV, 그래프, 세 데모가 생성된다.
- 출력 JSON을 끝까지 읽을 수 있다.

주의:

- smoke PPO는 256 timestep뿐이므로 성공률이나 수렴 여부는 합격 기준이 아니다.
- PPO 로그의 초기 `latest_reward=nan`은 해당 update에서 종료된 episode가 없을 때
  나타날 수 있다. checkpoint 생성과 후속 평가가 완료되면 smoke 실패로 보지 않는다.
- smoke 실행은 `artifacts/smoke/`와 smoke checkpoint를 갱신한다. `artifacts/`는
  Git 추적 대상이 아니다.

## 6. Gate 3 — M1 오케스트레이션 smoke

전체 M1을 다시 돌리기 전에 seed 분리, warm start, checkpoint, manifest, 보고서 생성
경로를 짧게 검증한다.

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib \
python -m amr_lite.cli.m1 \
  --smoke \
  --training-seeds 7 \
  --test-episodes 1 \
  --output artifacts/runbook-check/m1-smoke
```

현재 기준 예상 진행:

```text
M1 training seeds: [7]
[train 1/1] seed=7
PPO update 1/2
PPO update 2/2
[evaluate] policies=5 test_seeds=1
evaluation progress 30/30
```

필수 산출물:

```text
artifacts/runbook-check/m1-smoke/manifest.json
artifacts/runbook-check/m1-smoke/resolved_config.json
artifacts/runbook-check/m1-smoke/run_result.json
artifacts/runbook-check/m1-smoke/REPORT.md
artifacts/runbook-check/m1-smoke/test/episodes.csv
artifacts/runbook-check/m1-smoke/test/summary.csv
```

합격 기준:

- exit code가 0이다.
- selection seed 20000과 test seed 30000이 겹치지 않는다.
- 1개 학습 seed에 대한 checkpoint와 training CSV가 생성된다.
- manifest, report, episode 원자료와 summary가 모두 생성된다.

이 smoke 결과의 성능 수치는 M1 연구 결과를 대체하지 않는다.

## 7. Gate 4 — 기존 M1 산출물 감사

기존 기준선 위치:

```text
artifacts/m1/m1-reproducible-baseline-v1/
```

파일 존재와 JSON 유효성을 확인한다.

```bash
test -s artifacts/m1/m1-reproducible-baseline-v1/REPORT.md
test -s artifacts/m1/m1-reproducible-baseline-v1/manifest.json
test -s artifacts/m1/m1-reproducible-baseline-v1/resolved_config.json
test -s artifacts/m1/m1-reproducible-baseline-v1/run_result.json
test -s artifacts/m1/m1-reproducible-baseline-v1/test/episodes.csv
test -s artifacts/m1/m1-reproducible-baseline-v1/test/summary.csv
python -m json.tool artifacts/m1/m1-reproducible-baseline-v1/manifest.json >/dev/null
python -m json.tool artifacts/m1/m1-reproducible-baseline-v1/run_result.json >/dev/null
wc -l artifacts/m1/m1-reproducible-baseline-v1/test/episodes.csv
```

현재 기준 `episodes.csv`는 헤더를 포함해 4,161줄이어야 한다. 사람에게 보고할 때는
다음 두 문서를 함께 확인한다.

```text
docs/M1_RESULTS.md
artifacts/m1/m1-reproducible-baseline-v1/REPORT.md
```

핵심 판정값:

| 정책 | Shield | 성공률 | 충돌률 | Timeout |
|---|---:|---:|---:|---:|
| Rule | OFF | 86.2% | 13.8% | 0.0% |
| BC | OFF | 83.1% | 16.9% | 0.0% |
| Custom Stable PPO best | OFF | 85.4% | 14.6% | 0.0% |
| Custom Stable PPO best | ON | 83.0% | 12.5% | 4.5% |
| Custom Stable PPO final | OFF | 77.2% | 22.8% | 0.0% |

다음 항목은 현재의 알려진 실패이며 산출물 손상으로 오인하지 않는다.

- D1: custom PPO best가 Shield OFF/ON 모두 동적 충돌 100%
- D0: Shield OFF 충돌 17%, Shield ON 0%
- S3: Shield ON에서 36%가 `SHIELD_SATURATION` timeout
- best와 final checkpoint 사이의 성능 저하

다만 동일 코드·설정·seed로 기준선을 재실행했는데 위 값이 달라지면 회귀 또는 환경
차이로 기록하고 manifest의 Python, PyTorch, dependency, source/checkpoint hash를
비교한다.

## 8. Gate 5 — 전체 M1 재현

전체 재현은 학습 seed 5개를 각각 100,000 timestep 학습하고, 분리된 selection/test
seed로 4,160 episode를 평가한다. 일상적인 코드 확인에는 사용하지 않는다.

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib python -m amr_lite.cli.m1
```

기본 출력 위치:

```text
artifacts/m1/m1-reproducible-baseline-v1/
```

동작 규칙:

- seed, timestep, checkpoint config가 맞는 완료 checkpoint는 재사용한다.
- 완전한 재학습이 필요할 때만 `--no-resume`을 사용한다.
- 기존 기준선을 보존해야 한다면 `--output`으로 새 디렉터리를 지정한다.
- 실행 중단 후 재개할 때는 같은 output을 사용하고 `--no-resume`을 넣지 않는다.

권장 새 출력 예시:

```bash
PYTHONPATH=. MPLCONFIGDIR=artifacts/.matplotlib \
python -m amr_lite.cli.m1 \
  --output artifacts/m1/m1-reproducible-baseline-v2
```

합격 기준:

- 학습 seed가 `7, 17, 29, 43, 59`다.
- 각 full checkpoint가 100,000 timestep과 98 update를 기록한다.
- selection과 test seed가 겹치지 않는다.
- test `episodes.csv`가 4,160 episode를 포함한다.
- best와 final 결과가 별도 행으로 남는다.
- manifest에 dataset/checkpoint hash와 실행 환경이 기록된다.
- `REPORT.md`, summary, scenario별·training-seed별 summary가 생성된다.

## 9. 결과 판정 순서

결과를 볼 때는 아래 순서를 지킨다.

1. 실행이 끝났는지 확인한다: exit code, 필수 파일, JSON/CSV 유효성.
2. 재현 조건을 확인한다: commit, dirty scope, config, seed, checkpoint hash.
3. 안전을 확인한다: collision, minimum clearance, false negative.
4. 가용성을 확인한다: success, timeout, shield saturation.
5. 안정성을 확인한다: training seed 분산, best-final 차이.
6. 마지막으로 속도를 확인한다: episode time, inference time, simulation throughput.

충돌률만 낮아지고 timeout이 늘어난 결과를 안전성 향상으로 단독 해석하지 않는다.
또한 smoke 결과를 학습 성능이나 일반화 성능으로 인용하지 않는다.

## 10. 자주 발생하는 문제

### `ModuleNotFoundError: amr_lite`

`amr_lite_spike`에서 실행했는지와 `PYTHONPATH=.`을 확인한다. editable install을
사용했다면 활성화된 가상환경도 확인한다.

### Matplotlib cache 또는 권한 오류

모든 그래프 명령에 다음 환경값을 붙인다.

```bash
MPLCONFIGDIR=artifacts/.matplotlib
```

### `Teacher dataset not found`

Gate 2를 먼저 실행하거나 teacher dataset만 다시 만든다.

```bash
PYTHONPATH=. python -m amr_lite.learning.collect_teacher \
  --episodes 6 \
  --output artifacts/datasets/teacher
```

### M1이 예상과 다른 checkpoint를 재사용함

먼저 `run_result.json`, checkpoint의 metrics JSON, `resolved_config.json`을 비교한다.
기준선을 지우지 말고 새 `--output`을 지정해 재실행한다. 의도적으로 전부 다시
학습할 때만 `--no-resume`을 사용한다.

### 테스트는 통과하지만 성능 수치가 달라짐

단위테스트는 통계적 성능을 보장하지 않는다. 다음을 순서대로 비교한다.

1. Git commit과 AMR-Lite scope dirty 상태
2. `resolved_config.json`
3. dataset SHA-256
4. checkpoint SHA-256
5. Python, PyTorch, NumPy 버전
6. selection/test seed와 scenario randomization

## 11. 실행 기록 템플릿

검증 결과를 PR, 이슈 또는 연구 노트에 남길 때 아래 형식을 사용한다.

```text
AMR-Lite verification
- Date/time:
- Operator:
- Host / Python / PyTorch:
- Git commit:
- AMR-Lite scope dirty: yes/no
- Gate 1: PASS/FAIL, tests, elapsed
- Gate 2: PASS/FAIL, output path, elapsed
- Gate 3: PASS/FAIL, output path, elapsed
- M1 artifact audit: PASS/FAIL
- Shield strategy: legacy/predictive
- Deviations from baseline:
- Known failures observed:
- Artifact/report path:
```

## 12. 관련 문서

- `README.md`: 설치, 구성, 개별 학습·평가 명령
- `docs/RESULTS.md`: Phase 0 실험 결과와 해석
- `docs/M1_RESULTS.md`: M1 재현성 평가의 공식 요약
- `docs/DECISIONS.md`: 설계 선택과 근거
- `docs/KNOWN_LIMITATIONS.md`: 현재 주장할 수 없는 범위
- `docs/NEXT_STEPS.md`: Phase 1 이후 후보 작업
- `artifacts/m1/m1-reproducible-baseline-v1/REPORT.md`: 생성된 M1 보고서
- `artifacts/m1/m1-reproducible-baseline-v1/manifest.json`: 실행·hash·환경 증적
