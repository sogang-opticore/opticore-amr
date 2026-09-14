# M1 — Reproducible Evaluation Baseline

## 결론

Phase 0의 Stable PPO best 성공률 87.5%는 randomized test에서 재현되지 않았다.
기존 checkpoint는 Shield OFF 기준 68.8% 성공했고, 새로 학습한 5개 seed의
validation-selected best checkpoint는 평균 85.4% 성공했다. 새 best 정책의
학습-seed별 범위는 83.8–86.2%였다.

따라서 87.5%는 고정 시나리오 8개 중 7개 성공을 두 번 센 결과에 가까우며,
일반화 성공률의 추정치로 사용하지 않는다. 현재 가장 방어 가능한 수치는 새
5-seed best의 85.4%와 계층적 bootstrap 95% CI 84.0–86.5%다.

## Protocol

- 학습 seed: 7, 17, 29, 43, 59
- 각 학습 budget: 100,000 timestep
- checkpoint selection: 시나리오 8개 × randomized seed 5개, test와 분리
- final test: 시나리오 8개 × randomized seed 20개
- Shield OFF/ON paired evaluation
- custom PPO best/final, 기존 Stable PPO best, BC, rule baseline 비교
- 전체 test episode: 4,160
- 같은 scenario/seed의 instance ID를 모든 정책과 shield mode가 공유
- binary/clearance CI: scenario-stratified hierarchical bootstrap 2,000회
- pooled Wilson interval도 원본 summary에 병기

## Aggregate result

| Policy | Checkpoint | Shield | 성공률 (95% CI) | 충돌률 (95% CI) | Timeout (95% CI) |
|---|---|---:|---:|---:|---:|
| Rule | fixed | OFF | 86.2% (84.4–87.5) | 13.8% (12.5–15.6) | 0.0% |
| Rule | fixed | ON | 87.5% | 12.5% | 0.0% |
| BC | fixed | OFF | 83.1% (80.0–85.6) | 16.9% (14.4–20.0) | 0.0% |
| Legacy Stable PPO | legacy best | OFF | 68.8% (63.8–73.1) | 31.2% (26.9–35.6) | 0.0% |
| Custom Stable PPO | 5-seed best | OFF | 85.4% (84.0–86.5) | 14.6% (13.5–16.0) | 0.0% |
| Custom Stable PPO | 5-seed best | ON | 83.0% (79.8–86.1) | 12.5% | 4.5% (1.2–7.8) |
| Custom Stable PPO | 5-seed final | OFF | 77.2% (69.9–84.2) | 22.8% (15.9–29.8) | 0.0% |
| Custom Stable PPO | 5-seed final | ON | 79.9% (73.6–86.0) | 12.5% | 7.6% (1.6–13.6) |

새 custom PPO best와 rule의 Shield OFF paired 성공률 차이는 -0.9%p
(95% CI -2.0–0.0%p)다. best가 rule보다 우월하다는 증거는 없지만, 기존
단일-seed checkpoint보다 재현성이 크게 개선됐다.

## Training-seed stability

| Seed | Best 성공률 | Final 성공률 | Best timestep |
|---:|---:|---:|---:|
| 7 | 85.6% | 66.2% | 81,920 |
| 17 | 86.2% | 86.2% | 100,000 |
| 29 | 85.0% | 71.9% | 92,160 |
| 43 | 83.8% | 75.6% | 92,160 |
| 59 | 86.2% | 86.2% | 81,920 |

Best의 중앙값은 85.6%, 최악은 83.8%다. Final의 중앙값은 75.6%, 최악은
66.2%다. Seed 7과 29는 final이 best보다 각각 19.4%p, 13.1%p 낮다.
checkpoint 의존성은 여전히 해결되지 않았다.

## Failure concentration

- D1: custom PPO best가 Shield OFF/ON 모두 100% 동적 충돌
- D0: Shield OFF에서 custom PPO best 충돌 17%; Shield ON에서 0%
- 정적 S0/S1/S2/S3/S4/S5: custom PPO best Shield OFF는 모두 100% 성공
- Shield ON은 S3에서 36%를 SHIELD_SATURATION timeout으로 변경
- Final checkpoint는 D0, S3, S4에서도 추가 충돌이 발생

M2는 D1의 전면 접근 실패와 S3 shield saturation을 우선 분석한다.

## Reproducibility artifacts

실행 디렉터리의 manifest.json은 Git commit/dirty 상태, source tree hash,
dataset version/hash, 모든 checkpoint hash, 전체 resolved config, seed,
Python·PyTorch·dependency version을 기록한다.

최초 계산은 spike가 Git untracked인 상태에서 수행했지만, M1 소스·설정·테스트를
전용 branch에 고정한 뒤 최종 manifest를 갱신한다. 생성된 dataset, checkpoint,
episode 결과는 Git에 넣지 않고 dataset/checkpoint/source SHA-256으로 식별한다.
