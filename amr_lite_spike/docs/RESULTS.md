# Phase 0 Results

## 실행 조건

- 실행일: 2026-09-06
- CPU 단일 process, Python 3.11, PyTorch 2.8
- teacher: 6 episode, 1,041 transition
- BC smoke: 296→256→256→2, 2 epoch
- BC full: 같은 구조, 30 epoch
- PPO smoke: 128 hidden, 256 total timestep, 2 update
- PPO full: 같은 구조, 100,000 timestep, 98 update
- DAgger-lite: BC 방문 위험 상태 4 episode 추가 후 2 epoch 재학습
- 평가: `S0`, `S1`, `S2`, `S3`, `S4`, `S5`, `D0`, `D1`; 각 2 seed
- 모든 policy에 같은 scenario/seed/control frequency/action mapping 사용
- 각 모델을 Safety Shield OFF와 ON으로 각각 평가

## Open-loop BC

| 모델 | validation MSE | v MAE | w MAE |
|---|---:|---:|---:|
| BC smoke | 0.0405 | 0.1866 | 0.0914 |
| BC full | 0.0220 | 0.1237 | 0.0740 |
| BC + DAgger-lite | 0.1895 | 0.4264 | 0.2377 |

DAgger-lite는 open-loop 평균 오차를 악화시켰다. 위험 상태의 분포와 정상 상태 비율이 크게 달라졌기 때문으로 추정된다. 그러나 아래 closed-loop 성공률은 개선됐다. 이 차이는 action MSE만으로 주행 성능을 판단하면 안 된다는 사례다.

## Full-model closed-loop paired evaluation

각 행은 16 episode의 실제 측정값이다.

| Policy | Shield | 성공률 | 충돌률 | Timeout | 평균 시간(s) | 평균 shield 개입/episode |
|---|---:|---:|---:|---:|---:|---:|
| lite_rule_baseline | OFF | 87.5% | 12.5% | 0.0% | 8.14 | 0.00 |
| lite_rule_baseline | ON | 87.5% | 12.5% | 0.0% | 8.28 | 1.75 |
| BC full | OFF | 75.0% | 25.0% | 0.0% | 8.31 | 0.00 |
| BC full | ON | 75.0% | 12.5% | 12.5% | 9.36 | 20.38 |
| BC + DAgger-lite | OFF | 62.5% | 25.0% | 12.5% | 9.98 | 0.00 |
| BC + DAgger-lite | ON | 62.5% | 0.0% | 37.5% | 13.62 | 36.75 |
| PPO full | OFF | 0.0% | 12.5% | 87.5% | 17.20 | 0.00 |
| PPO full | ON | 0.0% | 12.5% | 87.5% | 17.20 | 0.38 |

Full 원자료는 `artifacts/full/results/episodes.csv`, 집계는 `artifacts/full/results/summary.csv`에 있다. Smoke 및 기본 checkpoint 평가는 별도 결과 영역에 둔다.

## 해석

1. 경량 환경, teacher 수집, BC/PPO checkpoint, closed-loop 평가까지 한 저장소에서 실행된다.
2. Rule baseline은 정적 및 횡단 시나리오에서 안정적이지만 `D1` 정면 접근 장애물에서 충돌했다.
3. BC full은 smoke BC보다 open-loop 오차와 closed-loop 성공률이 모두 개선되어 75% 성공했다. 남은 실패는 정적 충돌 2건과 동적 충돌 2건이었다.
4. 한 번의 DAgger-lite 모델은 2 epoch만 학습했으므로 BC full과 동일 학습량 비교가 아니다. Shield 적용 시 충돌 0%였지만 timeout 37.5%가 남았다.
5. PPO full은 smoke보다 충돌을 크게 줄였지만 0% 성공했다. 빈 직선에서도 낮은 전진 속도와 큰 단방향 각속도로 작은 원을 반복했다. Reward가 초기 목표거리 감소를 보상하지만 지속 선회를 직접 벌점하지 않아 생긴 reward hacking으로 판단된다.
6. Shield는 이미 충돌 궤적이 아닌 지속 선회를 교정하지 않는다. Policy 품질과 Safety Shield 역할을 분리해야 한다.
7. Shield 개입률과 성공률을 함께 봐야 한다. 충돌률만 낮아진 결과를 안전한 주행으로 해석할 수 없다.

## 성능

- Rule inference: 약 0.024 ms/step
- BC inference: 약 0.060 ms/step
- PPO inference: 약 0.072 ms/step
- Python headless simulation: 정책별 평균 약 941–1,001 policy step/s

환경과 기기에 따라 달라질 수 있으며 현재 값은 평가 실행의 wall-clock 측정이다.

## 산출물

- success/collision/episode-time 그래프
- PPO smoke/full training curve
- rule, BC, PPO, BC+DAgger 최종-frame 주행 데모
- 실제 checkpoint와 episode 단위 teacher/DAgger dataset

## PPO 후속 실험

초기 full 결과의 단방향 선회 실패를 확인한 뒤 다음 세 실험을 추가했다.

| 실험 | RL timestep | Shield OFF 평가 | 결론 |
|---|---:|---|---|
| Reward-v2 PPO | 100,000 | 성공 0.0%, 충돌 25.0% | path-progress와 회전 penalty만으로 해결되지 않음 |
| BC warm-start PPO final | 100,000 | quick 성공 0.0%, 충돌 100.0% | PPO update가 초기 actor를 파괴 |
| BC warm-start best | **0** | 성공 87.5%, 충돌 12.5% | BC-distilled 초기 actor; PPO 개선이 아님 |

Reward-v2는 path arc-length progress, heading cost, turn cost, time cost와 정적 curriculum을 사용했다. 일부 실패가 `CIRCLING`으로 정확히 분류됐지만 성공률은 개선되지 않았다.

BC warm-start actor는 PPO update 전 quick 평가 `S0/S1/S3`에서 3/3 성공했다. Critic-only 예열, 낮은 fine-tuning learning rate, BC anchor를 적용했지만 100,000-step 최종 정책은 다시 붕괴했다. 평가 callback이 보존한 best checkpoint는 timestep 0이며, 전체 8개 시나리오×2 seed에서 성공률 87.5%를 기록했다. 이는 PPO가 BC를 개선했다는 결과가 아니라 PPO 안정화가 다음 핵심 과제라는 증거다.

- Reward-v2 원자료: `artifacts/reward_v2/results/`
- Best checkpoint 원자료: `artifacts/ppo_best/results/`
- Best checkpoint: `artifacts/checkpoints/ppo_bc_best_full_best.pt`

## Stable PPO 결과

공유 actor/critic encoder가 warm-start actor를 훼손하는 문제를 줄이기 위해 다음을 적용했다.

- actor/critic encoder 완전 분리
- running return mean/variance 기반 value normalization
- target KL 0.01 초과 시 actor update 조기 중단
- critic-only 10,000-step 예열
- BC anchor와 5-update 주기 고정 평가

Stable PPO best는 **51,200 timestep**에서 선택됐다.

| 모델 | Shield | 성공률 | 충돌률 | Timeout |
|---|---:|---:|---:|---:|
| BC full | OFF | 75.0% | 25.0% | 0.0% |
| Stable PPO final (100k) | OFF | 50.0% | 50.0% | 0.0% |
| Stable PPO best (51.2k) | OFF | **87.5%** | 12.5% | 0.0% |
| Stable PPO best (51.2k) | ON | 75.0% | 12.5% | 12.5% |
| Rule baseline | OFF | 87.5% | 12.5% | 0.0% |

Stable PPO best는 BC full보다 성공률이 12.5%p 높고 rule baseline과 같은 성공률을 기록했다. 실패 2건은 모두 `D1` 정면 접근 동적 장애물 충돌이었다. Shield ON에서는 `S4`가 `SHIELD_SATURATION`으로 바뀌어 성공률이 낮아졌다.

따라서 Phase 0에서는 “PPO 실행 가능”을 넘어 실제 PPO update 후 BC보다 나은 checkpoint를 얻었다. 다만 final checkpoint 저하와 동적 장애물 실패 때문에 안정적인 수렴 또는 안전 우위를 주장하지 않는다.

- Stable final: `artifacts/checkpoints/ppo_stable_full.pt`
- Stable best: `artifacts/checkpoints/ppo_stable_full_best.pt`
- 전체 결과: `artifacts/ppo_stable/shield_v2_results/`
