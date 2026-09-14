# Phase 0 Decisions

## D-001: 기존 ROS 코드와 완전 분리

신규 구현은 `amr_lite_spike/` 아래에 두고 `dwa_node.py`를 import하거나 복사하지 않았다. ROS 의존성 없이 테스트하고, 스파이크 폐기/확장 경계를 명확히 하기 위해서다.

## D-002: 72 rays와 4 frames

270° 시야를 72개 ray로 나누면 ray 간격이 약 3.8°라서 Python 구현으로도 smoke 실행 속도를 유지할 수 있다. 최근 4 frame은 oldest → newest 순서로 flatten한다. 20 Hz에서 약 0.15초의 변화가 들어오며, Phase 1의 tracker/RNN 비교를 위한 최소 temporal baseline이다.

## D-003: 원형 footprint와 직사각형 지도

기존 설정의 외접 반경 0.36 m를 원형 footprint로 사용했다. 자세한 차체 자세 충돌보다 substep 충돌, 학습 계약, 평가 일관성을 먼저 검증하기 위한 단순화다. 정적 지도는 axis-aligned rectangle과 외벽으로 구성했다.

## D-004: Gymnasium 호환 API, 직접 의존하지 않음

실행 환경에 Gymnasium이 없어 `reset`, `step`, `render`, `close`, `action_space`, `observation_space` 의미를 유지하는 작은 `Box` fallback을 구현했다. API는 Gymnasium 5-tuple step 계약을 따른다. 정식 단계에서는 Gymnasium checker 통과 adapter를 추가할 수 있다.

## D-005: 외부 PPO 라이브러리 대신 소형 PyTorch PPO

Stable-Baselines3가 설치되어 있지 않고 추가 설치 없이 smoke pipeline을 완성할 수 있도록 Gaussian actor, critic, GAE, clipped objective를 가진 최소 PPO를 작성했다. Full 연구에서는 검증된 라이브러리와 수치 일치 검증이 필요하다.

## D-006: BC warm-start는 보류

BC actor는 `tanh` MLP이고 PPO actor는 Gaussian mean/log-std 구조라 직접 이식은 별도 mapping 검증이 필요하다. 이번 결과는 정확히 `BC`와 `PPO from scratch`로 표기하며 `BC + PPO`라고 부르지 않는다.

## D-007: Safety Shield는 공통 후처리

Rule/BC/PPO 모두 normalized action → scaling → acceleration limit → 0.7초 rollout 검사 순서를 공유한다. 위험 시 50%, 20% 감속 후보를 검사하고 불가능하면 정지한다. 정책 자체와 shield 효과를 분리하기 위해 평가 기본값은 OFF/ON 양쪽이다.

## D-008: Episode 단위 데이터 split

NPZ를 episode/chunk 단위로 저장하고 episode id로 train/validation을 나눈다. 인접 transition이 양쪽 split에 들어가는 누수를 막기 위해 transition shuffle split은 사용하지 않는다.

## D-009: Reward 변경 없음

최초 명세의 progress, goal, collision, near, smooth, path, idle 항을 그대로 사용했다. 100,000-step PPO에서 한 방향으로 작은 원을 돌며 초기 progress를 보존하는 실패가 관찰됐다. 원인을 확인한 뒤 heading/turn 항 또는 potential-based path progress를 비교해야 하므로 이번 결과에 맞춰 reward를 사후 변경하지 않았다.

## D-010: 후진 제외

Phase 0 action mapping은 `0 ≤ v ≤ v_max`를 유지한다. 정면 접근 장애물과 막다른 공간에서 회복 능력이 제한되는 현상은 실제 평가에 나타났으며 Phase 1 판단 항목으로 남겼다.

## D-011: Reward-v2를 별도 실험으로 보존

단방향 선회를 줄이기 위해 Euclidean goal progress를 A* path arc-length progress로 교체하고 heading, turn, time cost를 추가했다. 기존 checkpoint와 결과는 덮어쓰지 않았다. 100,000-step 재학습에서도 성공률은 0%였으므로 reward 변경만으로 문제가 해결됐다고 보지 않는다.

## D-012: BC warm start와 best checkpoint

PPO actor를 teacher dataset으로 먼저 distill하고 random critic이 안정될 때까지 critic head만 예열했다. 이후 낮은 actor learning rate와 BC anchor를 적용했다. 고정 `S0/S1/S3` 평가 callback은 초기 정책 및 5 update마다 평가해 lexicographic `(success, -collision, -time)` 기준 best checkpoint를 저장한다.

이번 run의 best는 timestep 0이었다. 따라서 87.5% full 성공률은 BC-distilled 초기 actor의 성능이며 PPO update가 만든 개선이 아니다. 최종 100,000-step 정책은 다시 붕괴했다.

## D-013: Stable PPO는 actor/critic 표현을 분리

공유 encoder에서는 critic loss가 BC로 초기화한 actor 표현까지 변경했다. Stable PPO는 actor와 critic encoder를 분리하고 return running mean/variance로 value target을 정규화했다. Actor update에는 target KL 0.01 조기 중단을 적용했다.

이 구성의 best checkpoint는 51,200 timestep에서 선택됐다. 전체 평가에서 Shield OFF 성공률 87.5%로 BC full 75%보다 높아, 실제 PPO update 후 개선을 확인했다.

## D-014: 동적 예측 Shield는 유지하되 안전 보장으로 표현하지 않음

Shield rollout에 scripted obstacle 미래 위치를 반영하고 감속이 모두 위험할 때 제한된 좌·우 회피 후보를 검사한다. 위험 감지는 빨라졌지만 전진 전용 로봇을 향해 계속 접근하는 `D1` 충돌은 제거하지 못했다. 이 구현은 보수적 safety filter이지 충돌 회피 보장기가 아니다.
