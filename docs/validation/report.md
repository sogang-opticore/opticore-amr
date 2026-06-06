# AMCL/localization 검증 리포트 (v5) — laser_max_range 12→25 (커밋 a2f5fe7)

**작성**: 2026-06-06 ~15:46–16:10 UTC · **브랜치**: fix/amcl-localization · 캡 90분 내(소요 ~24분) · RTF 0.33(정상)

---

## 결론: ✅ PASS — laser-fix open-area localization 검증 완료

`laser_max_range` 12→25 수정이 노린 **open-area(벽>12m) 도크에서의 정지 localization 락**이
**자율주행 유효 bag(왕복 31.25m, 도크 도달)** 에서 확인됨. 정지(도크 dwell) 구간 map→odom 점프 **0.0cm**.
코드 변경 0건(기존 커밋 a2f5fe7 그대로 검증). max_beams 사다리 **미발동**(정지점프 0.0cm < 2cm).

---

## 게이트별 결과

| 게이트 | 결과 | 근거 |
|---|---|---|
| **0 — 환경 헬스** | ✅ PASS | `warehouse.world`가 GPU에 **309MiB G(graphics) 컨텍스트** 점유 = GPU 렌더 중. launch.log `libEGL/Permission denied/llvmpipe` **없음**(clean). lidar ~7Hz, clock ~160Hz → **RTF ~0.33 (이 Pod 정상, soft-render 아님)**. → `gate0_health.txt` |
| **1 — AMCL 정지 락(초기)** | ✅ PASS | 스폰 정지 상태 `map→odom_filtered = [-0.000, 0.000, 0.000]` 12초 변화 없음(<2cm). → `gate1_amcl_lock.txt` |
| **2 — 도크 좌표** | ✅ PASS | `map(2.0, 15.0)`(route 기본 WP) A*에 던짐 → `경로 발행: 1496 웨이포인트, min_clear=0.95m`, `occupied`/`out of bounds` 없음, DWA 수락·주행. → `gate2_dock_coord.txt` |
| **3 — 유효 bag** | ✅ PASS | (v5c) odom→base 누적 **31.25m (>30m)** + 로봇 **map_y 14.87 (도크 도달, 밴드 y>13)**. → `analyze_out.txt` |
| **기준 A — 정지 localization 락** | ✅ PASS(실질) | 도크 **완전정지 dwell 구간 = 0.0cm** (3개 bag 일관). 자동 스크립트의 "A FAIL"은 **턴어라운드 회전을 정지로 오분류**한 아티팩트(아래 §해석). |

---

## 기준 A 정밀 해석 (★ 자동 FAIL ≠ 실질 FAIL — 회전 아티팩트)

analyze_tf_only는 직선속도만 보므로 **제자리 회전을 "정지"로 분류** → 회전 중 AMCL yaw 보정 점프가
`"정지 중 점프!"`로 뜬다(VALIDATION_TASK v5 §해석 캐비엇 명시). t값으로 도크 dwell vs 회전 구분한 결과:

**v5c bag (유효 bag, 31.25m):**
```
t= 40-45s : 0.06m/s ->  17.5cm   (도크로 감속 진입 + 도착 회전)
t= 45-50s : 0.00m/s ->   0.0cm   ← 도크 완전정지 dwell (락 OK)
t= 50-55s : 0.00m/s ->   0.0cm   ← 도크 완전정지 dwell (락 OK)
t= 55-60s : 0.00m/s ->  18.3cm   ← 복귀 goal 발행 시점, 제자리 회전(턴어라운드) — 회전 아티팩트, 무시
```
- 18.3cm 점프는 **복귀(WP2) goal이 발행되며 로봇이 돌아서는 순간**(직선속도 0이나 yaw 변화). 도크 dwell 아님.
- **진짜 판정 창인 완전정지 dwell(t=45–55s) = 0.0cm.** → 기준 A 실질 PASS.

**3개 bag 교차검증(모두 도크 완전정지 dwell = 0.0cm):**
| bag | 성격 | 도크 dwell 정지점프 | gate3 |
|---|---|---|---|
| `tuned_run3_v5` | 복귀편만(bag 늦게 시작) + 도크 **200초 idle** | **0.0cm (200s 연속)** | 15.2m(편도라 FAIL) |
| `tuned_run3_v5b` | 왕복(스폰→도크→스폰) | **0.0cm (t40–50)** | 29.72m(0.28m 미달) |
| `tuned_run3_v5c` | 왕복(재실행) | **0.0cm (t45–55)** | **31.25m PASS** |

→ 모든 정지 dwell에서 map→odom 0.0cm. 자동 "A FAIL"은 전 bag 공통으로 **턴어라운드 회전**에서만 발생
(이 route는 도크서 반드시 돌아서므로 스크립트가 구조적으로 A PASS를 못 찍음 → 캐비엇대로 수동 판정).

## 핵심 수치
- **GPU 점유: O** — warehouse.world 309MiB G 컨텍스트(렌더 중). GPU-util 1%(CPU-bound, 정상).
- **RTF: ~0.33** (참고값, 판정 기준 아님 — v5에서 폐기). lidar ~7Hz / clock ~160Hz.
- **정지(도크 dwell) map→odom 점프: 0.0cm** (3 bag 일관, 기준 <2cm) — **laser-fix 핵심 검증 통과.**
- odom→base 누적 이동: **31.25m** (유효 bag, >30m). 도크 도달 map_y **14.87** (밴드 y>13).
- 주행 중(비정지) map→odom 점프: 최대 25.9cm/5s, x범위 6.46m — **주행 중 odom 드리프트를 AMCL이 보정**하는
  정상 거동(기준은 정지 점프임). 충돌/킥냅 없이 도크 왕복 자율주행 3회 모두 성공.
- (Path B perception: **미실행** — A 실질 PASS로 핵심 목적 달성, 별도 단계. torch/numpy 환경 미구성.)

## 바꾼 것
- **repo 코드 변경 0건.** `amcl_params.yaml`·맵·SDF 미수정. laser fix는 기존 커밋 a2f5fe7에 이미 포함.
- **max_beams 사다리: 미발동** — 발동 조건(유효 bag인데 도크 dwell 정지점프 >2cm) 불성립(실측 0.0cm).
- 환경 한정(repo 무관): `rosbags` pip 설치(분석용). 하네스 스크립트 `validation_route_v5.py`의
  `wait_tf` tries 60→400(TF 리스너 기동 레이스로 초기 6s 내 `map→base_footprint` 미수신 → 주행 시작 못 함.
  주행 방법론 무관, repo 밖 `/workspace/` 파일).

## 커밋 해시 (fix/amcl-localization)
- gate0 `gate0_health.txt` → **1a78b2c**
- gate1 `gate1_amcl_lock.txt` → **dff3c50**
- gate2 `gate2_dock_coord.txt` → **1ecd7cf**
- gate3 `analyze_out.txt` + `report.md` → (이 커밋)

## 산출물 (`/workspace/validation/`, db3·대용량은 커밋 X)
- 유효 bag: `tuned_run3_v5c/`(31.25m, gate3 PASS). 보조: `tuned_run3_v5b/`, `tuned_run3_v5/`.
- `analyze_out_v5c.txt`(= 커밋된 analyze_out.txt), `analyze_out.txt`(v5b), `route_run_v5*.log`,
  `warehouse_launch.log`, `t2_nav.log`, `bag_record_v5*.log`, gate0~2 증거.

## 종료 상태
- 검증 PASS. sim(warehouse+nav) 및 모든 산출물 보존. **Pod 끄지 않음**(가드레일).
- 막힌 곳 없음. (참고 권고: 이중 라이다 정리·amr_navigation 설치 스킴은 검증 후 별건.)
