# BLOCKED — GATE-C4 (리루트 헤드라인) · `feature/cctv-nogo-overlay`

**2026-06-20.** CCTV no-go 오버레이(P-9+P-10)는 완성·검증됨(C0–C3,C5,C6 PASS). **GATE-C4(오버레이 ON→A\*가 좌핀치 회피 / OFF→좌핀치 사용, 그 delta)만 시연 불가** — 원인은 **amr_perception 밖(amr_navigation A\* clearance 설정)**. §6.6 규칙대로 강행하지 않고 보고.

## 증상
오버레이 ON/OFF 어느 쪽이든 A\*의 `/global_path`가 **좌핀치(world x≈14.5)를 애초에 안 쓴다** → no-go 유무로 경로 delta가 생기지 않음.
- OFF 기준선(no CCTV, person 물리적 14,18, AMCL 수렴): 복도 y구간(9.5–20.5) 경로 world_x = **7.21**(서측 마진 우회), 14.5 아님.
- ON(CCTV no-go 활성, A\*가 1009셀 소비 확인): 동일 서측 경로.
- 복도 내부 두 점 (14.5,16)→(14.5,20): A\* **NO PATH**.

## 근본 원인 (out of scope — amr_navigation `config/astar_params.yaml`)
좌핀치 복도는 **자유폭 ~1.40m**(x 13.78–15.13, /map 실측). A\* 설정(2026-05-24 튜닝):
- `inflation_radius: 0.50` → inflated-free = 1.40 − 2×0.50 = **0.40m** (셀 중앙 max clearance ~0.20m).
- `preferred_clearance: 1.25`, `clearance_cost_weight: 9.0`, `wall_avoid_clearance: 1.05`, `wall_avoid_cost_weight: 3.0`.

→ 복도 통과 셀당 clearance 페널티 ~9.45 누적. 폭넓은 서측 마진(저 clearance 비용)이 항상 더 싸서 A\*가 핀치를 **회피/불통(NO PATH)**. 사람 크기(0.9m) no-go는 좁은 통로에서만 의미가 있는데 A\*는 좁은 통로를 안 쓰고 넓은 통로만 쓰므로, no-go가 로봇 경로에 절대 안 걸린다.
이는 **spec §5a 전제("좌핀치가 최단경로, 로봇이 사용")와 충돌**. 핀치 미사용은 최근 clearance 상향 튜닝 + map2 핀치 폭의 결과로 추정(CCTV spec 작성 시점 이후 변경).

## 검증된 것 (perception 파이프라인 정상 — 본 작업 스코프 완료)
- **C0** 4 cam ~15.6Hz / **C1** 1n BG-sub person 검출 / **C2** 투영 **0.074m**(<0.5m) / **C3** 오버레이 OccupancyGrid 발행 → **A\*가 1009셀 소비**(`dynamic obstacle layer overlay active: 1009 cells`) / **C5** person 제거→1009→**0** decay / **C6** 빈복도+이동로봇 spurious **0**(range-gate+robot-exclusion 후).
- 즉 **A\*는 CCTV no-go를 정상적으로 plan grid에 합성한다(증명됨).** 로봇 경로가 그 셀을 지나가기만 하면 즉시 리루트됨.

## 시도 (재시도 루프 아님 — 진단 후 정지)
spec start/goal(14.5,5→25), 복도내부 근접 endpoints, ON/OFF 기준선, 1대 AMCL 오프셋 실측. 전부 핀치 미사용 확인. perception 버그 아님(스코프 내 수정으로 해결 불가).

## 추천 (HU / 소유팀 결정 — amr_perception 밖이라 미수행)
C4 시연을 살리려면 셋 중 하나 (모두 본 스코프 밖):
1. **amr_navigation(JW/SW)**: 데모용으로 clearance 완화 — `preferred_clearance`↓(≤0.7) 또는 `clearance_cost_weight`↓ 또는 `inflation_radius`<0.45 → 1.40m 핀치 사용 가능.
2. **map2(US)**: 좌핀치 복도 폭을 >~2m로 확장(inflation+clearance 견디게).
3. **시나리오**: 서측/동측 넓은 우회로를 막아 핀치가 유일 N–S 경로가 되게.
→ 위 중 하나 적용 즉시 **본 CCTV 파이프라인은 무수정으로 리루트 시연 가능**(A\* 소비 이미 검증).

## 재현
- `validation/cctv/scratch/run_gate.sh off` (기준선=서측), `... on` (오버레이, 동일 서측).
- 핀치 폭: `validation/cctv/scratch/`의 width 분석 / `astar_params.yaml` clearance 블록.
