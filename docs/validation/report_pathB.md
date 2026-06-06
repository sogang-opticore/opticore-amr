# Path B — perception 재측정 리포트 (v5, fused_tracker)

**작성**: 2026-06-06 ~16:22–16:32 UTC · **브랜치**: fix/amcl-localization · Path A(PASS) 이후 진행 · 캡 내(~10분)

> 전제: Path A에서 laser_max_range 12→25 open-area 정지 localization 검증 **PASS**(도크 dwell 0.0cm).
> Path B는 그 위에서 **perception(YOLO+fused_tracker) 출력 품질**이 깨끗해졌는지 재측정.

---

## 결론: ⚠️ 부분 PASS — localization은 perception 부하에도 안정, 그러나 **트랙 품질은 여전히 garbage**

- **localization(A): PASS** — perception 5노드 동시 가동 + GPU 부하에도 도크 완전정지 dwell **map→odom 0.0cm**.
  GPU 경합 없음(757MiB/32GB, util 3%, lidar 9.9Hz 유지 — RTF 열화 없음).
- **perception 트랙(D/s1): FAIL** — laser fix가 **perception 출력 오염을 제거하지 못함**.
  음수좌표 13/44, 드리프트 최대 13.9m, s1(LiDAR-only) 92.5%로 여전히 LiDAR 팬텀 클러스터 도배.
- → **AMCL 정지 수렴은 고쳤으나, perception garbage의 잔존 원인은 (a)주행 중 map→odom 보정(이동 중 범위 7.98m)으로
  정적 LiDAR 클러스터가 map 프레임에서 번짐 + (b)카메라 융합 실패(YOLO 탐지 거의 0 → s1 단독 지배).**

---

## torch/GPU 환경 (§12.2)
- torch **2.12.0+cu130**, `cuda.is_available()=True`, **device cap (12,0) = sm_120 Blackwell**(RTX PRO 4500).
- 512×512 matmul on cuda 정상(`c.sum()` 유한값) → **GPU 커널 실측 OK. cu128 재설치 불필요**(cu130이 sm_120 지원).
- numpy **1.26.4**(<2 핀) — torch GPU matmul·ultralytics·cv2 임포트 정상(opencv 4.13 numpy>=2 경고는 임포트 성공, 무해).
- YOLO `yolov8n.pt`(/workspace/models) 로드 성공, **inference 7–17ms = GPU 추론**(CPU면 100ms+), FPS ~15.

## 기준별 결과 (analyze_tracking_bag.py — `analyze_perc.txt`)

| 기준 | 결과 | 수치 |
|---|---|---|
| **A) 정지 localization 점프 <2cm** | ✅ PASS | 도크 dwell(t40–50s) **0.0cm**. 플래그(t50–55 16.1cm)는 턴어라운드 회전 아티팩트(Path A와 동일) |
| **B) <1s 유령 낮음** | ✅ PASS | 2/80 (2%) |
| **D) 음수좌표 0** | ❌ FAIL | **13/44** 창고 밖(예: (-3.22,3.82),(14.52,-5.17),(4.81,-6.58)) |
| **D) 드리프트 <3m** | ❌ FAIL | 다수 3–14m (id70 13.9m, id42 12.0m, id103 9.4m, id71 7.8m …) |
| **s1(LiDAR-only) 비율 급감** | ❌ FAIL | s1=74/80 (**92.5%**), s2(cam)=1, s3(LiDAR+cam)=5 |

- **C) 동시 트랙 수**: 주행 중 avg 4–11(피크 12), 정지 복귀 후 avg ~1–2. 트랙 폭증은 주행 구간 집중.
- **카메라 융합 사실상 미작동**: YOLO 탐지 거의 0(detections 218프레임 0 / 28프레임 1 / 1프레임 2) →
  cam/LiDAR 융합(s3) 5건뿐. 창고 객체(forklift/person/shelf)를 yolov8n이 거의 못 잡음(도메인 미스매치/카메라 토픽·FOV).

## §12.3(AMCL 미수정 baseline) 대비
- **직접 수치 비교 불가** — 저장소/`/workspace/validation/`에 AMCL-미수정 baseline analyze 출력이 **커밋·보존되어 있지 않음**
  (raw `t7_fused.log`만 존재, 정량 비교 불가). 따라서 "garbage 사라짐" 정량 판정은 baseline 부재로 **미확정**.
- 절대값 기준으로는: 정지 localization은 깨끗(0.0cm)하나 **트랙 음수좌표·고드리프트·s1 지배 garbage는 잔존**.

## 핵심 수치 요약
- GPU(perception ON): 757MiB(gazebo 309 + YOLO 428), util 2–3%, lidar 9.9Hz, **RTF 열화·경합 없음**.
- tracked_objects 8.9Hz, frame_id=map(계약 OK), 228s, 2027 메시지, 고유 id 80(내부 ~141 생성).
- 정지(도크 dwell) map→odom: **0.0cm**. 주행 중 범위 x 7.98m(이동 중 odom 보정, 정상이나 트랙 번짐 원인).
- 음수좌표 13/44, 드리프트 최대 13.9m, s1 92.5%, <1s 유령 2%.

## 바꾼 것
- **repo 코드 0건.** 환경 한정: `numpy<2`(1.26.4) 핀(perception용). bag 메타데이터 1건 수동 복구
  (recorder -9 조기 종료로 metadata.yaml 미생성 → sqlite 인트로스펙션으로 재구성, 데이터 무손실 2027 트랙).
- max_beams 사다리 미발동(정지 점프 0.0cm).

## 산출물 (`/workspace/validation/`, db3 커밋 X)
- `tuned_run3_perc/`(228s, 트랙 2027 + /tf), `analyze_perc.txt`(= 이 분석), `t6_perception_v5.log`, `t7_fused_v5.log`,
  `route_run_perc.log`, `bag_record_perc.log`.

## 권고 (perception garbage 후속 — 별건)
1. **LiDAR-only 팬텀 억제**: fused_tracker가 정적 구조물(벽/선반)을 unknown s1 트랙으로 양산 →
   클러스터 필터(맵 정적영역 마스킹/최소 confirm source) 강화. s1 단독 트랙 신뢰도 하향.
2. **카메라 융합 복구**: YOLOv8n이 창고 객체 거의 미탐 → 모델/클래스 매핑·카메라 토픽(`/camera`)·FOV 점검.
   (s3 융합 5건뿐이라 fused_tracker의 cam 보정이 사실상 부재.)
3. **주행 중 localization 보정 평활화**: 이동 중 map→odom 범위 7.98m → 트랙 map 좌표 번짐. 정지는 0.0cm로 OK이나
   주행 중 보정 점프가 트랙 드리프트로 전이. (laser fix 범위 밖, AMCL 모션모델/관측 가중 별도 튜닝.)
