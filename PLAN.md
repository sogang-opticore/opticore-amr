# PLAN — CCTV No-Go Overlay (Path B) · `feature/cctv-nogo-overlay`

CCTV perception → 바닥투영 → no-go 셀을 **`/dynamic_obstacle_layer`(OccupancyGrid)** 에 발행.
A\* 가 이미 이 레이어를 plan grid 에 합성 → **amr_perception 만 수정**, amr_navigation 불변.
데모: 로봇 LiDAR 사각(코너 너머) 좌핀치 복도 안 person_1 을 CCTV 가 보고 미리 no-go → A\* 가 진입 전 우회.

---

## 1. EXPLORE 실측 결과 (코드 전 — 전부 sim 정량 확인 완료)

### 1.1 `/dynamic_obstacle_layer` 계약 (astar_node.py + dwa_node.py 역공학, ★최중요)
- **타입** `nav_msgs/OccupancyGrid`. **QoS** depth=1, **RELIABLE, VOLATILE** (A\* 구독과 일치).
- **점유 임계 = 65** → no-go 셀에 **100** 발행, free=0.
- **해상도 = /map 과 1e-6 내 일치 필수** (불일치 시 A\* 가 `ValueError`→무시). /map res = **0.05** 확인.
- **full-size 또는 cropped 모두 지원**: A\* 가 `(dyn.origin − map.origin)/res` 오프셋으로 재투영.
  → 본 구현은 **/map.info 통째 복제(full-size)** → origin 동일 → col0=row0=0 → 오프셋 산술 버그 0.
- **freshness**: `header.stamp` 이 A\* **sim clock** 기준 `timeout(35s)` 내여야 usable.
  → 오버레이 노드 **use_sim_time=true** + 노드 clock 으로 stamp.
- **매 메시지 = 전체 교체**(A\* 가 static_inflated 복사 후 재페인트) → **decay = 다시 발행으로 표현**.
- **dynamic 셀은 inflation 없이 raw 페인트** → 디스크 반경이 복도를 물리적으로 막을 만큼 커야 함.
- **DWA 도 같은 토픽 발행자** but `_publish_dynamic_obstacle_layer()` 는 **자기 track 있을 때만** 발행
  (track 없으면 early-return, inactive 전환 시 1회 clear 후 침묵). CCTV 데모는 로봇 LiDAR 가 코너 너머
  사람을 못 봐 **DWA 침묵 → CCTV 가 유일 활성 발행자**. 두 발행자 공존 OK(spec 의도 = path B decoupled).
  → 본 노드도 DWA 패턴 모방: 활성 시 발행, 소멸 시 짧게 clear 발행 후 침묵(DWA 정상 장애물 안 지움).
- **DWA 가 곧 정답 포맷 레퍼런스**: frame_id='map', info=static_map.info, value=100, world(=map프레임)→cell
  `floor((mx−origin.x)/res)`. 그대로 미러.

### 1.2 카메라 (warehouse.world §CCTV + camera_info 실측)
- 4대 **640×480**, FOV 1.658rad, **~15.6 Hz**(GATE-C0 PASS), encoding **rgb8**.
- **camera_info K 실측: fx=fy=293.244, cx=320, cy=240** (해석값 293 과 일치). camera_info 우선, 없으면 이 값.
- **world-프레임 extrinsics**(pose `x y z roll pitch yaw`):
  - `corridor_1s` 14.5 5.5 4.49 0 1.1345 1.5708 | `corridor_1n` 14.5 18.5 4.49 0 1.1345 −1.5708
  - `corridor_2s` 49.5 11.5 4.49 0 1.1345 1.5708 | `corridor_2n` 49.5 24.5 4.49 0 1.1345 −1.5708
- gz 카메라 광축 = link **+X**, image-right=−Y, image-down=−Z. pitch=π/2 면 수직하향 → pitch=1.1345=수직서 정확히 25°(설계의도 확인).
- person_1 spawn **(14.0, 18.0)**. **1n 이 검출 카메라**(거리 ~3.7m<10m clip). **1s 는 ~13m>10m far-clip → 미검출**.

### 1.3 검출기 락 = **BG-subtraction** (YOLO 프레임 테스트로 확정)
- YOLOv8n: person_1 을 **"bird" 0.42 로 오분류**(23px 급경사 top-down) — person 클래스 0건. → YOLO 불가.
- **BG-sub(빈 복도 기준프레임 abs-diff, gray, thresh~30, open+dilate, 최대 컨투어, foot=bbox 하단중앙)**:
  person_1 검출 bbox(346,318,26,23) → 투영 **world(13.932,18.029)** vs GT(14,18) = **0.074m**(GATE-C2 여유). bg-vs-bg=**0 검출**(C6).

### 1.4 좌표 파이프라인 (검증 완료)
1. pixel(u,v) → `ray_link=(1, −(u−cx)/fx, −(v−cy)/fy)`; `R=Rz(yaw)·Ry(pitch)·Rx(roll)`; `dir=R·ray_link`;
   z=0 교점 → **world(Xw,Yw)** (해석+실측 0.07m 일치).
2. world→map(실측): **map = world − (2.96, 15.12)** ⇒ `map_offset=(−2.96,−15.12)`.
   ※ spec 가설 (3.18,16.9) 은 **오류**(그건 grid→mapframe origin). fleet 검증값 (3.0,15.0)≈실측과 일치.
3. map→cell: `col=floor((mx−origin.x)/res)`, `row=floor((my−origin.y)/res)`; origin=(−3.18,−16.9), res=0.05.
   - 검산: person world(13.93,18.03)→map(10.97,2.91)→cell(283,396), 복도 col 277–312 내부. ✔
- /map = **1207×840**, res 0.05, origin (−3.18,−16.9).

### 1.5 런치/토픽 (실측)
- `robot.launch.py` 단독 = **버그**(`use_sim_time` LaunchConfig 부재로 전체 teardown). 수정 불가(amr_bringup).
- **검증용 단일로봇 = `validation/cctv/single_robot.launch.py`**(비커밋): multi_robot 의 검증된
  `make_robot_nodes('amr1',…)` import → 1대 구성(`/map` remap 포함, use_sim_time 하드코딩) + /cctv bridge.
- 데모 토픽(amr1 네임스페이스): **`/amr1/map` `/amr1/goal_pose` `/amr1/global_path` `/amr1/dynamic_obstacle_layer`**.
- `/cctv/*` 는 ign 센서 실재 but **ROS 브리지 누락** → 내 launch 가 parameter_bridge 추가(perception 스코프).

---

## 2. 산출물 (amr_perception/ 만 — §8.1 화이트리스트)

### `amr_perception/cctv_detector.py` (P-9)
- 구독 `/cctv/{cam}/image`(param 카메라 리스트, 기본 4대). 디코드 `np.frombuffer`(rgb8, **cv_bridge 미사용 — NumPy2.x 회피**).
- per-camera **frozen background**: warmup N프레임 평균 후 고정. (static 게이트=warmup 중 person 텔레포트 아웃; dynamic=입장 전 자연 빈배경.)
- gray abs-diff → thresh → MORPH open+dilate → contours → min_area 필터 → blob(들).
- 발행 `/cctv/detections` (**vision_msgs/Detection2DArray**, 신규 메시지 X) — `header.frame_id=카메라명`, 각 Detection2D=bbox.

### `amr_perception/cctv_nogo_overlay.py` (P-10)
- 구독 `/cctv/detections`, `/map`(TRANSIENT_LOCAL→info 캐시), `/cctv/{cam}/camera_info`(옵션→K).
- extrinsics(world pose) param 맵(카메라명→pose). projection 모듈로 foot pixel→world→map→저장(타임스탬프).
- 타이머 `publish_rate_hz`(5): 오래된 점 만료(decay_timeout) → 클러스터 → **min_persist** 충족 클러스터만
  full-size OccupancyGrid(/map.info 복제)에 반경 `nogo_radius_m` 디스크=100 페인트 → frame=map, stamp=now → `layer_topic` 발행.
  활성→비활성 전환 시 clear grid 짧게 발행 후 침묵(DWA 공존).
- params: `layer_topic`(/dynamic_obstacle_layer), `map_topic`(/map), `map_offset_{x,y}`(−2.96,−15.12),
  `nogo_radius_m`(0.9), `decay_timeout_sec`(1.5), `publish_rate_hz`(5), `occupied_value`(100), `min_persist_*`.

### `amr_perception/cctv_projection.py` (P-10 코어, 순수함수 단위테스트)
- `rot_rpy`, `project_pixel_to_ground(u,v,pose,K,z=0)`, `world_to_map`, `map_to_cell`.
- test: 1n 광축(320,240)→(14.5,~16.41); foot(359,341)→(~13.93,~18.03); 상향 ray→None.

### `config/cctv_nogo_params.yaml`, `launch/cctv.launch.py`(/cctv bridge+P-9+P-10), `setup.py`(entry_points 2개 추가).
### 검증 스크립트·런치·기준프레임 = `/workspace/validation/cctv/`(비커밋).

---

## 3. 검증 게이트 → 스크립트 매핑 (PASS=정량만)
- **C0** sensor live: 4×~15Hz ✔(실측). | **C1** 검출: 1n person_1 매프레임 검출률.
- **C2** 투영 <0.5m: 0.074m ✔(코어 실측). | **C3** 오버레이 발행+포맷일치: A\* `dynamic_layer_active_cells>0`, 좌핀치 cell.
- **★C4** 리루트 delta: 오버레이 ON→`/global_path` 좌핀치(x≈14.5) 회피 + gap2(x≈21.5) 경유 + 골도달; OFF→좌핀치 사용. 둘 bag.
- **★C5** decay/재개방: person 제거/검출중단 → 오버레이 소멸 → 경로 좌핀치 재사용.
- **C6** false-positive 0: 빈 복도 spurious=0 ✔(bg-vs-bg 0).

## 4. 리스크/완화
- **오프셋 미세오차**: 실측 0.1m 내, 디스크 0.9m + 복도 병목 → C4 견고. 게이트서 시각확인 후 param 미세조정.
- **DWA 공존 클로버**: 데모서 DWA 침묵. 5Hz 연속 재발행으로 순간 clobber 자가복구.
- **bbox foot ≠ 발**: 급경사뷰서 작은 bbox. min_area+persist+decay 로 안정. 투영 0.07m 실증.
- **AMCL 드리프트**(장주행): 게이트별 클린재시작, sim cap ~200s/timeout 900(§5c).
- **막힘 기준**: 같은 게이트 3회/A\* 포맷 불명/amr_navigation 수정 필요 → BLOCKED.md.

## 5. 작업 순서
projection.py(+test) → cctv_detector → cctv_nogo_overlay → config/launch/setup → colcon build(**symlink 금지**)
→ 단위테스트 → 클린재시작+single_robot launch → C0–C3 → **C4 ON/OFF delta** → C5 → C6 → 게이트별 커밋(amr_perception 명시경로).
