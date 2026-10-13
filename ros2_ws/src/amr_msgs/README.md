# amr_msgs

커스텀 메시지 패키지 (ament_cmake, rosidl).

> 2026-10-13 갱신: 실제 정의된 메시지 기준으로 재작성. 서비스·액션 정의는 아직 없다.

## 메시지

| 메시지 | 내용 | 발행 / 구독 |
|---|---|---|
| `msg/TrackedObject.msg` | 융합 트랙 1개: `id`, `class_name`('person'·'forklift'·'unknown'), `class_confidence`, `position`(map), `velocity`(map), `source` 비트마스크(LIDAR=1, ROBOT_CAM=2, CCTV=4), `position_covariance`(2×2) | — |
| `msg/TrackedObjectArray.msg` | `header`(frame_id = `map`) + `TrackedObject[] tracks` | `fused_tracker` 발행 → `/perception/tracked_objects`, `f3_priority_node` 구독 |
