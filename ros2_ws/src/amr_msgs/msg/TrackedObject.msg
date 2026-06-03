# 단일 융합 트랙. fused_tracker가 LiDAR/로봇카메라/CCTV를 묶어 map 프레임으로 발행한다.

# --- source 비트마스크 (한 객체를 여러 소스가 동시에 볼 수 있음) ---
uint8 SOURCE_LIDAR=1
uint8 SOURCE_ROBOT_CAM=2
uint8 SOURCE_CCTV=4

uint32 id                       # 트랙 ID (KalmanTrack이 부여, 트랙 수명 동안 유지)
string class_name               # 'person' | 'forklift' | 'unknown'
float32 class_confidence        # YOLO score. class unknown(LiDAR-only)이면 0.0
geometry_msgs/Point position    # map 프레임. 평지 가정이라 z=0
geometry_msgs/Vector3 velocity  # map 프레임 속도. vx, vy 사용 (vz ~ 0)
uint8 source                    # 위 상수의 OR. 예: LiDAR+robot_cam = 3
float32[4] position_covariance  # 2x2 row-major [xx, xy, yx, yy]. 융합 가중/디버깅용
