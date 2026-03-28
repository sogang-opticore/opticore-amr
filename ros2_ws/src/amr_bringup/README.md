# amr_bringup

AMR 시스템 Launch 패키지

## 설명
- 전체 시스템 통합 런치 파일
- 개별 모듈 런치 파일
- 파라미터 설정 YAML
- RViz 설정 파일

## 구조 (예정)
```
amr_bringup/
├── launch/
│   ├── amr_full.launch.py
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   └── perception.launch.py
├── config/
│   ├── nav2_params.yaml
│   └── slam_params.yaml
├── rviz/
└── package.xml
```
