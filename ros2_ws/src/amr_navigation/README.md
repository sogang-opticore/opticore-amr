# amr_navigation

Nav2 기반 자율 주행 패키지

## 설명
- Nav2 경로 계획 (Global/Local Planner)
- DWB (Dynamic Window B) 로컬 플래너
- Costmap2D 장애물 회피
- Behavior Tree 기반 네비게이션
- Waypoint Following

## 구조 (예정)
```
amr_navigation/
├── launch/
│   └── navigation.launch.py
├── config/
│   ├── nav2_params.yaml
│   └── costmap_params.yaml
├── behavior_trees/
└── package.xml
```
