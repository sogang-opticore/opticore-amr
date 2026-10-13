# Git Workflow 가이드

> 2026-10-13 갱신: 실제 운영 방식(브랜치 접두사, 기본 브랜치 상태, 커밋 메시지 형식)에 맞춰 수정.

## 0. 먼저 알아둘 것 — `main`은 오래된 스텁이다

- 실제 통합 브랜치는 **`dev`** 이다. 기능 PR(#19 ~ #27)은 모두 `dev`로 머지되었다.
- GitHub 기본 브랜치(`origin/HEAD`)는 아직 `main`이다. 그런데 `main`은 초기 커밋 직후 `dev`와 갈라진 스텁이다. 2026-10-13 기준으로 `dev`에만 있는 커밋이 258개, `main`에만 있는 커밋이 3개다(README 초안, .gitignore, 초기 bringup).
- 그래서 그냥 `git clone`을 하면 내비게이션·플릿·인지 코드가 없는 `main`을 받는다. **반드시 `dev`로 클론**한다.

```bash
git clone -b dev git@github.com:sogang-opticore/opticore-amr.git
```

- `main`을 정리하는 방법(기본 브랜치를 `dev`로 변경, 또는 `main`을 `dev` 기준으로 재설정)은 팀 결정 사항이다. 두 브랜치가 갈라져 있어 fast-forward로는 맞출 수 없다.

## 1. 브랜치 전략

```
dev                       ── 통합 브랜치 (PR로 머지)
 ├── feature/<topic>      ── 기능 개발   예: feature/fleet-f3-priority, feature/cctv-nogo-overlay
 ├── fix/<topic>          ── 결함 수정   예: fix/amcl-localization, fix/tf-tree-cleanup
 ├── chore/<topic>        ── 정리·환경   예: chore/cleanup-dev-cruft
 └── docs/<topic>         ── 문서 전용 (선택)
main                      ── (현재 미사용 스텁, §0 참고)
```

- 실제로 쓰는 접두사는 `feature/`, `fix/`, `chore/`이다. 예전 문서의 `feat/*`로 만든 브랜치는 없다.
- 기능 브랜치는 `dev`에서 분기하고, 검증 후 PR로 `dev`에 머지한다.

## 2. 커밋 메시지

`type(scope): 설명` 형식을 쓴다. 설명은 한국어·영어 모두 쓴다.

| type | 용도 | 실제 예 |
|---|---|---|
| `feat` | 새 기능 | `feat(fleet): F-3 ROS2 노드 + config + 독립 launch (additive)` |
| `fix` | 버그 수정 | `fix(nav): DWA robot_radius 0.20→0.36 — ...` |
| `tune` | 파라미터 튜닝 | `tune(nav): A* clearance for 2m pinch demo` |
| `docs` | 문서 | `docs(cctv): PROGRESS Morning Summary + BLOCKED(C4)` |
| `chore` / `style` / `refactor` / `test` | 정리, 스타일, 리팩터링, 테스트 | `style(fleet): deadlock_manager docstring/lint 정리 (로직 불변)` |

scope는 `nav`, `fleet`, `cctv`, `perception`, `sim`, `slam` 등 영역 이름을 쓴다.

## 3. 작업 흐름

### 3.1 새 기능 시작
```bash
git checkout dev
git pull origin dev
git checkout -b feature/my-feature
```

### 3.2 작업 & 커밋
```bash
git add <files>
git commit -m "feat(scope): add new feature"
git push origin feature/my-feature
```

### 3.3 PR 생성
- GitHub에서 `feature/my-feature` → `dev` PR 생성 (base가 `main`이 아닌지 확인)
- 팀원 1명 이상 리뷰 요청
- 리뷰 승인 후 merge

### 3.4 정리
```bash
git checkout dev
git pull origin dev
git branch -d feature/my-feature
```

## 4. 충돌 해결
```bash
git checkout dev
git pull origin dev
git checkout feature/my-feature
git merge dev
# 충돌 해결 후
git add <resolved files>
git commit -m "fix: resolve merge conflicts with dev"
git push origin feature/my-feature
```

## 5. 저장소에 올리지 않는 것

- 빌드 산출물(`build/`, `install/`, `log/`), `Dockerfile`(개인 환경용, `.gitignore`에 등록됨)
- 서버 검증 하네스(`/workspace/validation/...`)는 현재 저장소 밖에 있다. 게이트 재현에 필요한 스크립트는 저장소로 옮기는 것을 권장한다([`server-run-prescription.md`](server-run-prescription.md) 참고).
