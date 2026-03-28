# Git Workflow 가이드

## 브랜치 전략 (Git Flow)

```
main        ── 안정 버전 (배포용)
  └── dev   ── 개발 통합 브랜치
       ├── feat/slam-node
       ├── feat/nav2-setup
       └── feat/yolo-integration
```

## 브랜치 규칙
- `main`: 직접 push 금지, PR + 리뷰 필수
- `dev`: 기능 브랜치에서 PR로 merge
- `feat/*`: 기능별 개발 브랜치

## 커밋 컨벤션

```
<type>: <description>

Types:
  feat     새 기능
  fix      버그 수정
  docs     문서 변경
  refactor 리팩토링
  test     테스트 추가
  chore    빌드/환경 설정
```

예시:
- `feat: add SLAM node with slam_toolbox`
- `fix: resolve nav2 costmap crash`
- `docs: update environment setup guide`

## 작업 흐름

### 1. 새 기능 시작
```bash
git checkout dev
git pull origin dev
git checkout -b feat/my-feature
```

### 2. 작업 & 커밋
```bash
git add <files>
git commit -m "feat: add new feature"
git push origin feat/my-feature
```

### 3. PR 생성
- GitHub에서 `feat/my-feature` -> `dev` PR 생성
- 팀원 1명 이상 리뷰 요청
- 리뷰 승인 후 merge

### 4. 정리
```bash
git checkout dev
git pull origin dev
git branch -d feat/my-feature
```

## 충돌 해결
```bash
git checkout dev
git pull origin dev
git checkout feat/my-feature
git merge dev
# 충돌 해결 후
git add .
git commit -m "fix: resolve merge conflicts with dev"
git push origin feat/my-feature
```
