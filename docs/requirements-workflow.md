# 요구사항 작업 흐름

StrictDoc ON 저장소에만 적용한다. OFF 저장소에서는 이 문서가 필요 없고, 관련 스킬과 검사도 아무 작업을 하지 않는다.

## 시작

`heven.toml`에서 `enabled = true`로 바꾸고 `levels`에 필요한 계층을 고른 뒤 `make requirements-init`을 실행한다. 없는 문서만 만들고 기존 문서는 덮어쓰지 않는다. 계층을 빼면 해당 문서를 어떻게 할지 먼저 검토한다.

계층과 작성 기준은 [요구사항 프로필](../.claude/skills/idea-to-requirements/references/requirements-profile.md)에 있다. `uv run --python 3.11 --no-project python tools/requirements.py <command>`가 모든 명령의 공통 진입점이다. 요구사항 도구 전용 환경에 고정 버전의 StrictDoc을 설치하며 자동으로 올리지 않는다.

## 작성과 검토

`idea-to-requirements`를 명시적으로 호출하여 선택한 계층의 Draft를 작성한다. 수치·조건·예외와 검증 가능한 의무를 기록하며, 불확실한 선택은 COMMENT에 남긴다. 승인 없이 Approved로 바꾸지 않는다. 관계는 가장 가까운 선택 상위 계층에 연결한다. Draft의 미완성 추적성은 허용하지만 존재하지 않는 UID와 순환 관계는 오류다.

`make requirements-check`는 StrictDoc 자체 검사와 HEVEN 검사를 실행하고 HTML은 만들지 않는다. 선택한 계층의 문서가 없으면 오류다. 결과는 터미널과 `build/requirements-report.json`에 남는다. 문서는 필요할 때 `make serve`나 `make docs`로 본다. Actions check는 표시하되 branch protection의 필수 check로 자동 지정하지 않는다.

## 기존 코드를 확장할 때

고정한 원본과 현재 동작을 먼저 확인하고, 요구사항별로 유지·수정·신규·제외를 구분한다. 코드가 있다는 사실만으로 충족이나 Approved로 표시하지 않는다. 유지 기능은 검증 이슈로, 수정·신규 기능은 구현과 검증 이슈로 나눈다. 원본의 알려진 결함을 정답으로 고정하지 않는다.

## Issue → PR 연결

검토된 범위에 `requirements-to-issues`를 호출한다. Issue에는 다음 블록을 하나 두고 각 UID를 한 줄에 기록한다. UID가 없는 Issue도 허용한다. 블록 밖 본문은 자유롭게 편집한다.

```markdown
<!-- heven:requirements:start -->
- STK-001
- SW-001
<!-- heven:requirements:end -->
```

Issue의 블록이 연결의 정본이다. PR의 `Resolves #번호` 등 GitHub closing link로 연결된 같은 저장소 Issue들의 UID 합집합을 PR의 동일 블록에 자동 반영한다. 닫힌 Issue도 연결을 보존한다. 요구사항 COMMENT에 역방향 링크를 쓰지 않는다. 파생 조회는 `build/requirements-report.json`의 `issues_by_uid`에 기록한다.

`make requirements-sync`는 변경을 미리 본다. `uv run --python 3.11 --no-project python tools/requirements.py sync --repo owner/repo --apply`는 PR 본문에 적용한다. `gh` 인증이 필요하다. 연결할 UID가 없는 PR의 본문은 건드리지 않는다. 요구사항을 기본 브랜치에 반영한 뒤 해당 UID를 이슈에 연결한다.

열린 Issue나 PR의 알 수 없는 UID와 모호한 블록은 오류로 보고하고 임의로 고치지 않는다. 해당 PR만 건너뛰고 나머지 PR은 계속 동기화한다. 닫힌 Issue의 오류는 경고만 남긴다. 나중에 요구사항이 바뀌어도 과거 Issue 때문에 동기화가 멈추지 않게 하기 위해서다.

자동 동기화는 기본 브랜치의 도구와 요구사항 정본으로 실행한다. PR head의 코드를 쓰기 토큰으로 실행하지 않는다. 동일 결과에는 쓰기를 하지 않으며, 저장소별 동기화 실행은 직렬화한다. API는 페이지 끝까지 조회하고 일시적 오류는 제한적으로 재시도한다. workflow 토큰은 contents/Issues 읽기와 PR 쓰기만 사용한다. 조직 정책이 PR 쓰기를 막으면 권한 오류를 보고하며 PAT로 자동 우회하지 않는다.

GitHub PR 본문 PATCH는 원자적 부분 수정이나 조건부 쓰기를 제공하지 않는다. 동기화 직전에 본문을 다시 읽어 보존하지만 사람이 같은 순간 저장하는 경우까지 원자성을 보장하지 않는다. 동기화 실행 중 본문 동시 편집은 피하고, 충돌 시 GitHub 편집 이력으로 확인한다.
