# 저장소 전용 Notion hub

`NOTION_MODE=enabled`일 때만 읽는다. Notion MCP가 없거나 아래 과정이 실패하면 curl이나 브라우저로 우회하지 않고, GitHub를 수정하기 전에 bootstrap을 중단한다.

## 만들기

1. Notion MCP로 부모 [repo wiki](https://app.notion.com/p/39c3bf06830080b5a024c7ad91855240)와 [공통 가이드](https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee)를 읽는다.
2. 부모의 직계 자식 중 제목이 저장소 이름과 정확히 같은 페이지를 찾는다. 0개면 만들고, 1개면 재사용하고, 2개 이상이면 중단한다. 부분 일치나 전체 검색으로 대신하지 않는다.
3. hub의 직계 자식 `Project Info`도 같은 규칙으로 처리한다. 만들 때는 공통 템플릿을 쓰고, 재사용할 때는 덮어쓰지 않는다.

## 메타데이터

hub에 아래 형식의 섹션을 정확히 하나 둔다.

```markdown
## Repository metadata

- GitHub: https://github.com/<owner>/<repo>
- Purpose: <REPO_PURPOSE와 동일한 값>
- Common guide: https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee
- Project Info: <직계 자식 Project Info URL>
```

섹션이 없으면 추가한다. 하나면 없는 항목만 추가한다. 같은 항목이 중복되거나 값이 다르면 덮어쓰지 않고 중단한다. 다른 내용은 삭제하거나 옮기지 않는다.

## 확인과 기록

1. 다시 읽어 네 항목, 부모, 제목, 서로 다른 페이지 ID, `Project Info`의 직계 관계를 확인한다.
2. 통과하면 hub URL을 `NOTION_REPO_URL`, 자식 URL을 `NOTION_PROJECT_INFO_URL`로 쓴다.
3. `README.md`, `AGENTS.md`, `ops/runner/repo-context.sh`의 `<NOTION_REPO_URL>`만 hub URL로 바꾼다. `Project Info` URL은 이 파일들에 쓰지 않는다.

스크립트는 Notion을 읽지 않고 URL 형식과 위치만 검사한다. 부모·제목·자식 관계는 MCP 확인이 기준이다.
