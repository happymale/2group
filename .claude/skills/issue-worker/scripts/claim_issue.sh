#!/usr/bin/env bash
#
# issue-worker / claim_issue.sh
#
# Ready 이슈 하나를 잡아 작업 시작 상태로 세팅:
#   issue -> branch (<type>/<N>-<slug>) -> 첫 빈 commit -> push -> draft PR (Resolves #N)
#
# 브랜치 프리픽스(type)는 Issue Type에서 매핑:
#   Bug->fix / Task->feat / Experiment->exp / Chore->chore / (없으면) feat
#
# draft PR이 생기면 GitHub Project "PR linked to issue" workflow가 발화해서
# 이슈 Status가 자동으로 In progress로 넘어감. 그래서 이 스크립트는 Project
# 보드를 직접 건드리지 않음 (gh + git 만으로 자족적으로 동작).
#
# repo와 base 브랜치는 현재 clone에서 자동 감지. env로 override 가능:
#   GH_REPO=<owner/repo>  DEFAULT_BRANCH=<branch>
#
# 사용법:
#   claim_issue.sh <issue-number>
#   claim_issue.sh --issue <issue-number>

set -Eeuo pipefail

# 문자 분류와 Bash substring이 UTF-8 code point 기준으로 동작하도록 locale을 고정한다.
# slugify는 locale별 collation이 다른 `가-힣` 범위 대신 `[:alnum:]`을 사용한다.
export LC_ALL=C.UTF-8

die()  { echo "✗ $*" >&2; exit 1; }
info() { echo "→ $*"; }

# --- args ---
ISSUE=""
case "${1:-}" in
  --issue)   ISSUE="${2:-}";;
  -h|--help) sed -n '2,20p' "$0"; exit 0;;
  "")        die "issue 번호 필요. 사용법: claim_issue.sh <issue-number>";;
  *)         ISSUE="$1";;
esac
[[ "$ISSUE" =~ ^[0-9]+$ ]] || die "issue 번호는 숫자여야 함: '$ISSUE'"

# --- preconditions ---
command -v gh >/dev/null || die "gh CLI 없음 (https://cli.github.com)."
gh auth status >/dev/null 2>&1 || die "gh 로그인 안 됨. 'gh auth login' 먼저."
git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "git repo 안에서 실행할 것."
[[ -z "$(git status --porcelain)" ]] || die "커밋 안 된 변경 있음. commit/stash 후 다시."

# --- repo / base branch (현재 clone에서 자동 감지, env로 override) ---
REPO="${GH_REPO:-$(gh repo view --json nameWithOwner --jq .nameWithOwner 2>/dev/null || true)}"
[[ -n "$REPO" ]] || die "repo 판별 실패. GitHub remote가 있는 clone에서 실행하거나 GH_REPO=<owner/repo> 지정."
BASE="${DEFAULT_BRANCH:-$(gh repo view "$REPO" --json defaultBranchRef --jq .defaultBranchRef.name 2>/dev/null || true)}"
[[ -n "$BASE" ]] || die "base 브랜치 판별 실패. DEFAULT_BRANCH=<branch> 지정."

# --- fetch issue ---
info "issue #$ISSUE 확인 중... ($REPO, base=$BASE)"
ISSUE_JSON=$(gh issue view "$ISSUE" -R "$REPO" --json number,title,state) \
  || die "issue #$ISSUE 못 찾음 ($REPO)."
STATE=$(jq -r .state  <<<"$ISSUE_JSON")
TITLE=$(jq -r .title <<<"$ISSUE_JSON")
[[ "$STATE" == "OPEN" ]] || die "issue #$ISSUE 는 $STATE 상태. open 이슈만 잡을 수 있음."

# --- linked open PR reuse (title/type edits와 무관한 stable discovery) ---
LINKED_JSON=$(gh api graphql -f query='
  query($o:String!,$r:String!,$n:Int!){
    repository(owner:$o,name:$r){
      issue(number:$n){
        closedByPullRequestsReferences(first:20){
          pageInfo{ hasNextPage }
          nodes{
            number state isDraft baseRefName headRefName
            repository{ nameWithOwner }
            headRepository{ nameWithOwner }
            closingIssuesReferences(first:20){
              nodes{ number repository{ nameWithOwner } }
            }
          }
        }
      }
    }
  }' -F o="${REPO%%/*}" -F r="${REPO#*/}" -F n="$ISSUE" \
  --jq '.data.repository.issue.closedByPullRequestsReferences | {hasNextPage:(.pageInfo.hasNextPage // false),nodes:(.nodes // [])}') \
  || die "linked PR 조회 실패."

[[ "$(jq -r '.hasNextPage' <<< "$LINKED_JSON")" != true ]] \
  || die "linked PR 목록 pagination 감지. 전체를 검증할 수 없어 중단."
OPEN_LINKED_LINES=$(jq -c '.nodes[]? | select(.state == "OPEN")' <<< "$LINKED_JSON") \
  || die "linked PR 응답 해석 실패."
OPEN_LINKED=()
while IFS= read -r line; do
  [[ -n "$line" ]] && OPEN_LINKED+=("$line")
done <<< "$OPEN_LINKED_LINES"

if [[ ${#OPEN_LINKED[@]} -gt 1 ]]; then
  die "linked PR이 여러 개 열려 있어 canonical PR을 결정할 수 없음."
fi
if [[ ${#OPEN_LINKED[@]} -eq 1 ]]; then
  LINKED_PR="${OPEN_LINKED[0]}"
  LINKED_NUMBER=$(jq -r '.number' <<< "$LINKED_PR")
  LINKED_BASE=$(jq -r '.baseRefName // ""' <<< "$LINKED_PR")
  LINKED_HEAD=$(jq -r '.headRefName // ""' <<< "$LINKED_PR")
  LINKED_HEAD_REPO=$(jq -r '.headRepository.nameWithOwner // ""' <<< "$LINKED_PR")
  LINKED_BASE_REPO=$(jq -r '.repository.nameWithOwner // ""' <<< "$LINKED_PR")
  LINKED_ISSUE_COUNT=$(jq -r --arg repo "$REPO" --argjson issue "$ISSUE" \
    '[.closingIssuesReferences.nodes[]? | select(.number == $issue and .repository.nameWithOwner == $repo)] | length' \
    <<< "$LINKED_PR")

  [[ "$LINKED_BASE" == "$BASE" ]] \
    || die "linked PR #$LINKED_NUMBER invalid: base=$LINKED_BASE (expected $BASE)."
  [[ "$LINKED_HEAD_REPO" == "$REPO" && "$LINKED_BASE_REPO" == "$REPO" ]] \
    || die "linked PR #$LINKED_NUMBER invalid: same-repository PR이 아님."
  [[ "$LINKED_HEAD" =~ ^(fix|feat|exp|chore)/${ISSUE}-[^/]+$ ]] \
    || die "linked PR #$LINKED_NUMBER invalid: issue branch 형식이 아님 ($LINKED_HEAD)."
  [[ "$LINKED_ISSUE_COUNT" -ge 1 ]] \
    || die "linked PR #$LINKED_NUMBER invalid: issue #$ISSUE 를 close/link하지 않음."

  if [[ "$(jq -r '.isDraft' <<< "$LINKED_PR")" == true ]]; then
    LINKED_STATE='draft'
  else
    LINKED_STATE='ready for review'
  fi
  info "PR #$LINKED_NUMBER linked open PR 재사용 (state: $LINKED_STATE, head=$LINKED_HEAD)."
  exit 0
fi

require_single_ready_project_item() {
  local owner="$1" repo_name="$2" issue="$3" project_json project_tsv
  local -a project_lines=()

  if ! project_json=$(gh api graphql -f query='
    query($o:String!,$r:String!,$n:Int!){
      repository(owner:$o,name:$r){
        issue(number:$n){
          projectItems(first:20){
            pageInfo{ hasNextPage }
            nodes{
              project{ number title }
              fieldValueByName(name:"Status"){
                ... on ProjectV2ItemFieldSingleSelectValue { name }
              }
            }
          }
        }
      }
    }' -F o="$owner" -F r="$repo_name" -F n="$issue" \
    --jq '.data.repository.issue.projectItems | {hasNextPage:(.pageInfo.hasNextPage // false),nodes:(.nodes // [])}'); then
    die "Project Status 조회 실패. gh token의 project 읽기 권한과 Project 접근권한 확인."
  fi

  [[ "$(jq -r '.hasNextPage' <<< "$project_json")" != true ]] \
    || die "Project projectItems pagination 감지. 전체 item을 확인할 수 없어 중단."
  project_tsv=$(jq -r \
    '.nodes[] | [.project.number,.project.title,(.fieldValueByName.name // "<unset>")] | @tsv' \
    <<< "$project_json") || die "Project Status 응답 해석 실패."

  while IFS= read -r line; do
    [[ -n "$line" ]] && project_lines+=("$line")
  done <<< "$project_tsv"

  if [[ ${#project_lines[@]} -ne 1 ]]; then
    printf '보이는 Project 항목:\n%s\n' "${project_tsv:-<none>}" >&2
    die "issue #$issue 는 정확히 한 Project에 있어야 함."
  fi

  local project_no project_title status
  IFS=$'\t' read -r project_no project_title status <<< "${project_lines[0]}"
  [[ "$status" == "Ready" ]] || die \
    "issue #$issue Project #$project_no '$project_title' Status=$status. Ready만 새로 잡을 수 있음."
  info "Project #$project_no '$project_title': Ready 확인"
}

require_no_open_blockers() {
  local owner="$1" repo_name="$2" issue="$3" blocked_json open_blockers

  blocked_json=$(gh api graphql -f query='
    query($o:String!,$r:String!,$n:Int!){
      repository(owner:$o,name:$r){
        issue(number:$n){
          blockedBy(first:100){
            pageInfo{ hasNextPage }
            nodes{ number state }
          }
        }
      }
    }' -F o="$owner" -F r="$repo_name" -F n="$issue" \
    --jq '.data.repository.issue.blockedBy') \
    || die "선행 이슈(Blocked by) 조회 실패. 확인할 수 없어 중단."
  [[ "$(jq -r '.pageInfo.hasNextPage' <<< "$blocked_json")" == false ]] \
    || die "선행 이슈 목록 pagination 감지. 전체를 확인할 수 없어 중단."
  open_blockers=$(jq -r '[.nodes[] | select(.state == "OPEN") | "#\(.number)"] | join(", ")' \
    <<< "$blocked_json") || die "선행 이슈 응답 해석 실패."
  [[ -z "$open_blockers" ]] \
    || die "issue #$issue 는 선행 이슈가 열려 있어 시작할 수 없음: $open_blockers"
  info "선행 이슈 확인 완료"
}

# --- Issue Type -> conventional type (branch prefix + PR title) ---
OWNER="${REPO%%/*}"; NAME="${REPO##*/}"
ITYPE=$(gh api graphql -f query='
  query($o:String!,$r:String!,$n:Int!){
    repository(owner:$o,name:$r){ issue(number:$n){ issueType{ name } } }
  }' -F o="$OWNER" -F r="$NAME" -F n="$ISSUE" \
  --jq '.data.repository.issue.issueType.name // ""' 2>/dev/null || echo "")
case "$ITYPE" in
  Bug)        TYPE=fix;;
  Experiment) TYPE=exp;;
  Chore)      TYPE=chore;;
  Task)       TYPE=feat;;
  *)          TYPE=feat;;    # 타입 없거나 못 읽으면 기본 feat
esac
info "issue type: ${ITYPE:-<none>} → prefix: $TYPE"

# --- branch name: <type>/<N>-<slug> ---
slugify() {
  local slug
  if ! slug=$(printf '%s\n' "$1" | tr '[:upper:]' '[:lower:]' \
    | sed -e 's/[^[:alnum:] -]//g' -e 's/[ -]\+/-/g' -e 's/^-//' -e 's/-$//'); then
    return 1
  fi
  printf '%s\n' "${slug:0:40}"
}
if ! SLUG=$(slugify "$TITLE"); then
  die "branch slug 생성 실패. UTF-8 locale과 sed 동작을 확인할 것."
fi
[[ -n "$SLUG" ]] || SLUG="task"
BRANCH="$TYPE/$ISSUE-$SLUG"

# --- effective branch discovery before any local ref mutation ---
# ls-remote로 recovery branch를 먼저 확정해야 title/type 변경 전 branch의
# unlinked open PR도 fetch/switch/push 전에 검사할 수 있다.
info "remote issue branch 확인"
REMOTE_REF_TEXT=$(git ls-remote --heads origin) || die "remote branch 목록 조회 실패."
REMOTE_ISSUE_BRANCHES=()
while IFS=$'\t' read -r _remote_sha remote_ref; do
  remote_branch="${remote_ref#refs/heads/}"
  if [[ "$remote_branch" =~ ^(fix|feat|exp|chore)/${ISSUE}-[^/]+$ ]]; then
    REMOTE_ISSUE_BRANCHES+=("$remote_branch")
  fi
done <<< "$REMOTE_REF_TEXT"

if [[ ${#REMOTE_ISSUE_BRANCHES[@]} -gt 1 ]]; then
  printf 'remote issue branches:\n%s\n' "${REMOTE_ISSUE_BRANCHES[*]}" >&2
  die "issue #$ISSUE remote branch가 여러 개라 복구할 수 없음."
fi
if [[ ${#REMOTE_ISSUE_BRANCHES[@]} -eq 1 ]]; then
  BRANCH="${REMOTE_ISSUE_BRANCHES[0]}"
fi

CONFLICT_COUNT=$(gh pr list -R "$REPO" --head "$BRANCH" --state open \
  --json number --jq 'length') || die "canonical head PR conflict 조회 실패."
[[ "$CONFLICT_COUNT" =~ ^[0-9]+$ ]] || die "canonical head PR conflict 응답 오류."
if [[ "$CONFLICT_COUNT" -gt 0 ]]; then
  die "conflict: $BRANCH 에 issue #$ISSUE 와 linked되지 않은 open PR이 있음."
fi

require_single_ready_project_item "$OWNER" "$NAME" "$ISSUE"
require_no_open_blockers "$OWNER" "$NAME" "$ISSUE"

# --- create/recover issue branch after read-only preflight ---
info "origin/$BASE와 선택된 branch 최신화"
git fetch origin --prune --quiet
if [[ ${#REMOTE_ISSUE_BRANCHES[@]} -eq 1 ]]; then
  if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    git switch "$BRANCH"
    git branch --set-upstream-to="origin/$BRANCH" "$BRANCH" >/dev/null
  else
    git switch -c "$BRANCH" --track "origin/$BRANCH"
  fi
  info "기존 remote issue branch 복구: $BRANCH"
elif git show-ref --verify --quiet "refs/heads/$BRANCH"; then
  git switch "$BRANCH"
else
  git switch -c "$BRANCH" "origin/$BASE"
fi

# --- first empty commit so the branch has a diff vs base (draft PR needs one) ---
AHEAD_COUNT=$(git rev-list --count "origin/$BASE..HEAD")
if [[ "$AHEAD_COUNT" -eq 0 ]]; then
  git commit --allow-empty -m "chore: start issue #$ISSUE ($TITLE)" --quiet
  info "빈 commit 생성 (작업 시작점)"
fi

# --- push ---
info "origin/$BRANCH push..."
git push -u origin "$BRANCH" --quiet

# --- draft PR (Resolves #N drives Status -> In progress) ---
PR_TEMPLATE="$(git rev-parse --show-toplevel)/.github/pull_request_template.md"
if [[ -f "$PR_TEMPLATE" ]] && grep -qx 'Resolves #' "$PR_TEMPLATE"; then
  PR_BODY=$(sed "s/^Resolves #\$/Resolves #$ISSUE/" "$PR_TEMPLATE")
else
  PR_BODY="Resolves #$ISSUE"
fi
if ! PR_URL=$(gh pr create -R "$REPO" --base "$BASE" --head "$BRANCH" --draft \
  --title "$TYPE: $TITLE (#$ISSUE)" --body "$PR_BODY"); then
  die "draft PR 생성 실패. push된 branch는 다음 실행에서 복구됨: $BRANCH"
fi
info "draft PR 생성: $PR_URL"

cat <<EOF

✓ 준비 완료
  issue : #$ISSUE  $TITLE  (type: ${ITYPE:-?})
  branch: $BRANCH
  PR    : draft (Resolves #$ISSUE → Project Status 자동으로 In progress)

다음:
  1. 이 브랜치에서 코딩 (codex / claude code / 직접)
  2. 주기적으로 git commit && git push  (draft PR 자동 갱신 = 진행상황 공유)
  3. 끝나면 GitHub에서 "Ready for review" 클릭 → CI + 리뷰 발화
EOF
