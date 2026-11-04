#!/usr/bin/env bash

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd -P)
SCRIPT="$ROOT/.claude/skills/issue-worker/scripts/claim_issue.sh"
SKILL="$ROOT/.claude/skills/issue-worker/SKILL.md"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/bin" "$TMP/data"

export REAL_SED
REAL_SED=$(command -v sed)

export GH_CALL_LOG="$TMP/gh-calls.log"
export GH_MUTATION_LOG="$TMP/gh-mutations.log"
export GH_JQ_LOG="$TMP/gh-jq.log"
export GH_BODY_LOG="$TMP/gh-pr-body.log"

cat > "$TMP/bin/gh" <<'FAKE_GH'
#!/usr/bin/env bash
set -Eeuo pipefail

printf '%q ' "$@" >> "$GH_CALL_LOG"
printf '\n' >> "$GH_CALL_LOG"

args=("$@")
jq_expr=''
head_arg=''
for ((i = 0; i < ${#args[@]}; i++)); do
  if [[ "${args[$i]}" == --jq ]]; then
    jq_expr="${args[$((i + 1))]:-}"
  elif [[ "${args[$i]}" == --head ]]; then
    head_arg="${args[$((i + 1))]:-}"
  fi
done

apply_jq() {
  local source="$1"
  if [[ -n "$jq_expr" ]]; then
    printf '%s\n' "$jq_expr" >> "$GH_JQ_LOG"
    jq -c -r "$jq_expr" "$source"
  else
    cat "$source"
  fi
}

case "${1:-} ${2:-}" in
  'auth status')
    exit 0
    ;;
  'repo view')
    if [[ "$*" == *defaultBranchRef* ]]; then
      printf 'dev\n'
    else
      printf 'skku-heven/example\n'
    fi
    exit 0
    ;;
  'issue view')
    jq -nc \
      --argjson number 42 \
      --arg title "${FAKE_ISSUE_TITLE:-Ready task}" \
      --arg state "${FAKE_ISSUE_STATE:-OPEN}" \
      '{number:$number,title:$title,state:$state}'
    exit 0
    ;;
  'pr list')
    count="${FAKE_CONFLICT_COUNT:-0}"
    if [[ -n "${FAKE_CONFLICT_HEAD:-}" && "$head_arg" != "$FAKE_CONFLICT_HEAD" ]]; then
      count=0
    fi
    jq -nc --argjson count "$count" \
      '[range(0; $count) | {number:(501 + .)}]' > "${GH_TMP_JSON:?}"
    apply_jq "$GH_TMP_JSON"
    exit 0
    ;;
  'pr create')
    printf '%q ' "$@" >> "$GH_MUTATION_LOG"
    printf '\n' >> "$GH_MUTATION_LOG"
    for ((i = 0; i < ${#args[@]}; i++)); do
      [[ "${args[$i]}" == --body ]] && printf '%s\n' "${args[$((i + 1))]}" > "$GH_BODY_LOG"
    done
    [[ "${FAKE_PR_CREATE_FAIL:-0}" != 1 ]] || exit 74
    printf 'https://github.com/skku-heven/example/pull/123\n'
    exit 0
    ;;
  'api graphql')
    if [[ "$*" == *closedByPullRequestsReferences* ]]; then
      source="${FAKE_LINKED_JSON:?}"
    elif [[ "$*" == *blockedBy* ]]; then
      [[ "${FAKE_BLOCKED_QUERY_FAIL:-0}" != 1 ]] || exit 75
      source="${FAKE_BLOCKED_JSON:?}"
    elif [[ "$*" == *projectItems* ]]; then
      source="${FAKE_PROJECT_JSON:?}"
    elif [[ "$*" == *issueType* ]]; then
      source="${FAKE_TYPE_JSON:?}"
    else
      printf 'unexpected GraphQL query\n' >&2
      exit 92
    fi
    apply_jq "$source"
    exit 0
    ;;
esac

printf 'unexpected gh call:' >&2
printf ' %q' "$@" >&2
printf '\n' >&2
exit 90
FAKE_GH
cat > "$TMP/bin/sed" <<'FAKE_SED'
#!/usr/bin/env bash
set -Eeuo pipefail

for arg in "$@"; do
  if [[ "${FAKE_SED_FAIL:-0}" == 1 ]]; then
    printf 'sed: simulated execution failure\n' >&2
    exit 1
  fi
  if [[ "$arg" == *'가-힣'* ]]; then
    printf 'sed: simulated Ubuntu invalid collation character\n' >&2
    exit 1
  fi
done

exec "${REAL_SED:?}" "$@"
FAKE_SED
chmod +x "$TMP/bin/gh" "$TMP/bin/sed"
export PATH="$TMP/bin:$PATH"

failures=0
fail() {
  printf 'not ok - %s\n' "$1" >&2
  failures=$((failures + 1))
}

pass() {
  printf 'ok - %s\n' "$1"
}

print_claim_diagnostics() {
  local name="$1" output="$2"
  printf '%s\n' '--- local refs ---' >&2
  git -C "$TMP/$name" for-each-ref refs --format='%(refname) %(objectname)' >&2 || true
  printf '%s\n' '--- remote refs ---' >&2
  git --git-dir="$TMP/$name.git" for-each-ref refs --format='%(refname) %(objectname)' >&2 || true
  printf '%s\n' '--- claim output ---' >&2
  cat "$output" >&2 || true
  printf '%s\n' '--- gh mutations ---' >&2
  cat "$GH_MUTATION_LOG" >&2 || true
}

reset_logs() {
  : > "$GH_CALL_LOG"
  : > "$GH_MUTATION_LOG"
  : > "$GH_JQ_LOG"
}

setup_case() {
  local name="$1"
  local repo="$TMP/$name"
  local remote="$TMP/$name.git"
  local data="$TMP/data/$name"

  mkdir -p "$data"
  git init -q --bare "$remote"
  git init -q "$repo"
  git -C "$repo" checkout -q -b dev
  git -C "$repo" config user.name test
  git -C "$repo" config user.email test@example.com
  printf 'fixture\n' > "$repo/README.md"
  git -C "$repo" add README.md
  git -C "$repo" commit -qm init
  git -C "$repo" remote add origin "$remote"
  git -C "$repo" push -q -u origin dev

  write_linked_none "$data/linked.json"
  write_project "$data/project.json" Ready false 1
  jq -n '{data:{repository:{issue:{issueType:{name:"Task"}}}}}' \
    > "$data/type.json"
  write_blockers "$data/blocked.json" '[]' false
}

write_blockers() {
  local file="$1" nodes="$2" has_next="$3"
  jq -n --argjson nodes "$nodes" --argjson has_next "$has_next" \
    '{data:{repository:{issue:{blockedBy:{pageInfo:{hasNextPage:$has_next},nodes:$nodes}}}}}' \
    > "$file"
}

write_linked_none() {
  local file="$1"
  jq -n '{data:{repository:{issue:{closedByPullRequestsReferences:{pageInfo:{hasNextPage:false},nodes:[]}}}}}' \
    > "$file"
}

write_linked_one() {
  local file="$1" state="$2" draft="$3" base="$4" head="$5"
  local head_repo="$6" base_repo="$7" linked_issue="$8" linked_repo="$9"
  jq -n \
    --arg state "$state" \
    --argjson draft "$draft" \
    --arg base "$base" \
    --arg head "$head" \
    --arg head_repo "$head_repo" \
    --arg base_repo "$base_repo" \
    --argjson linked_issue "$linked_issue" \
    --arg linked_repo "$linked_repo" \
    '{data:{repository:{issue:{closedByPullRequestsReferences:{
      pageInfo:{hasNextPage:false},
      nodes:[{
        number:88,state:$state,isDraft:$draft,baseRefName:$base,headRefName:$head,
        headRepository:{nameWithOwner:$head_repo},repository:{nameWithOwner:$base_repo},
        closingIssuesReferences:{nodes:[{number:$linked_issue,repository:{nameWithOwner:$linked_repo}}]}
      }]
    }}}}}' > "$file"
}

write_linked_multiple() {
  local file="$1"
  jq -n '{data:{repository:{issue:{closedByPullRequestsReferences:{
    pageInfo:{hasNextPage:false},
    nodes:[
      {number:88,state:"OPEN",isDraft:true,baseRefName:"dev",headRefName:"fix/42-one",
       headRepository:{nameWithOwner:"skku-heven/example"},repository:{nameWithOwner:"skku-heven/example"},
       closingIssuesReferences:{nodes:[{number:42,repository:{nameWithOwner:"skku-heven/example"}}]}},
      {number:89,state:"OPEN",isDraft:false,baseRefName:"dev",headRefName:"feat/42-two",
       headRepository:{nameWithOwner:"skku-heven/example"},repository:{nameWithOwner:"skku-heven/example"},
       closingIssuesReferences:{nodes:[{number:42,repository:{nameWithOwner:"skku-heven/example"}}]}}
    ]
  }}}}}' > "$file"
}

write_project() {
  local file="$1" status="$2" has_next="$3" count="$4"
  jq -n \
    --arg status "$status" \
    --argjson has_next "$has_next" \
    --argjson count "$count" \
    '{data:{repository:{issue:{projectItems:{
      pageInfo:{hasNextPage:$has_next},
      nodes:[range(0;$count) | {
        project:{number:(13 + .),title:("Roadmap " + (. | tostring))},
        fieldValueByName:{name:$status}
      }]
    }}}}}' > "$file"
}

run_claim() {
  local name="$1" output="$2"
  shift 2
  local repo="$TMP/$name"
  local data="$TMP/data/$name"

  (cd "$repo" && env \
    GH_REPO=skku-heven/example DEFAULT_BRANCH=dev \
    GH_TMP_JSON="$TMP/gh-tmp.json" \
    FAKE_LINKED_JSON="$data/linked.json" \
    FAKE_PROJECT_JSON="$data/project.json" \
    FAKE_TYPE_JSON="$data/type.json" \
    FAKE_BLOCKED_JSON="$data/blocked.json" \
    FAKE_ISSUE_TITLE='Ready task' FAKE_ISSUE_STATE=OPEN \
    FAKE_CONFLICT_COUNT=0 FAKE_CONFLICT_HEAD='' FAKE_PR_CREATE_FAIL=0 \
    "$@" "$SCRIPT" 42 > "$output" 2>&1)
}

snapshot_refs() {
  local name="$1"
  {
    git -C "$TMP/$name" for-each-ref refs \
      --format='local %(refname) %(objectname)'
    git --git-dir="$TMP/$name.git" for-each-ref refs/heads \
      --format='remote %(refname) %(objectname)'
  } | sort
}

assert_unchanged_without_gh_mutation() {
  local label="$1" name="$2" before="$3"
  local after
  after=$(snapshot_refs "$name")
  [[ "$after" == "$before" ]] || fail "$label changed local or remote refs"
  [[ ! -s "$GH_MUTATION_LOG" ]] || fail "$label made mutation-capable gh calls"
}

# Backlog rejects before git/GitHub mutation.
name=backlog
setup_case "$name"
write_project "$TMP/data/$name/project.json" Backlog false 1
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'Backlog claim unexpectedly succeeded'
fi
grep -q 'Ready' "$TMP/$name.out" || fail 'Backlog error does not explain Ready requirement'
assert_unchanged_without_gh_mutation 'Backlog rejection' "$name" "$before"
pass 'Backlog is rejected before mutation'

# Canonical linked PR reuse is independent from edited title/type and reports draft/ready state.
for reuse_state in draft ready; do
  name="reuse-$reuse_state"
  setup_case "$name"
  if [[ "$reuse_state" == draft ]]; then
    draft=true
    expected_state='state: draft'
  else
    draft=false
    expected_state='state: ready for review'
  fi
  write_linked_one "$TMP/data/$name/linked.json" OPEN "$draft" dev \
    fix/42-original-title skku-heven/example skku-heven/example 42 skku-heven/example
  write_project "$TMP/data/$name/project.json" 'In progress' false 1
  before=$(snapshot_refs "$name")
  reset_logs
  if ! run_claim "$name" "$TMP/$name.out" FAKE_ISSUE_TITLE='Edited title'; then
    cat "$TMP/$name.out" >&2
    fail "$reuse_state canonical linked PR was not reused"
  fi
  grep -q 'PR #88' "$TMP/$name.out" || fail "$reuse_state reuse did not report PR number"
  grep -qi "$expected_state" "$TMP/$name.out" || fail "$reuse_state reuse state was not reported"
  assert_unchanged_without_gh_mutation "$reuse_state reuse" "$name" "$before"
  ! grep -q 'projectItems' "$GH_CALL_LOG" || fail "$reuse_state reuse queried Project"
  ! grep -q 'pr list' "$GH_CALL_LOG" || fail "$reuse_state reuse used recomputed branch discovery"
  grep -q 'closedByPullRequestsReferences' "$GH_JQ_LOG" \
    || fail "$reuse_state linked raw JSON did not use real --jq"
  pass "$reuse_state canonical linked PR is reused without mutation"
done

# Malformed or multiple linked open PRs fail closed.
while IFS='|' read -r name base head head_repo base_repo linked_issue linked_repo; do
  setup_case "$name"
  write_linked_one "$TMP/data/$name/linked.json" OPEN true "$base" "$head" \
    "$head_repo" "$base_repo" "$linked_issue" "$linked_repo"
  before=$(snapshot_refs "$name")
  reset_logs
  if run_claim "$name" "$TMP/$name.out"; then
    fail "$name malformed linked PR unexpectedly succeeded"
  fi
  grep -qi 'linked PR' "$TMP/$name.out" || fail "$name failure did not identify linked PR"
  assert_unchanged_without_gh_mutation "$name rejection" "$name" "$before"
  pass "$name linked PR is rejected without mutation"
done <<'EOF'
wrong-base|main|fix/42-original|skku-heven/example|skku-heven/example|42|skku-heven/example
fork-head|dev|fix/42-original|someone/fork|skku-heven/example|42|skku-heven/example
wrong-base-repo|dev|fix/42-original|skku-heven/example|other/repo|42|skku-heven/example
wrong-branch|dev|feature/42-original|skku-heven/example|skku-heven/example|42|skku-heven/example
missing-linkage|dev|fix/42-original|skku-heven/example|skku-heven/example|41|skku-heven/example
EOF

name=multiple-linked
setup_case "$name"
write_linked_multiple "$TMP/data/$name/linked.json"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'multiple linked PRs unexpectedly succeeded'
fi
grep -qi 'linked PR' "$TMP/$name.out" || fail 'multiple linked failure did not identify linked PRs'
assert_unchanged_without_gh_mutation 'multiple linked rejection' "$name" "$before"
pass 'multiple linked open PRs fail closed'

# A recomputed canonical branch with an unlinked open PR conflicts before Project lookup.
name=unlinked-conflict
setup_case "$name"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out" FAKE_CONFLICT_COUNT=1; then
  fail 'unlinked canonical-head conflict unexpectedly succeeded'
fi
grep -qi 'conflict' "$TMP/$name.out" || fail 'unlinked PR failure does not explain conflict'
! grep -q 'projectItems' "$GH_CALL_LOG" || fail 'unlinked PR conflict queried Project'
assert_unchanged_without_gh_mutation 'unlinked conflict' "$name" "$before"
pass 'unlinked canonical-head PR is rejected before mutation'

# A changed title/type can recover an old remote branch; conflict-check that
# effective branch before fetch, switch, push, or PR creation.
name=recovered-unlinked-conflict
setup_case "$name"
base_sha=$(git --git-dir="$TMP/$name.git" rev-parse refs/heads/dev)
git --git-dir="$TMP/$name.git" update-ref refs/heads/fix/42-original-title "$base_sha"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out" \
  FAKE_ISSUE_TITLE='Edited task' \
  FAKE_CONFLICT_HEAD='fix/42-original-title' \
  FAKE_CONFLICT_COUNT=1; then
  fail 'recovered old-branch unlinked conflict unexpectedly succeeded'
fi
grep -qi 'conflict' "$TMP/$name.out" \
  || fail 'recovered old-branch failure does not explain conflict'
grep -q 'pr list.*--head fix/42-original-title' "$GH_CALL_LOG" \
  || fail 'recovered old-branch conflict did not query the effective branch'
! grep -q 'projectItems' "$GH_CALL_LOG" \
  || fail 'recovered old-branch conflict queried Project'
assert_unchanged_without_gh_mutation 'recovered old-branch conflict' "$name" "$before"
pass 'recovered old-branch unlinked PR is rejected before mutation'

# Project pagination fails closed before mutation.
name=project-pagination
setup_case "$name"
write_project "$TMP/data/$name/project.json" Ready true 1
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'paginated Project lookup unexpectedly succeeded'
fi
grep -qi 'pag' "$TMP/$name.out" || fail 'pagination failure is not explained'
assert_unchanged_without_gh_mutation 'Project pagination rejection' "$name" "$before"
pass 'Project pagination fails before mutation'

# A slug tool failure must stop before branch, push, or PR mutation.
name=slug-tool-failure
setup_case "$name"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out" FAKE_SED_FAIL=1; then
  fail 'slug tool failure unexpectedly succeeded'
fi
grep -q 'slug 생성 실패' "$TMP/$name.out" \
  || fail 'slug tool failure is not explained'
assert_unchanged_without_gh_mutation 'slug tool failure' "$name" "$before"
pass 'slug tool failure stops before mutation'

# New Ready claim creates the exact draft PR contract.
name=ready-create
setup_case "$name"
reset_logs
if ! run_claim "$name" "$TMP/$name.out"; then
  cat "$TMP/$name.out" >&2
  fail 'Ready claim failed'
fi
if ! git --git-dir="$TMP/$name.git" show-ref --verify refs/heads/feat/42-ready-task >/dev/null; then
  print_claim_diagnostics "$name" "$TMP/$name.out"
  fail 'Ready claim did not push canonical branch'
fi
grep -q 'pr create.*--base dev' "$GH_MUTATION_LOG" || fail 'PR base is not dev'
grep -q -- '--head feat/42-ready-task' "$GH_MUTATION_LOG" || fail 'PR head is not canonical'
grep -q -- '--draft' "$GH_MUTATION_LOG" || fail 'PR was not created as draft'
grep -q 'Resolves.*#42' "$GH_MUTATION_LOG" || fail 'PR body does not close issue 42'
grep -q 'projectItems' "$GH_JQ_LOG" || fail 'raw Project JSON did not use real --jq'
pass 'Ready creates canonical draft PR with closing body'

# A Korean title must never be truncated in the middle of a UTF-8 code point.
name=utf8-title
setup_case "$name"
reset_logs
utf8_title='[Task] 문서용 workflow 화면 캡처 테스트'
utf8_branch='feat/42-task-문서용-workflow-화면-캡처-테스트'
if ! run_claim "$name" "$TMP/$name.out" FAKE_ISSUE_TITLE="$utf8_title"; then
  cat "$TMP/$name.out" >&2
  fail 'Korean-title claim failed'
fi
if ! git --git-dir="$TMP/$name.git" show-ref --verify "refs/heads/$utf8_branch" >/dev/null; then
  print_claim_diagnostics "$name" "$TMP/$name.out"
  fail 'Korean title was not preserved as a valid UTF-8 branch name'
fi
grep -Fq -- "--head $utf8_branch" "$GH_MUTATION_LOG" \
  || fail 'Korean branch was not passed intact to PR creation'
pass 'Korean title creates a valid UTF-8 branch name'

# A pushed branch from a failed PR attempt is recovered without another empty commit.
name=retry-after-push
setup_case "$name"
reset_logs
if run_claim "$name" "$TMP/$name-first.out" FAKE_PR_CREATE_FAIL=1; then
  fail 'first PR-create failure unexpectedly succeeded'
fi
remote_branch=refs/heads/feat/42-ready-task
first_sha=$(git --git-dir="$TMP/$name.git" rev-parse "$remote_branch")
first_count=$(git --git-dir="$TMP/$name.git" rev-list --count refs/heads/dev.."$remote_branch")
[[ "$first_count" -eq 1 ]] || fail 'first attempt did not create exactly one start commit'
reset_logs
if ! run_claim "$name" "$TMP/$name-second.out"; then
  cat "$TMP/$name-second.out" >&2
  fail 'retry after pushed branch failed'
fi
second_sha=$(git --git-dir="$TMP/$name.git" rev-parse "$remote_branch")
second_count=$(git --git-dir="$TMP/$name.git" rev-list --count refs/heads/dev.."$remote_branch")
[[ "$second_sha" == "$first_sha" ]] || fail 'retry changed the recovered remote branch SHA'
[[ "$second_count" -eq 1 ]] || fail 'retry appended a second empty commit'
pass 'pushed-branch retry reuses the single start commit'

# Multiple remote branches for the issue fail before push/PR creation.
name=multiple-remote
setup_case "$name"
base_sha=$(git --git-dir="$TMP/$name.git" rev-parse refs/heads/dev)
git --git-dir="$TMP/$name.git" update-ref refs/heads/feat/42-old-one "$base_sha"
git --git-dir="$TMP/$name.git" update-ref refs/heads/fix/42-old-two "$base_sha"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'multiple remote issue branches unexpectedly succeeded'
fi
grep -qi 'remote.*branch' "$TMP/$name.out" || fail 'multiple remote failure is not explained'
assert_unchanged_without_gh_mutation 'multiple remote rejection' "$name" "$before"
pass 'multiple remote issue branches fail closed'

# Open blockers stop a new claim and name the blocking issue.
name=open-blocker
setup_case "$name"
write_blockers "$TMP/data/$name/blocked.json" '[{"number":7,"state":"OPEN"},{"number":8,"state":"CLOSED"}]' false
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'open blocker unexpectedly claimed'
fi
grep -q '#7' "$TMP/$name.out" || fail 'open blocker is not named'
grep -q '#8' "$TMP/$name.out" && fail 'closed blocker reported as open'
assert_unchanged_without_gh_mutation 'open blocker rejection' "$name" "$before"
pass 'open blocker stops a new claim'

# Closed blockers do not stop a new claim.
name=closed-blocker
setup_case "$name"
write_blockers "$TMP/data/$name/blocked.json" '[{"number":8,"state":"CLOSED"}]' false
reset_logs
run_claim "$name" "$TMP/$name.out" || fail 'closed blocker rejected the claim'
grep -q 'pr create' "$GH_MUTATION_LOG" || fail 'closed blocker did not create a PR'
pass 'closed blocker allows a new claim'

# Unknown blocker state must not be treated as no blockers.
name=blocker-query-failure
setup_case "$name"
before=$(snapshot_refs "$name")
reset_logs
if run_claim "$name" "$TMP/$name.out" FAKE_BLOCKED_QUERY_FAIL=1; then
  fail 'blocker query failure unexpectedly claimed'
fi
assert_unchanged_without_gh_mutation 'blocker query failure' "$name" "$before"
write_blockers "$TMP/data/$name/blocked.json" '[]' true
reset_logs
if run_claim "$name" "$TMP/$name.out"; then
  fail 'paginated blockers unexpectedly claimed'
fi
assert_unchanged_without_gh_mutation 'paginated blockers' "$name" "$before"
pass 'unreadable blockers fail closed'

# The draft PR body comes from the repository's PR template.
name=pr-template
setup_case "$name"
mkdir -p "$TMP/$name/.github"
printf '## 템플릿 섹션 고유문구\n\nResolves #\n' > "$TMP/$name/.github/pull_request_template.md"
git -C "$TMP/$name" add .github
git -C "$TMP/$name" commit -qm template
git -C "$TMP/$name" push -q origin dev
reset_logs
run_claim "$name" "$TMP/$name.out" || fail 'PR template claim failed'
grep -q '템플릿 섹션 고유문구' "$GH_BODY_LOG" || fail 'PR body ignored the template'
grep -qx 'Resolves #42' "$GH_BODY_LOG" || fail 'template PR body lost the issue link'
pass 'draft PR body uses the repository template'

for phrase in \
  'linked open PR references' \
  'draft 또는 Ready for review' \
  'projectItems pagination' \
  'remote issue branch' \
  'origin/$BASE..HEAD'; do
  grep -Fq "$phrase" "$SKILL" || fail "issue-worker skill missing: $phrase"
done

if ((failures > 0)); then
  printf 'FAILED: %d issue claim safety expectation(s)\n' "$failures" >&2
  exit 1
fi

printf 'ok - all claim reuse and retry safety regressions\n'
