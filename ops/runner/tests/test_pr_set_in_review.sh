#!/usr/bin/env bash

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)
SCRIPT="$ROOT/ops/runner/bin/pr-set-in-review.sh"
PROJECT_WORKFLOW="$ROOT/.github/workflows/project-status.yml"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

failures=0
fail() {
  printf 'not ok - %s\n' "$1" >&2
  failures=$((failures + 1))
}

setup_case() {
  local name="$1"
  local home="$TMP/$name/home"
  mkdir -p "$home/.local/bin" "$TMP/$name/agent" "$TMP/$name/state"

  cat > "$home/.local/bin/gh" <<'FAKE_GH'
#!/usr/bin/env bash
set -Eeuo pipefail
printf '%q ' "$@" >> "$GH_CALL_LOG"
printf '\n' >> "$GH_CALL_LOG"

get_env() {
  local key="$1" fallback="$2"
  if [[ -v "$key" ]]; then
    printf '%s' "${!key}"
  else
    printf '%s' "$fallback"
  fi
}

case "$*" in
  *'projectsV2(first: 10)'*)
    # A deliberately wrong repo-linked project. The implementation must not
    # select this instead of the linked issue's own single Project item.
    printf 'PVT_repo_project 99\n'
    ;;
  *'closingIssuesReferences(first: 10)'*)
    [[ "${FAKE_PR_QUERY_MODE:-success}" == success ]] || exit 81
    pr_query_count=$(grep -c 'closingIssuesReferences' "$GH_CALL_LOG")
    pr_state="${FAKE_PR_STATE:-OPEN}"
    pr_draft="${FAKE_PR_DRAFT:-false}"
    closing_mode="${FAKE_CLOSING_MODE:-one}"
    closing_has_next="${FAKE_CLOSING_HAS_NEXT:-false}"
    if [[ "$pr_query_count" -eq 2 ]]; then
      pr_state="${FAKE_FINAL_PR_STATE:-$pr_state}"
      pr_draft="${FAKE_FINAL_PR_DRAFT:-$pr_draft}"
      closing_mode="${FAKE_FINAL_CLOSING_MODE:-$closing_mode}"
      closing_has_next="${FAKE_FINAL_CLOSING_HAS_NEXT:-$closing_has_next}"
    elif [[ "$pr_query_count" -gt 2 ]]; then
      pr_state="${FAKE_POST_PR_STATE:-${FAKE_FINAL_PR_STATE:-$pr_state}}"
      pr_draft="${FAKE_POST_PR_DRAFT:-${FAKE_FINAL_PR_DRAFT:-$pr_draft}}"
      closing_mode="${FAKE_POST_CLOSING_MODE:-${FAKE_FINAL_CLOSING_MODE:-$closing_mode}}"
      closing_has_next="${FAKE_POST_CLOSING_HAS_NEXT:-${FAKE_FINAL_CLOSING_HAS_NEXT:-$closing_has_next}}"
    fi
    if [[ "${FAKE_PR_DRAFT_MODE:-}" == bounded-flap ]]; then
      # Each reconciliation pass performs initial/final/post PR reads. Keep the
      # first two stable and flip the post read so another pass is requested.
      # The hard guard makes the pre-fix unbounded recursion terminate in tests.
      [[ "$pr_query_count" -le 12 ]] || exit 94
      flap_pass=$(((pr_query_count - 1) / 3))
      flap_read=$(((pr_query_count - 1) % 3))
      if ((flap_pass % 2 == 0)); then
        pr_draft=false
      else
        pr_draft=true
      fi
      if [[ "$flap_read" -eq 2 ]]; then
        [[ "$pr_draft" == true ]] && pr_draft=false || pr_draft=true
      fi
    fi
    printf 'PR\t%s\t%s\t%s\n' \
      "$pr_state" "$pr_draft" "$closing_has_next"
    if [[ "$pr_state" != MERGED ]]; then
      case "$closing_mode" in
        empty) ;;
        one) issue_count=1 ;;
        two) issue_count=2 ;;
        four) issue_count=4 ;;
      esac
      for issue_index in $(seq 1 "${issue_count:-0}"); do
        issue_state=$(get_env "FAKE_ISSUE${issue_index}_STATE" "${FAKE_ISSUE_STATE:-OPEN}")
        if [[ "$pr_query_count" -ge 2 ]]; then
          issue_state=$(get_env "FAKE_FINAL_ISSUE${issue_index}_STATE" \
            "${FAKE_FINAL_ISSUE_STATE:-$issue_state}")
        fi
        printf 'ISSUE\tISSUE_NODE_%s\t%s\t%s\n' \
          "$issue_index" "$((40 + issue_index))" "$issue_state"
      done
    fi
    ;;
  *'projectItems(first: 20)'*)
    case "$*" in
      *'node(id: "ISSUE_NODE_1")'*) issue_index=1 ;;
      *'node(id: "ISSUE_NODE_2")'*) issue_index=2 ;;
      *'node(id: "ISSUE_NODE_3")'*) issue_index=3 ;;
      *'node(id: "ISSUE_NODE_4")'*) issue_index=4 ;;
      *) printf 'unknown issue node in query\n' >&2; exit 91 ;;
    esac
    counter_file="$GH_STATE_DIR/item-$issue_index-count"
    item_query_count=0
    [[ ! -f "$counter_file" ]] || item_query_count=$(<"$counter_file")
    item_query_count=$((item_query_count + 1))
    printf '%s\n' "$item_query_count" > "$counter_file"

    item_mode=$(get_env "FAKE_ITEM${issue_index}_MODE" "${FAKE_ITEM_MODE:-one}")
    item_has_next=$(get_env "FAKE_ITEM${issue_index}_HAS_NEXT" "${FAKE_ITEM_HAS_NEXT:-false}")
    issue_state=$(get_env "FAKE_RECHECK_ISSUE${issue_index}_STATE" \
      "${FAKE_RECHECK_ISSUE_STATE:-$(get_env "FAKE_ISSUE${issue_index}_STATE" "${FAKE_ISSUE_STATE:-OPEN}")}")
    status_file="$GH_STATE_DIR/status-$issue_index"
    if [[ -f "$status_file" ]]; then
      item_status=$(<"$status_file")
    else
      item_status=$(get_env "FAKE_ITEM${issue_index}_STATUS" "${FAKE_ITEM_STATUS:-In progress}")
    fi
    if [[ "$item_query_count" -eq 2 && "$item_mode" != fail ]]; then
      item_mode=$(get_env "FAKE_FINAL_ITEM${issue_index}_MODE" "${FAKE_FINAL_ITEM_MODE:-$item_mode}")
      item_has_next=$(get_env "FAKE_FINAL_ITEM${issue_index}_HAS_NEXT" \
        "${FAKE_FINAL_ITEM_HAS_NEXT:-$item_has_next}")
      issue_state=$(get_env "FAKE_FINAL_ISSUE${issue_index}_STATE" \
        "${FAKE_FINAL_ISSUE_STATE:-$issue_state}")
      item_status=$(get_env "FAKE_FINAL_ITEM${issue_index}_STATUS" \
        "${FAKE_FINAL_ITEM_STATUS:-$item_status}")
    fi
    [[ "$item_mode" != fail ]] || exit 82
    printf 'ISSUE_STATE\t%s\n' \
      "$issue_state"
    printf 'PAGE\t%s\n' "$item_has_next"
    case "$item_mode" in
      missing) ;;
      one)
        if [[ "$issue_index" -eq 1 ]]; then
          printf 'ITEM\tITEM_NODE_1\tPVT_issue_project\t13\t%s\n' "$item_status"
        else
          printf 'ITEM\tITEM_NODE_%s\tPVT_issue_project_%s\t%s\t%s\n' \
            "$issue_index" "$issue_index" "$((12 + issue_index))" "$item_status"
        fi
        ;;
      changed)
        printf 'ITEM\tITEM_NODE_%s_CHANGED\tPVT_issue_project_%s\t%s\t%s\n' \
          "$issue_index" "$issue_index" "$((12 + issue_index))" "$item_status"
        ;;
      multiple)
        printf 'ITEM\tITEM_NODE_%s\tPVT_issue_project_%s\t%s\tIn progress\n' \
          "$issue_index" "$issue_index" "$((12 + issue_index))"
        printf 'ITEM\tITEM_NODE_%s_OTHER\tPVT_other_project_%s\t%s\tReady\n' \
          "$issue_index" "$issue_index" "$((22 + issue_index))"
        ;;
    esac
    ;;
  *'field(name: "Status")'*)
    case "${FAKE_FIELD_MODE:-success}" in
      fail) exit 84 ;;
      incomplete) printf 'FIELD\tPVT_field\n' ;;
      success)
        printf 'FIELD\tPVT_field\n'
        printf 'OPTION\tIn review\tOPT_in_review\n'
        printf 'OPTION\tIn progress\tOPT_in_progress\n'
        printf 'OPTION\tReady\tOPT_ready\n'
        ;;
    esac
    ;;
  *'updateProjectV2ItemFieldValue'*)
    printf '%q ' "$@" >> "$GH_MUTATION_LOG"
    printf '\n' >> "$GH_MUTATION_LOG"
    case "${FAKE_MUTATION_MODE:-success}" in
      success) ;;
      fail) exit 83 ;;
      fail-second)
        [[ "$*" != *'itemId: "ITEM_NODE_2"'* ]] || exit 83
        ;;
    esac
    case "$*" in
      *'itemId: "ITEM_NODE_1"'*) issue_index=1 ;;
      *'itemId: "ITEM_NODE_2"'*) issue_index=2 ;;
      *'itemId: "ITEM_NODE_3"'*) issue_index=3 ;;
      *'itemId: "ITEM_NODE_4"'*) issue_index=4 ;;
      *) printf 'unknown mutation item\n' >&2; exit 92 ;;
    esac
    case "$*" in
      *OPT_in_review*) target_status='In review' ;;
      *OPT_in_progress*) target_status='In progress' ;;
      *OPT_ready*) target_status='Ready' ;;
      *) printf 'unknown mutation option\n' >&2; exit 93 ;;
    esac
    printf '%s\n' "$target_status" > "$GH_STATE_DIR/status-$issue_index"
    printf '{"data":{"updateProjectV2ItemFieldValue":{"projectV2Item":{"id":"ITEM_NODE_1"}}}}\n'
    ;;
  *)
    printf 'unexpected gh call: %s\n' "$*" >&2
    exit 90
    ;;
esac
FAKE_GH

  cat > "$home/.local/bin/sleep" <<'FAKE_SLEEP'
#!/usr/bin/env bash
exit 0
FAKE_SLEEP

  chmod +x "$home/.local/bin/gh" "$home/.local/bin/sleep"
  : > "$TMP/$name/gh.log"
  : > "$TMP/$name/mutations.log"
}

run_case() {
  local name="$1" output="$2"
  shift 2
  local home="$TMP/$name/home"

  env \
    HOME="$home" \
    PATH="$home/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    RUNNER_TEMP="$TMP/$name/agent" \
    GITHUB_REPOSITORY=skku-heven/example \
    GH_TOKEN=incoming-project-token \
    GH_CALL_LOG="$TMP/$name/gh.log" \
    GH_MUTATION_LOG="$TMP/$name/mutations.log" \
    GH_STATE_DIR="$TMP/$name/state" \
    FAKE_PR_QUERY_MODE=success FAKE_PR_STATE=OPEN \
    FAKE_PR_DRAFT=false FAKE_CLOSING_MODE=one FAKE_CLOSING_HAS_NEXT=false \
    FAKE_ISSUE_STATE=OPEN FAKE_ITEM_MODE=one FAKE_ITEM_HAS_NEXT=false \
    FAKE_ITEM_STATUS='In progress' FAKE_FIELD_MODE=success \
    FAKE_MUTATION_MODE=success \
    "$@" "$SCRIPT" 17 > "$output" 2>&1
}

assert_no_mutation() {
  local name="$1" label="$2"
  [[ ! -s "$TMP/$name/mutations.log" ]] || fail "$label attempted Status mutation"
}

name=pr-query-failure
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_PR_QUERY_MODE=fail; then
  fail 'PR-state query failure unexpectedly succeeded'
fi
grep -qi 'PR.*조회 실패\|PR.*query.*failed' "$TMP/$name/out" \
  || fail 'PR-state query failure was not reported'
assert_no_mutation "$name" 'PR-state query failure'
printf 'ok - PR-state query failure propagates\n'

name=closed-pr
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_PR_STATE=MERGED; then
  cat "$TMP/$name/out" >&2
  fail 'merged PR stale run did not succeed'
fi
grep -qi 'PR.*MERGED' "$TMP/$name/out" \
  || fail 'merged PR stale no-op was not reported'
assert_no_mutation "$name" 'merged PR stale run'
printf 'ok - merged PR is a successful no-op\n'

name=closed-unmerged-in-progress
setup_case "$name"
run_case "$name" "$TMP/$name/out" FAKE_PR_STATE=CLOSED FAKE_PR_DRAFT=true \
  || fail 'closed unmerged PR failed'
grep -q 'OPT_ready' "$TMP/$name/mutations.log" \
  || fail 'closed unmerged PR did not return the issue to Ready'
printf 'ok - closed unmerged PR returns In progress issue to Ready\n'

name=closed-unmerged-in-review
setup_case "$name"
run_case "$name" "$TMP/$name/out" FAKE_PR_STATE=CLOSED "FAKE_ITEM_STATUS=In review" \
  || fail 'closed unmerged In review PR failed'
grep -q 'OPT_ready' "$TMP/$name/mutations.log" \
  || fail 'closed unmerged PR did not return In review issue to Ready'
printf 'ok - closed unmerged PR returns In review issue to Ready\n'

for status in Backlog Ready Done; do
  name="closed-unmerged-$status"
  setup_case "$name"
  run_case "$name" "$TMP/$name/out" FAKE_PR_STATE=CLOSED "FAKE_ITEM_STATUS=$status" \
    || fail "closed unmerged PR failed for $status"
  assert_no_mutation "$name" "closed unmerged PR with $status"
done
printf 'ok - closed unmerged PR leaves other statuses unchanged\n'

name=no-linked-issue
setup_case "$name"
run_case "$name" "$TMP/$name/out" FAKE_CLOSING_MODE=empty \
  || fail 'open PR without a closing issue failed the workflow'
grep -q 'linked issue 없음' "$TMP/$name/out" \
  || fail 'missing closing issue was not reported'
assert_no_mutation "$name" 'missing closing issue'
printf 'ok - open PR without a closing issue warns and succeeds\n'

name=closed-issue
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_ISSUE_STATE=CLOSED; then
  cat "$TMP/$name/out" >&2
  fail 'closed linked issue stale run did not succeed'
fi
grep -qi 'issue.*CLOSED\|닫힌 issue' "$TMP/$name/out" \
  || fail 'closed linked issue stale no-op was not reported'
assert_no_mutation "$name" 'closed linked issue stale run'
printf 'ok - closed linked issue is a successful terminal no-op\n'

name=item-failure
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_ITEM_MODE=fail; then
  fail 'Project-item query failure unexpectedly succeeded'
fi
grep -Fq 'Project item 조회 실패' "$TMP/$name/out" \
  || fail 'Project-item query failure was not reported'
item_calls=$(grep -c 'projectItems' "$TMP/$name/gh.log" || true)
[[ "$item_calls" -eq 3 ]] || fail 'Project-item query failure did not exhaust three retries'
assert_no_mutation "$name" 'Project-item query failure'
printf 'ok - Project-item query failure propagates after retries\n'

name=item-missing
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_ITEM_MODE=missing; then
  fail 'open issue without a Project item unexpectedly succeeded'
fi
grep -qi 'Project item.*없\|exactly one Project' "$TMP/$name/out" \
  || fail 'missing Project item failure was not reported'
assert_no_mutation "$name" 'missing Project item'
printf 'ok - open issue without a Project item fails closed\n'

name=item-multiple
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_ITEM_MODE=multiple; then
  fail 'issue with multiple Project items unexpectedly succeeded'
fi
grep -qi 'Project item.*여러\|exactly one Project' "$TMP/$name/out" \
  || fail 'multiple Project item failure was not reported'
assert_no_mutation "$name" 'multiple Project items'
printf 'ok - multiple Project items fail closed\n'

name=item-pagination
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_ITEM_HAS_NEXT=true; then
  fail 'paginated Project-item response unexpectedly succeeded'
fi
grep -qi 'pagination\|페이징' "$TMP/$name/out" \
  || fail 'Project-item pagination failure was not reported'
assert_no_mutation "$name" 'paginated Project items'
printf 'ok - paginated Project configuration fails closed\n'

name=done
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_ITEM_STATUS=Done; then
  cat "$TMP/$name/out" >&2
  fail 'Done issue terminal run did not succeed'
fi
grep -q 'Done' "$TMP/$name/out" || fail 'Done terminal no-op was not reported'
assert_no_mutation "$name" 'Done terminal run'
printf 'ok - Done is a successful terminal no-op\n'

name=already-in-review
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_ITEM_STATUS='In review'; then
  cat "$TMP/$name/out" >&2
  fail 'already-In-review idempotent run did not succeed'
fi
grep -q 'In review' "$TMP/$name/out" \
  || fail 'already-In-review idempotent no-op was not reported'
assert_no_mutation "$name" 'already-In-review run'
printf 'ok - In review is an idempotent successful no-op\n'

name=field-failure
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_FIELD_MODE=fail; then
  fail 'Status configuration query failure unexpectedly succeeded'
fi
grep -qi 'Status.*조회 실패\|Status.*query.*failed' "$TMP/$name/out" \
  || fail 'Status configuration query failure was not reported'
assert_no_mutation "$name" 'Status configuration query failure'
printf 'ok - unreadable Status configuration fails closed\n'

name=field-incomplete
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_FIELD_MODE=incomplete; then
  fail 'incomplete Status configuration unexpectedly succeeded'
fi
grep -qi 'In review.*옵션 없음\|incomplete' "$TMP/$name/out" \
  || fail 'incomplete Status configuration failure was not reported'
assert_no_mutation "$name" 'incomplete Status configuration'
printf 'ok - incomplete Status configuration fails closed\n'

name=mixed-terminal-active
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" \
  FAKE_CLOSING_MODE=four \
  FAKE_ISSUE1_STATE=CLOSED \
  FAKE_ITEM2_STATUS=Done \
  FAKE_ITEM3_STATUS='In review' \
  FAKE_ITEM4_STATUS='In progress'; then
  cat "$TMP/$name/out" >&2
  fail 'mixed terminal/idempotent/active issues did not complete'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 1 ]] \
  || fail 'mixed issue run did not mutate exactly one eligible item'
grep -q 'ITEM_NODE_4' "$TMP/$name/mutations.log" \
  || fail 'mixed issue run did not mutate the eligible fourth issue'
if grep -Eq 'ITEM_NODE_[123]([^0-9]|$)' "$TMP/$name/mutations.log"; then
  fail 'mixed issue run mutated a terminal or idempotent issue'
fi
printf 'ok - terminal and idempotent issues do not block an eligible sibling\n'

name=mixed-active-misconfigured
setup_case "$name"
if run_case "$name" "$TMP/$name/out" \
  FAKE_CLOSING_MODE=two FAKE_ITEM2_MODE=multiple; then
  fail 'mixed active issues with one ambiguous Project item unexpectedly succeeded'
fi
assert_no_mutation "$name" 'mixed active issue preflight failure'
printf 'ok - any active issue misconfiguration fails before all mutations\n'

name=partial-failure-retry
setup_case "$name"
if run_case "$name" "$TMP/$name/first.out" \
  FAKE_CLOSING_MODE=two FAKE_MUTATION_MODE=fail-second; then
  fail 'partial mutation failure unexpectedly succeeded'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 2 ]] \
  || fail 'partial failure did not attempt both eligible issue mutations'
grep -q 'ITEM_NODE_1' "$TMP/$name/mutations.log" \
  || fail 'partial failure did not attempt the first issue'
grep -q 'ITEM_NODE_2' "$TMP/$name/mutations.log" \
  || fail 'partial failure did not attempt the second issue'

if ! run_case "$name" "$TMP/$name/retry.out" \
  FAKE_CLOSING_MODE=two \
  FAKE_ITEM1_STATUS='In review' \
  FAKE_ITEM2_STATUS='In progress' \
  FAKE_MUTATION_MODE=success; then
  cat "$TMP/$name/retry.out" >&2
  fail 'retry after partial failure did not converge'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 3 ]] \
  || fail 'partial-failure retry did not attempt exactly one remaining mutation'
[[ $(grep -c 'ITEM_NODE_1' "$TMP/$name/mutations.log") -eq 1 ]] \
  || fail 'partial-failure retry attempted the already-In-review first issue again'
[[ $(grep -c 'ITEM_NODE_2' "$TMP/$name/mutations.log") -eq 2 ]] \
  || fail 'partial-failure retry did not retry the second issue exactly once'
printf 'ok - partial mutation failure converges on retry\n'

name=live-draft
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" \
  FAKE_PR_DRAFT=true FAKE_FINAL_PR_DRAFT=true FAKE_POST_PR_DRAFT=true \
  FAKE_ITEM_STATUS='In review'; then
  cat "$TMP/$name/out" >&2
  fail 'live draft PR did not reconcile to In progress'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 1 ]] \
  || fail 'live draft PR did not mutate exactly once'
grep -q 'OPT_in_progress' "$TMP/$name/mutations.log" \
  || fail 'live draft PR did not target In progress'
printf 'ok - live draft PR reconciles to In progress\n'

name=race-converted-to-draft
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" \
  FAKE_PR_DRAFT=false FAKE_FINAL_PR_DRAFT=false FAKE_POST_PR_DRAFT=true; then
  cat "$TMP/$name/out" >&2
  fail 'PR converted to draft during mutation did not converge'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 2 ]] \
  || fail 'mid-mutation draft conversion did not perform bounded compensation'
[[ $(sed -n '1p' "$TMP/$name/mutations.log") == *OPT_in_review* ]] \
  || fail 'mid-mutation draft conversion did not first use the ready target'
[[ $(sed -n '2p' "$TMP/$name/mutations.log") == *OPT_in_progress* ]] \
  || fail 'mid-mutation draft conversion did not compensate to In progress'
[[ $(<"$TMP/$name/state/status-1") == 'In progress' ]] \
  || fail 'mid-mutation draft conversion did not finish at In progress'
printf 'ok - post-mutation draft conversion converges to In progress\n'

name=reconcile-depth-env-reset
setup_case "$name"
printf 'PROJECT_STATUS_RECONCILE_DEPTH=0\n' > "$TMP/$name/no-env"
if run_case "$name" "$TMP/$name/out" FAKE_PR_DRAFT_MODE=bounded-flap; then
  fail 'repeated PR mode flapping unexpectedly succeeded'
fi
grep -q '반복 변경.*event retry' "$TMP/$name/out" \
  || fail 'repeated PR mode flapping did not stop at the reconciliation bound'
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 3 ]] \
  || fail 'reconciliation depth was not bounded'
flap_pr_calls=$(grep -c 'closingIssuesReferences' "$TMP/$name/gh.log" || true)
[[ "$flap_pr_calls" -eq 9 ]] \
  || fail 'repeated PR mode flapping exceeded the bounded three passes'
printf 'ok - repeated PR mode changes stop at the reconciliation bound\n'

name=ready-after-draft
setup_case "$name"
if ! run_case "$name" "$TMP/$name/draft.out" \
  FAKE_PR_DRAFT=true FAKE_FINAL_PR_DRAFT=true FAKE_POST_PR_DRAFT=true \
  FAKE_ITEM_STATUS='In review'; then
  cat "$TMP/$name/draft.out" >&2
  fail 'draft phase did not reconcile to In progress'
fi
if ! run_case "$name" "$TMP/$name/ready.out" \
  FAKE_PR_DRAFT=false FAKE_FINAL_PR_DRAFT=false FAKE_POST_PR_DRAFT=false; then
  cat "$TMP/$name/ready.out" >&2
  fail 'ready-after-draft phase did not reconcile to In review'
fi
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 2 ]] \
  || fail 'draft-to-ready transition did not perform one mutation per target'
[[ $(sed -n '1p' "$TMP/$name/mutations.log") == *OPT_in_progress* ]] \
  || fail 'draft phase did not target In progress'
[[ $(sed -n '2p' "$TMP/$name/mutations.log") == *OPT_in_review* ]] \
  || fail 'ready phase did not target In review'
[[ $(<"$TMP/$name/state/status-1") == 'In review' ]] \
  || fail 'ready-after-draft did not finish at In review'
printf 'ok - ready-after-draft event converges back to In review\n'

name=race-pr-closes
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_FINAL_PR_STATE=MERGED; then
  cat "$TMP/$name/out" >&2
  fail 'PR closing after preflight did not become a stale no-op'
fi
grep -qi 'PR.*CLOSED\|stale' "$TMP/$name/out" \
  || fail 'final closed-PR recheck was not reported'
assert_no_mutation "$name" 'PR closed during preflight'
printf 'ok - final PR recheck prevents a stale mutation\n'

name=race-done
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_FINAL_ITEM_STATUS=Done; then
  cat "$TMP/$name/out" >&2
  fail 'Status becoming Done after preflight did not become a terminal no-op'
fi
grep -q 'Done' "$TMP/$name/out" \
  || fail 'final Done recheck was not reported'
assert_no_mutation "$name" 'Status changed to Done during preflight'
printf 'ok - final issue recheck preserves Done\n'

name=race-in-review
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out" FAKE_FINAL_ITEM_STATUS='In review'; then
  cat "$TMP/$name/out" >&2
  fail 'Status becoming In review after preflight did not become an idempotent no-op'
fi
grep -q 'In review' "$TMP/$name/out" \
  || fail 'final In review recheck was not reported'
assert_no_mutation "$name" 'Status changed to In review during preflight'
printf 'ok - final issue recheck is idempotent for In review\n'

name=race-item-multiple
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_FINAL_ITEM_MODE=multiple; then
  fail 'ambiguous final Project items unexpectedly succeeded'
fi
grep -qi 'Project item.*여러\|exactly one Project' "$TMP/$name/out" \
  || fail 'ambiguous final Project items were not reported'
assert_no_mutation "$name" 'Project items became ambiguous during preflight'
printf 'ok - final Project ambiguity fails before mutation\n'

name=race-item-changed
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_FINAL_ITEM_MODE=changed; then
  fail 'changed final Project item unexpectedly succeeded'
fi
grep -qi 'Project item.*변경\|changed' "$TMP/$name/out" \
  || fail 'changed final Project item was not reported'
assert_no_mutation "$name" 'Project item changed during preflight'
printf 'ok - final Project item mismatch fails before mutation\n'

name=mutation-failure
setup_case "$name"
if run_case "$name" "$TMP/$name/out" FAKE_MUTATION_MODE=fail; then
  fail 'Status mutation failure unexpectedly succeeded'
fi
grep -qi 'Status.*실패\|mutation.*fail' "$TMP/$name/out" \
  || fail 'Status mutation failure was not reported'
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 1 ]] \
  || fail 'Status mutation failure call count is not one'
printf 'ok - Status mutation failure propagates\n'

name=success
setup_case "$name"
if ! run_case "$name" "$TMP/$name/out"; then
  cat "$TMP/$name/out" >&2
  fail 'successful Status update failed'
fi
grep -q 'ISSUE_NODE_1 -> In review' "$TMP/$name/out" \
  || fail 'successful Status update was not reported'
[[ $(wc -l < "$TMP/$name/mutations.log") -eq 1 ]] \
  || fail 'successful Status mutation call count is not one'
grep -q 'PVT_issue_project' "$TMP/$name/mutations.log" \
  || fail 'Status mutation did not use the issue-derived Project id'
if grep -q 'projectsV2(first:' "$TMP/$name/gh.log"; then
  fail 'successful path queried arbitrary repo-linked Projects'
fi
printf 'ok - successful update uses the issue-derived Project\n'

if grep -Fq 'hasNextPage // "MISSING"' "$SCRIPT"; then
  fail 'GraphQL parser uses jq // on hasNextPage=false'
fi
printf 'ok - GraphQL pagination parser preserves boolean false\n'

project_concurrency=$(awk '/^concurrency:/{in_block=1; next} in_block && /^[^[:space:]]/{exit} in_block{print}' "$PROJECT_WORKFLOW")
grep -q 'cancel-in-progress: true' <<< "$project_concurrency" \
  || fail 'project-status does not cancel superseded runs'
project_types=$(grep -E '^[[:space:]]+types:' "$PROJECT_WORKFLOW")
grep -q 'closed' <<< "$project_types" \
  || fail 'project-status does not cancel a ready run when the PR closes'
grep -q 'converted_to_draft' <<< "$project_types" \
  || fail 'project-status does not cancel a ready run when the PR converts to draft'
grep -q 'reopened' <<< "$project_types" \
  || fail 'project-status does not handle reopened PRs'
grep -q 'edited' <<< "$project_types" \
  || fail 'project-status does not handle links added by editing the PR'
grep -q 'ref: ${{ github.event.repository.default_branch }}' "$PROJECT_WORKFLOW" \
  || fail 'project-status does not run the default-branch script'
grep -q 'ops/runner/bin/pr-set-in-review.sh' "$PROJECT_WORKFLOW" \
  || fail 'project-status does not run the repository script'
if grep -q 'RUNNER_SCRIPTS' "$PROJECT_WORKFLOW"; then
  fail 'project-status still depends on a manually deployed script'
fi
missing_pat_guard=$(awk '/if \[\[ -z "\$\{GH_TOKEN\}" \]\]; then/{in_block=1} in_block{print} in_block && /^[[:space:]]*fi$/{exit}' "$PROJECT_WORKFLOW")
grep -q 'exit 1' <<< "$missing_pat_guard" \
  || fail 'project-status still reports enabled automation without PROJECT_PAT as success'
printf 'ok - project-status fails missing PROJECT_PAT and cancels stale runs\n'

if ((failures > 0)); then
  printf 'FAILED: %d project-status expectation(s)\n' "$failures" >&2
  exit 1
fi

printf 'ok - all project-status stale-state and fail-closed regressions\n'
