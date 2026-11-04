#!/usr/bin/env bash
#
# pr-set-in-review.sh <PR_NUMBER>
#
# PR이 닫는(Resolves #N) 이슈들의 Project Status를 PR 상태와 맞춘다.
# draft면 "In progress", ready면 "In review"이며 project-status.yml이 호출한다.
#
# project-status.yml이 기본 브랜치를 checkout해 이 파일을 직접 실행한다.
#
# 프로젝트 값(project id / Status 필드 / Status 옵션)은 **linked issue의 유일한
# Project item에서 런타임에 파생**한다. repo에 연결된 임의의 Project를 고르지 않는다.
#
# 인증: org ProjectV2 읽기/쓰기는 PROJECT_PAT(project + repo)이 필요하며 workflow가
# GH_TOKEN으로 전달한다.
#
# 대상 상태: draft PR=In progress, Ready PR=In review, merge 없이 닫힌 PR=Ready.

set -Eeuo pipefail
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

GH_REPO="${GITHUB_REPOSITORY:-${GH_REPO:-}}"
: "${GH_REPO:?GITHUB_REPOSITORY 또는 GH_REPO 필요}"
GH_OWNER="${GH_REPO%%/*}"
REPO_NAME="${GH_REPO#*/}"

PR="${1:?usage: pr-set-in-review.sh <PR_NUMBER>}"

LOG="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/project-status-$PR.log"
mkdir -p "$(dirname "$LOG")"
log() { printf '[%s] [pr-%s] %s\n' "$(date '+%F %T %Z')" "$PR" "$*" | tee -a "$LOG"; }

# 두 번째 인자는 recursive reconciliation 전용이다. workflow는 PR 인자만 쓴다.
RECONCILE_DEPTH="${2:-0}"
if [[ ! "$RECONCILE_DEPTH" =~ ^[0-9]+$ ]]; then
  log "WARN: invalid reconcile depth ($RECONCILE_DEPTH)"
  exit 1
fi
MAX_RECONCILE_DEPTH=2

query_pr_snapshot() {
  gh api graphql -f query="
    query {
      repository(owner: \"$GH_OWNER\", name: \"$REPO_NAME\") {
        pullRequest(number: $PR) {
          state
          isDraft
          closingIssuesReferences(first: 10) {
            pageInfo { hasNextPage }
            nodes { id number state }
          }
        }
      }
    }
  " --jq '
    .data.repository.pullRequest as $pr |
    if $pr == null then
      ["PR", "MISSING", "MISSING"] | @tsv
    else
      (["PR", ($pr.state // "MISSING"),
        ($pr.isDraft | if . == null then "MISSING" else tostring end),
        ($pr.closingIssuesReferences.pageInfo.hasNextPage |
         if . == null then "MISSING" else tostring end)] | @tsv),
      ($pr.closingIssuesReferences.nodes[]? |
        ["ISSUE", (.id // "MISSING"), ((.number // "MISSING") | tostring),
         (.state // "MISSING")] | @tsv)
    end
  ' 2>>"$LOG"
}

query_issue_snapshot() {
  local issue_id="$1"

  gh api graphql -f query="
    query {
      node(id: \"$issue_id\") {
        ... on Issue {
          state
          projectItems(first: 20) {
            pageInfo { hasNextPage }
            nodes {
              id
              project { id number }
              fieldValueByName(name: \"Status\") {
                ... on ProjectV2ItemFieldSingleSelectValue { name }
              }
            }
          }
        }
      }
    }
  " --jq '
    .data.node as $issue |
    if $issue == null then
      ["ISSUE_STATE", "MISSING"] | @tsv
    else
      (["ISSUE_STATE", ($issue.state // "MISSING")] | @tsv),
      (["PAGE", ($issue.projectItems.pageInfo.hasNextPage |
        if . == null then "MISSING" else tostring end)] | @tsv),
      ($issue.projectItems.nodes[]? |
        ["ITEM", (.id // "MISSING"), (.project.id // "MISSING"),
         ((.project.number // "MISSING") | tostring),
         (.fieldValueByName.name // "<unset>")] | @tsv)
    end
  ' 2>>"$LOG"
}

log "=== reconcile project Status start (repo=$GH_REPO, pass=$RECONCILE_DEPTH) ==="

# 1) PR과 closing issue의 현재 상태를 한 번에 읽는다. merge된 PR에서 늦게 도착한
# workflow run은 성공 no-op이고, 누락/불완전 응답은 실패다.
if ! PR_RESULT=$(query_pr_snapshot); then
  log "WARN: PR 및 linked issue 조회 실패"
  exit 1
fi

PR_STATE=""
PR_IS_DRAFT=""
CLOSING_HAS_NEXT=""
ISSUE_IDS=()
ISSUE_NUMBERS=()
ISSUE_STATES=()
while IFS=$'\t' read -r record value1 value2 value3 value4; do
  [[ -n "$record" ]] || continue
  case "$record" in
    PR)
      PR_STATE="$value1"
      PR_IS_DRAFT="$value2"
      CLOSING_HAS_NEXT="$value3"
      ;;
    ISSUE)
      ISSUE_IDS+=("$value1")
      ISSUE_NUMBERS+=("$value2")
      ISSUE_STATES+=("$value3")
      ;;
    *)
      log "WARN: PR/linked issue 응답 해석 실패"
      exit 1
      ;;
  esac
done <<< "$PR_RESULT"

case "$PR_STATE" in
  MERGED)
    log "PR #$PR state=$PR_STATE: stale workflow no-op"
    exit 0
    ;;
  CLOSED) INITIAL_TARGET_STATUS='Ready' ;;
  OPEN)
    case "$PR_IS_DRAFT" in
      true) INITIAL_TARGET_STATUS='In progress' ;;
      false) INITIAL_TARGET_STATUS='In review' ;;
      *)
        log "WARN: PR #$PR isDraft 응답 불완전 ($PR_IS_DRAFT)"
        exit 1
        ;;
    esac
    ;;
  *)
    log "WARN: PR #$PR state 응답 불완전 ($PR_STATE)"
    exit 1
    ;;
esac
log "PR #$PR isDraft=$PR_IS_DRAFT: initial target=$INITIAL_TARGET_STATUS"

case "$CLOSING_HAS_NEXT" in
  false) ;;
  true)
    log "WARN: linked issue pagination 감지. 전체 issue를 확인할 수 없어 중단."
    exit 1
    ;;
  *)
    log "WARN: linked issue pageInfo 응답 불완전"
    exit 1
    ;;
esac

if [[ ${#ISSUE_IDS[@]} -eq 0 ]]; then
  log "WARN: linked issue 없음 (PR 본문에 Resolves #N 필요). 상태 변경 없이 종료."
  exit 0
fi

# 닫힌 issue는 그 item만 terminal no-op으로 표시한다. 다른 열린 issue는 계속
# preflight하되, mutation은 모든 active issue의 검증이 끝날 때까지 시작하지 않는다.
ISSUE_ACTIVE=()
for index in "${!ISSUE_IDS[@]}"; do
  if [[ -z "${ISSUE_IDS[$index]}" || "${ISSUE_IDS[$index]}" == MISSING ||
        -z "${ISSUE_NUMBERS[$index]}" || "${ISSUE_NUMBERS[$index]}" == MISSING ]]; then
    log "WARN: linked issue 응답 불완전"
    exit 1
  fi

  case "${ISSUE_STATES[$index]}" in
    CLOSED)
      ISSUE_ACTIVE[$index]=0
      log "issue #${ISSUE_NUMBERS[$index]} state=CLOSED: per-item terminal no-op"
      ;;
    OPEN) ISSUE_ACTIVE[$index]=1 ;;
    *)
      log "WARN: issue #${ISSUE_NUMBERS[$index]} state 응답 불완전 (${ISSUE_STATES[$index]})"
      exit 1
      ;;
  esac
done

# 2) mutation 전에 모든 열린 issue를 preflight한다. 각 issue의 유일한 Project
# item에서 Project id와 현재 Status를 읽고, exact Project의 Status 설정을 파생한다.
UPDATE_ISSUE_IDS=()
UPDATE_ISSUE_NUMBERS=()
UPDATE_ITEM_IDS=()
UPDATE_PROJECT_IDS=()
UPDATE_PROJECT_NUMBERS=()
UPDATE_FIELD_IDS=()
UPDATE_OPTION_IN_REVIEW_IDS=()
UPDATE_OPTION_IN_PROGRESS_IDS=()
UPDATE_OPTION_READY_IDS=()
UPDATE_CURRENT_STATUSES=()
UPDATE_ELIGIBLE=()

for index in "${!ISSUE_IDS[@]}"; do
  [[ "${ISSUE_ACTIVE[$index]}" -eq 1 ]] || continue
  IID="${ISSUE_IDS[$index]}"
  ISSUE_NO="${ISSUE_NUMBERS[$index]}"
  ITEM_RESULT=""
  ITEM_QUERY_SUCCEEDED=0
  for attempt in 1 2 3; do
    # 이슈 자신의 projectItems를 조회한다. pageInfo가 true/누락이면 일부 item만
    # 보고 임의 선택할 수 있으므로 fail closed한다.
    if ITEM_RESULT=$(query_issue_snapshot "$IID"); then
      ITEM_QUERY_SUCCEEDED=1
      break
    else
      log "WARN: issue node $IID Project item 조회 실패 (attempt $attempt/3)"
    fi
    [[ "$attempt" -eq 3 ]] || sleep 2
  done

  if [[ "$ITEM_QUERY_SUCCEEDED" -eq 0 ]]; then
    log "WARN: issue node $IID Project item 조회 3회 실패"
    exit 1
  fi

  ITEM_ISSUE_STATE=""
  ITEM_HAS_NEXT=""
  ITEM_IDS=()
  PROJECT_IDS=()
  PROJECT_NUMBERS=()
  ITEM_STATUSES=()
  while IFS=$'\t' read -r record value1 value2 value3 value4; do
    [[ -n "$record" ]] || continue
    case "$record" in
      ISSUE_STATE) ITEM_ISSUE_STATE="$value1" ;;
      PAGE) ITEM_HAS_NEXT="$value1" ;;
      ITEM)
        ITEM_IDS+=("$value1")
        PROJECT_IDS+=("$value2")
        PROJECT_NUMBERS+=("$value3")
        ITEM_STATUSES+=("$value4")
        ;;
      *)
        log "WARN: issue #$ISSUE_NO Project item 응답 해석 실패"
        exit 1
        ;;
    esac
  done <<< "$ITEM_RESULT"

  case "$ITEM_ISSUE_STATE" in
    CLOSED)
      log "issue #$ISSUE_NO state=CLOSED: per-item terminal no-op"
      continue
      ;;
    OPEN) ;;
    *)
      log "WARN: issue #$ISSUE_NO state 응답 불완전 ($ITEM_ISSUE_STATE)"
      exit 1
      ;;
  esac

  case "$ITEM_HAS_NEXT" in
    false) ;;
    true)
      log "WARN: issue #$ISSUE_NO Project item pagination 감지. 전체 item을 확인할 수 없어 중단."
      exit 1
      ;;
    *)
      log "WARN: issue #$ISSUE_NO Project item pageInfo 응답 불완전"
      exit 1
      ;;
  esac

  if [[ ${#ITEM_IDS[@]} -eq 0 ]]; then
    log "WARN: issue #$ISSUE_NO Project item 없음 (정확히 하나 필요)"
    exit 1
  fi
  if [[ ${#ITEM_IDS[@]} -ne 1 ]]; then
    log "WARN: issue #$ISSUE_NO Project item 여러 개 (${#ITEM_IDS[@]}개; 정확히 하나 필요)"
    exit 1
  fi

  ITEM_ID="${ITEM_IDS[0]}"
  PROJECT_ID="${PROJECT_IDS[0]}"
  PROJECT_NO="${PROJECT_NUMBERS[0]}"
  ITEM_STATUS="${ITEM_STATUSES[0]}"
  if [[ -z "$ITEM_ID" || "$ITEM_ID" == MISSING ||
        -z "$PROJECT_ID" || "$PROJECT_ID" == MISSING ||
        -z "$PROJECT_NO" || "$PROJECT_NO" == MISSING ||
        -z "$ITEM_STATUS" || "$ITEM_STATUS" == '<unset>' ]]; then
    log "WARN: issue #$ISSUE_NO Project item 응답 불완전"
    exit 1
  fi

  case "$ITEM_STATUS" in
    Done)
      log "issue #$ISSUE_NO Project #$PROJECT_NO Status=Done: per-item terminal no-op"
      continue
      ;;
  esac
  if [[ "$INITIAL_TARGET_STATUS" == Ready && "$ITEM_STATUS" != 'In progress' && "$ITEM_STATUS" != 'In review' ]]; then
    log "issue #$ISSUE_NO Status=$ITEM_STATUS: 닫힌 PR 복귀 대상 아님"
    continue
  fi

  if ! FIELD_RESULT=$(gh api graphql -f query="
    query {
      node(id: \"$PROJECT_ID\") {
        ... on ProjectV2 {
          field(name: \"Status\") {
            ... on ProjectV2SingleSelectField { id options { id name } }
          }
        }
      }
    }
  " --jq '
    .data.node.field as $field |
    if $field != null then
      (["FIELD", ($field.id // "MISSING")] | @tsv),
      ($field.options[]? |
        select(.name == "In review" or .name == "In progress" or .name == "Ready") |
        ["OPTION", (.name // "MISSING"), (.id // "MISSING")] | @tsv)
    else empty end
  ' 2>>"$LOG"); then
    log "WARN: Project #$PROJECT_NO Status 설정 조회 실패"
    exit 1
  fi

  FIELD_IDS=()
  OPTION_IN_REVIEW_IDS=()
  OPTION_IN_PROGRESS_IDS=()
  while IFS=$'\t' read -r record value1 value2 _; do
    [[ -n "$record" ]] || continue
    case "$record" in
      FIELD) FIELD_IDS+=("$value1") ;;
      OPTION)
        case "$value1" in
          'In review') OPTION_IN_REVIEW_IDS+=("$value2") ;;
          'In progress') OPTION_IN_PROGRESS_IDS+=("$value2") ;;
          'Ready') OPTION_READY_IDS+=("$value2") ;;
          *)
            log "WARN: Project #$PROJECT_NO Status option 응답 해석 실패 ($value1)"
            exit 1
            ;;
        esac
        ;;
      *)
        log "WARN: Project #$PROJECT_NO Status 설정 응답 해석 실패"
        exit 1
        ;;
    esac
  done <<< "$FIELD_RESULT"

  if [[ ${#FIELD_IDS[@]} -ne 1 || -z "${FIELD_IDS[0]:-}" || "${FIELD_IDS[0]:-}" == MISSING ]]; then
    log "WARN: Project #$PROJECT_NO Status 설정 incomplete"
    exit 1
  fi
  if [[ ${#OPTION_IN_REVIEW_IDS[@]} -ne 1 ||
        -z "${OPTION_IN_REVIEW_IDS[0]:-}" || "${OPTION_IN_REVIEW_IDS[0]:-}" == MISSING ]]; then
    log "WARN: Project #$PROJECT_NO에 'In review' 옵션 없음 또는 중복 (보드 Status 칸 확인)"
    exit 1
  fi
  if [[ ${#OPTION_IN_PROGRESS_IDS[@]} -ne 1 ||
        -z "${OPTION_IN_PROGRESS_IDS[0]:-}" || "${OPTION_IN_PROGRESS_IDS[0]:-}" == MISSING ]]; then
    log "WARN: Project #$PROJECT_NO에 'In progress' 옵션 없음 또는 중복 (보드 Status 칸 확인)"
    exit 1
  fi

  UPDATE_ISSUE_IDS+=("$IID")
  UPDATE_ISSUE_NUMBERS+=("$ISSUE_NO")
  UPDATE_ITEM_IDS+=("$ITEM_ID")
  UPDATE_PROJECT_IDS+=("$PROJECT_ID")
  UPDATE_PROJECT_NUMBERS+=("$PROJECT_NO")
  UPDATE_FIELD_IDS+=("${FIELD_IDS[0]}")
  UPDATE_OPTION_IN_REVIEW_IDS+=("${OPTION_IN_REVIEW_IDS[0]}")
  UPDATE_OPTION_IN_PROGRESS_IDS+=("${OPTION_IN_PROGRESS_IDS[0]}")
  UPDATE_OPTION_READY_IDS+=("${OPTION_READY_IDS[0]:-}")
  UPDATE_CURRENT_STATUSES+=("$ITEM_STATUS")
  UPDATE_ELIGIBLE+=(1)
  log "issue #$ISSUE_NO Project #$PROJECT_NO Status=$ITEM_STATUS: preflight 완료"
done

if [[ ${#UPDATE_ITEM_IDS[@]} -eq 0 ]]; then
  log "eligible linked issue 없음: terminal/idempotent no-op 완료"
  exit 0
fi

# 3) preflight와 mutation 사이에 merge/Project automation이 먼저 끝날 수 있으므로,
# 모든 issue와 PR을 다시 읽는다. final recheck가 전부 active/same-item일 때만 mutation
# 단계로 넘어가며, terminal 상태는 성공 no-op, ambiguity/drift는 실패다.
for index in "${!UPDATE_ITEM_IDS[@]}"; do
  IID="${UPDATE_ISSUE_IDS[$index]}"
  ISSUE_NO="${UPDATE_ISSUE_NUMBERS[$index]}"
  if ! FINAL_ITEM_RESULT=$(query_issue_snapshot "$IID"); then
    log "WARN: issue #$ISSUE_NO final Project item 조회 실패"
    exit 1
  fi

  FINAL_ISSUE_STATE=""
  FINAL_ITEM_HAS_NEXT=""
  FINAL_ITEM_IDS=()
  FINAL_PROJECT_IDS=()
  FINAL_PROJECT_NUMBERS=()
  FINAL_ITEM_STATUSES=()
  while IFS=$'\t' read -r record value1 value2 value3 value4; do
    [[ -n "$record" ]] || continue
    case "$record" in
      ISSUE_STATE) FINAL_ISSUE_STATE="$value1" ;;
      PAGE) FINAL_ITEM_HAS_NEXT="$value1" ;;
      ITEM)
        FINAL_ITEM_IDS+=("$value1")
        FINAL_PROJECT_IDS+=("$value2")
        FINAL_PROJECT_NUMBERS+=("$value3")
        FINAL_ITEM_STATUSES+=("$value4")
        ;;
      *)
        log "WARN: issue #$ISSUE_NO final Project item 응답 해석 실패"
        exit 1
        ;;
    esac
  done <<< "$FINAL_ITEM_RESULT"

  case "$FINAL_ISSUE_STATE" in
    CLOSED)
      UPDATE_ELIGIBLE[$index]=0
      log "issue #$ISSUE_NO final state=CLOSED: per-item terminal no-op"
      continue
      ;;
    OPEN) ;;
    *)
      log "WARN: issue #$ISSUE_NO final state 응답 불완전 ($FINAL_ISSUE_STATE)"
      exit 1
      ;;
  esac

  case "$FINAL_ITEM_HAS_NEXT" in
    false) ;;
    true)
      log "WARN: issue #$ISSUE_NO final Project item pagination 감지"
      exit 1
      ;;
    *)
      log "WARN: issue #$ISSUE_NO final Project item pageInfo 응답 불완전"
      exit 1
      ;;
  esac

  if [[ ${#FINAL_ITEM_IDS[@]} -eq 0 ]]; then
    log "WARN: issue #$ISSUE_NO final Project item 없음 (정확히 하나 필요)"
    exit 1
  fi
  if [[ ${#FINAL_ITEM_IDS[@]} -ne 1 ]]; then
    log "WARN: issue #$ISSUE_NO final Project item 여러 개 (${#FINAL_ITEM_IDS[@]}개; 정확히 하나 필요)"
    exit 1
  fi

  FINAL_ITEM_ID="${FINAL_ITEM_IDS[0]}"
  FINAL_PROJECT_ID="${FINAL_PROJECT_IDS[0]}"
  FINAL_PROJECT_NO="${FINAL_PROJECT_NUMBERS[0]}"
  FINAL_ITEM_STATUS="${FINAL_ITEM_STATUSES[0]}"
  if [[ -z "$FINAL_ITEM_ID" || "$FINAL_ITEM_ID" == MISSING ||
        -z "$FINAL_PROJECT_ID" || "$FINAL_PROJECT_ID" == MISSING ||
        -z "$FINAL_PROJECT_NO" || "$FINAL_PROJECT_NO" == MISSING ||
        -z "$FINAL_ITEM_STATUS" || "$FINAL_ITEM_STATUS" == '<unset>' ]]; then
    log "WARN: issue #$ISSUE_NO final Project item 응답 불완전"
    exit 1
  fi

  if [[ "$FINAL_ITEM_ID" != "${UPDATE_ITEM_IDS[$index]}" ||
        "$FINAL_PROJECT_ID" != "${UPDATE_PROJECT_IDS[$index]}" ||
        "$FINAL_PROJECT_NO" != "${UPDATE_PROJECT_NUMBERS[$index]}" ]]; then
    log "WARN: issue #$ISSUE_NO final Project item 변경 감지"
    exit 1
  fi

  case "$FINAL_ITEM_STATUS" in
    Done)
      UPDATE_ELIGIBLE[$index]=0
      log "issue #$ISSUE_NO final Status=Done: per-item terminal no-op"
      continue
      ;;
  esac
  UPDATE_CURRENT_STATUSES[$index]="$FINAL_ITEM_STATUS"
  log "issue #$ISSUE_NO final Status=$FINAL_ITEM_STATUS: mutation eligible"
done

if ! FINAL_PR_RESULT=$(query_pr_snapshot); then
  log "WARN: final PR 및 linked issue 조회 실패"
  exit 1
fi

FINAL_PR_STATE=""
FINAL_PR_IS_DRAFT=""
FINAL_CLOSING_HAS_NEXT=""
FINAL_ISSUE_IDS=()
FINAL_ISSUE_NUMBERS=()
FINAL_ISSUE_STATES=()
while IFS=$'\t' read -r record value1 value2 value3 value4; do
  [[ -n "$record" ]] || continue
  case "$record" in
    PR)
      FINAL_PR_STATE="$value1"
      FINAL_PR_IS_DRAFT="$value2"
      FINAL_CLOSING_HAS_NEXT="$value3"
      ;;
    ISSUE)
      FINAL_ISSUE_IDS+=("$value1")
      FINAL_ISSUE_NUMBERS+=("$value2")
      FINAL_ISSUE_STATES+=("$value3")
      ;;
    *)
      log "WARN: final PR/linked issue 응답 해석 실패"
      exit 1
      ;;
  esac
done <<< "$FINAL_PR_RESULT"

case "$FINAL_PR_STATE" in
  MERGED)
    log "PR #$PR final state=$FINAL_PR_STATE: stale workflow no-op"
    exit 0
    ;;
  CLOSED) TARGET_STATUS='Ready' ;;
  OPEN)
    case "$FINAL_PR_IS_DRAFT" in
      true) TARGET_STATUS='In progress' ;;
      false) TARGET_STATUS='In review' ;;
      *)
        log "WARN: PR #$PR final isDraft 응답 불완전 ($FINAL_PR_IS_DRAFT)"
        exit 1
        ;;
    esac
    ;;
  *)
    log "WARN: PR #$PR final state 응답 불완전 ($FINAL_PR_STATE)"
    exit 1
    ;;
esac

case "$FINAL_CLOSING_HAS_NEXT" in
  false) ;;
  true)
    log "WARN: final linked issue pagination 감지"
    exit 1
    ;;
  *)
    log "WARN: final linked issue pageInfo 응답 불완전"
    exit 1
    ;;
esac

if [[ ${#FINAL_ISSUE_IDS[@]} -ne ${#ISSUE_IDS[@]} ]]; then
  log "WARN: final linked issue 목록 변경 감지"
  exit 1
fi

FINAL_MATCHED=()
for _ in "${ISSUE_IDS[@]}"; do FINAL_MATCHED+=(0); done
for final_index in "${!FINAL_ISSUE_IDS[@]}"; do
  FINAL_IID="${FINAL_ISSUE_IDS[$final_index]}"
  FINAL_ISSUE_NO="${FINAL_ISSUE_NUMBERS[$final_index]}"
  MATCHED_INDEX=-1
  for expected_index in "${!ISSUE_IDS[@]}"; do
    if [[ "$FINAL_IID" == "${ISSUE_IDS[$expected_index]}" ]]; then
      MATCHED_INDEX="$expected_index"
      break
    fi
  done

  if [[ "$MATCHED_INDEX" -lt 0 || "${FINAL_MATCHED[$MATCHED_INDEX]:-0}" -ne 0 ||
        "$FINAL_ISSUE_NO" != "${ISSUE_NUMBERS[$MATCHED_INDEX]:-}" ]]; then
    log "WARN: final linked issue 목록 변경 또는 중복 감지"
    exit 1
  fi
  FINAL_MATCHED[$MATCHED_INDEX]=1

  case "${FINAL_ISSUE_STATES[$final_index]}" in
    CLOSED)
      for update_index in "${!UPDATE_ISSUE_IDS[@]}"; do
        if [[ "$FINAL_IID" == "${UPDATE_ISSUE_IDS[$update_index]}" ]]; then
          UPDATE_ELIGIBLE[$update_index]=0
          break
        fi
      done
      log "issue #$FINAL_ISSUE_NO final state=CLOSED: per-item terminal no-op"
      ;;
    OPEN) ;;
    *)
      log "WARN: issue #$FINAL_ISSUE_NO final state 응답 불완전 (${FINAL_ISSUE_STATES[$final_index]})"
      exit 1
      ;;
  esac
done

# 최종 live PR 상태가 target의 정본이다. 같은 target은 idempotent no-op이고,
# Done/closed는 위에서 terminal로 제외되었다.
for index in "${!UPDATE_ITEM_IDS[@]}"; do
  [[ "${UPDATE_ELIGIBLE[$index]}" -eq 1 ]] || continue
  if [[ "${UPDATE_CURRENT_STATUSES[$index]}" == "$TARGET_STATUS" ]]; then
    UPDATE_ELIGIBLE[$index]=0
    log "issue #${UPDATE_ISSUE_NUMBERS[$index]} final Status=$TARGET_STATUS: per-item idempotent no-op"
  elif [[ "$TARGET_STATUS" == Ready && "${UPDATE_CURRENT_STATUSES[$index]}" != 'In progress' &&
          "${UPDATE_CURRENT_STATUSES[$index]}" != 'In review' ]]; then
    UPDATE_ELIGIBLE[$index]=0
    log "issue #${UPDATE_ISSUE_NUMBERS[$index]} final Status=${UPDATE_CURRENT_STATUSES[$index]}: 닫힌 PR 복귀 대상 아님"
  fi
done

# 4) 모든 final recheck가 끝난 뒤에만 live target mutation을 실행한다.
FAILURES=0
MUTATIONS=0
for index in "${!UPDATE_ITEM_IDS[@]}"; do
  [[ "${UPDATE_ELIGIBLE[$index]}" -eq 1 ]] || continue
  IID="${UPDATE_ISSUE_IDS[$index]}"
  ITEM_ID="${UPDATE_ITEM_IDS[$index]}"
  PROJECT_ID="${UPDATE_PROJECT_IDS[$index]}"
  STATUS_FIELD_ID="${UPDATE_FIELD_IDS[$index]}"
  case "$TARGET_STATUS" in
    'In review') STATUS_OPTION_ID="${UPDATE_OPTION_IN_REVIEW_IDS[$index]}" ;;
    'In progress') STATUS_OPTION_ID="${UPDATE_OPTION_IN_PROGRESS_IDS[$index]}" ;;
    'Ready') STATUS_OPTION_ID="${UPDATE_OPTION_READY_IDS[$index]}" ;;
  esac

  if [[ -z "$STATUS_OPTION_ID" ]]; then
    log "WARN: Project #${UPDATE_PROJECT_NUMBERS[$index]}에 '$TARGET_STATUS' 옵션 없음"
    FAILURES=$((FAILURES + 1))
    continue
  fi

  if gh api graphql -f query="
    mutation {
      updateProjectV2ItemFieldValue(input: {
        projectId: \"$PROJECT_ID\",
        itemId: \"$ITEM_ID\",
        fieldId: \"$STATUS_FIELD_ID\",
        value: { singleSelectOptionId: \"$STATUS_OPTION_ID\" }
      }) { projectV2Item { id } }
    }
  " >>"$LOG" 2>&1; then
    MUTATIONS=$((MUTATIONS + 1))
    log "issue node $IID -> $TARGET_STATUS"
  else
    log "WARN: $IID Status 변경 실패 (PROJECT_PAT scope 확인)"
    FAILURES=$((FAILURES + 1))
  fi
done

if [[ "$FAILURES" -gt 0 ]]; then
  log "WARN: project-status 실패 ${FAILURES}건"
  exit 1
fi

# mutation 뒤 PR mode를 다시 읽는다. 그 사이 draft/ready가 뒤집혔다면 fresh
# full preflight를 bounded re-exec하여 보상한다. 마지막 postcheck 이후의 race는
# converted_to_draft/ready_for_review event가 같은 concurrency group에서 수렴시킨다.
if [[ "$MUTATIONS" -gt 0 ]]; then
  if ! POST_PR_RESULT=$(query_pr_snapshot); then
    log "WARN: post-mutation PR 조회 실패"
    exit 1
  fi

  POST_PR_STATE=""
  POST_PR_IS_DRAFT=""
  while IFS=$'\t' read -r record value1 value2 _; do
    [[ -n "$record" ]] || continue
    case "$record" in
      PR)
        POST_PR_STATE="$value1"
        POST_PR_IS_DRAFT="$value2"
        ;;
      ISSUE) ;;
      *)
        log "WARN: post-mutation PR 응답 해석 실패"
        exit 1
        ;;
    esac
  done <<< "$POST_PR_RESULT"

  case "$POST_PR_STATE" in
    CLOSED|MERGED)
      log "PR #$PR post-mutation state=$POST_PR_STATE: terminal no-op"
      exit 0
      ;;
    OPEN) ;;
    *)
      log "WARN: PR #$PR post-mutation state 응답 불완전 ($POST_PR_STATE)"
      exit 1
      ;;
  esac

  case "$POST_PR_IS_DRAFT" in
    true) POST_TARGET_STATUS='In progress' ;;
    false) POST_TARGET_STATUS='In review' ;;
    *)
      log "WARN: PR #$PR post-mutation isDraft 응답 불완전 ($POST_PR_IS_DRAFT)"
      exit 1
      ;;
  esac

  if [[ "$POST_TARGET_STATUS" != "$TARGET_STATUS" ]]; then
    if [[ "$RECONCILE_DEPTH" -ge "$MAX_RECONCILE_DEPTH" ]]; then
      log "WARN: PR mode가 반복 변경됨 ($TARGET_STATUS -> $POST_TARGET_STATUS); event retry 필요"
      exit 1
    fi
    log "PR mode 변경 감지 ($TARGET_STATUS -> $POST_TARGET_STATUS): fresh reconciliation pass"
    exec "$0" "$PR" "$((RECONCILE_DEPTH + 1))"
  fi
fi
log "=== done ==="
