#!/usr/bin/env bash
#
# bootstrap-repo.sh [--preflight-only] <owner/new-repo>
# Trust boundary: Notion MCP readback verifies hub parent/title and Project Info direct-child. This shell does not read Notion; it validates only canonical URL form, distinct page IDs, common-page exclusion, and versioned placement.
#
# heven-common template으로 "Use this template"해서 만든 새 private repo에,
# template이 복사해주지 않는 것들을 API로 채운다:
#   - main 브랜치 (template은 default 브랜치=dev만 복사)
#   - repo 설정 (default=dev, Wiki 비활성화, squash option, 머지 후 브랜치 삭제)
#   - 명시적으로 enabled인 기능의 secret/variable
#   - 필수 Project 상태 전환 secret (PROJECT_PAT)
#   - org Project #14를 <repo> Roadmap으로 복사/재사용하고 repo에 연결
#
# 복사된 Project의 Auto-add workflow만 GitHub UI에서 설정해야 한다.
# 끝에 남은 수동 단계 체크리스트를 출력한다.
#
# 사용법:
#   bootstrap-repo.sh skku-heven/heven-ad-2026
#   아래 입력값과 선택한 기능의 조건부 값을 안전하게 미리 export한 뒤 실행한다.
#
# env:
#   REPO_PURPOSE — repo 목적 한 줄
#   NOTION_MODE — enabled 또는 disabled (생략 시 disabled)
#   NOTION_REPO_URL — Notion MCP readback 후 README/AGENTS/runner context에 기록된 repo hub URL
#   NOTION_PROJECT_INFO_URL — 같은 MCP readback으로 확인한 Project Info direct-child URL
#   CODEX_REVIEW — enabled 또는 disabled (생략 시 disabled)
#   DISCORD_NOTIFICATIONS — enabled 또는 disabled
#   DISCORD_MENTIONS — enabled 또는 disabled
#   DISCORD_WEBHOOK — notifications enabled일 때 필수 secret
#   DISCORD_USER_MAP — mentions enabled일 때 필수 repo variable
#   PROJECT_PAT — Project 상태 전환에 필수 secret (scope: project + repo)
#
# 멱등: 여러 번 돌려도 이미 된 건 skip한다.

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd -P)
PREFLIGHT_ONLY=0
if [[ "${1:-}" == --preflight-only ]]; then
  PREFLIGHT_ONLY=1
  shift
fi
if (($# != 1)); then
  echo "사용법: bootstrap-repo.sh [--preflight-only] <owner/new-repo>" >&2
  exit 1
fi
REPO="$1"
[[ "$REPO" == */* ]] || { echo "✗ owner/repo 형식이어야 함: '$REPO'" >&2; exit 1; }
REPO_OWNER="${REPO%%/*}"
REPO_NAME="${REPO#*/}"

REPO_WIKI_URL='https://app.notion.com/p/39c3bf06830080b5a024c7ad91855240'
COMMON_NOTION_GUIDE_URL='https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee'
GETTING_STARTED_URL='https://app.notion.com/p/39c3bf06830081fb8e76d5c9a1be6d82'
TEAM_WORKFLOW_URL='https://app.notion.com/p/39c3bf068300810db8adc15fce65450d'
BRANCHING_MERGE_URL='https://app.notion.com/p/39c3bf06830081a9b1b5c3ba62d26ff6'
CODE_CONVENTIONS_URL='https://app.notion.com/p/39c3bf06830081f08325d07294e5fb08'
REPOSITORY_STRUCTURE_URL='https://app.notion.com/p/39c3bf06830081e4b319efa1ed8f9527'
OPERATIONS_URL='https://app.notion.com/p/39c3bf068300815fb880e1d20602e662'
COMMON_PROJECT_INFO_URL='https://app.notion.com/p/39c3bf0683008121b6e7dea451003a13'

require_value() {
  local name="$1"
  if ! declare -p "$name" >/dev/null 2>&1; then
    echo "✗ 필수 환경변수 $name가 없음" >&2
    exit 1
  fi
  if [[ -z "${!name}" ]]; then
    echo "✗ 필수 환경변수 $name가 비어 있음" >&2
    exit 1
  fi
}

require_mode() {
  local name="$1"
  case "${!name}" in
    enabled|disabled) ;;
    *)
      echo "✗ $name는 정확히 enabled 또는 disabled여야 함" >&2
      exit 1
      ;;
  esac
}

document_exact_url_count() {
  local document="$1"
  local expected_url="$2"

  grep -Eo "https://[^][(){}<>\"'\`[:space:]]+" "$document" \
    | grep -Fxc -- "$expected_url" || true
}

document_has_repo_hub_field() {
  local relative_document="$1"
  local document="$2"
  local expected_url="$3"
  local expected_line=''

  case "$relative_document" in
    README.md)
      printf -v expected_line -- '- 이 repository의 Notion hub: `%s`' "$expected_url"
      ;;
    AGENTS.md)
      printf -v expected_line -- '- 현재 repo hub: `%s`' "$expected_url"
      ;;
    ops/runner/repo-context.sh)
      printf -v expected_line 'export NOTION_GUIDE_URL="${NOTION_GUIDE_URL:-%s}"' "$expected_url"
      ;;
    *)
      return 1
      ;;
  esac

  grep -Fqx -- "$expected_line" "$document"
}

if ((PREFLIGHT_ONLY == 0)); then
  NOTION_MODE="${NOTION_MODE:-disabled}"
  require_mode NOTION_MODE
  CODEX_REVIEW="${CODEX_REVIEW:-disabled}"
  require_mode CODEX_REVIEW
  for required_name in \
      REPO_PURPOSE \
      DISCORD_NOTIFICATIONS DISCORD_MENTIONS; do
    require_value "$required_name"
  done
  if [[ "$NOTION_MODE" == enabled ]]; then
    require_value NOTION_REPO_URL
    require_value NOTION_PROJECT_INFO_URL
    if [[ ! "$NOTION_REPO_URL" =~ ^https://app\.notion\.com/p/[0-9a-f]{32}$ \
       || ! "$NOTION_PROJECT_INFO_URL" =~ ^https://app\.notion\.com/p/[0-9a-f]{32}$ ]]; then
      echo "✗ Notion URL은 canonical https://app.notion.com/p/<32-hex-page-id> 형식이어야 함" >&2
      exit 1
    fi
    if [[ "$NOTION_REPO_URL" == "$NOTION_PROJECT_INFO_URL" ]]; then
      echo "✗ repo hub URL과 Project Info URL은 달라야 함" >&2
      exit 1
    fi
    case "$NOTION_REPO_URL" in
      "$REPO_WIKI_URL"|"$COMMON_NOTION_GUIDE_URL"|"$GETTING_STARTED_URL"|"$TEAM_WORKFLOW_URL"|\
      "$BRANCHING_MERGE_URL"|"$CODE_CONVENTIONS_URL"|"$REPOSITORY_STRUCTURE_URL"|\
      "$OPERATIONS_URL"|"$COMMON_PROJECT_INFO_URL")
        echo "✗ NOTION_REPO_URL에는 공통 문서가 아니라 파생 repo 전용 hub URL이 필요" >&2
        exit 1
        ;;
    esac
    case "$NOTION_PROJECT_INFO_URL" in
      "$REPO_WIKI_URL"|"$COMMON_NOTION_GUIDE_URL"|"$GETTING_STARTED_URL"|"$TEAM_WORKFLOW_URL"|\
      "$BRANCHING_MERGE_URL"|"$CODE_CONVENTIONS_URL"|"$REPOSITORY_STRUCTURE_URL"|\
      "$OPERATIONS_URL"|"$COMMON_PROJECT_INFO_URL")
        echo "✗ NOTION_PROJECT_INFO_URL에는 repo hub의 Project Info child URL이 필요" >&2
        exit 1
        ;;
    esac

  elif [[ -n "${NOTION_REPO_URL:-}" || -n "${NOTION_PROJECT_INFO_URL:-}" ]]; then
    echo "✗ NOTION_MODE=disabled일 때 Notion URL을 제공할 수 없음" >&2
    exit 1
  fi

  for mode_name in DISCORD_NOTIFICATIONS DISCORD_MENTIONS; do
    require_mode "$mode_name"
  done

  if [[ "$DISCORD_MENTIONS" == enabled \
     && "$DISCORD_NOTIFICATIONS" != enabled ]]; then
    echo "✗ DISCORD_MENTIONS=enabled에는 DISCORD_NOTIFICATIONS=enabled가 필요" >&2
    exit 1
  fi

  if [[ "$DISCORD_NOTIFICATIONS" == enabled ]]; then
    require_value DISCORD_WEBHOOK
  elif [[ -n "${DISCORD_WEBHOOK:-}" ]]; then
    echo "✗ DISCORD_NOTIFICATIONS=disabled일 때 DISCORD_WEBHOOK을 제공할 수 없음" >&2
    exit 1
  fi

  if [[ "$DISCORD_MENTIONS" == enabled ]]; then
    require_value DISCORD_USER_MAP
  elif [[ -n "${DISCORD_USER_MAP:-}" ]]; then
    echo "✗ DISCORD_MENTIONS=disabled일 때 DISCORD_USER_MAP을 제공할 수 없음" >&2
    exit 1
  fi

  require_value PROJECT_PAT

  if [[ "$NOTION_MODE" == enabled ]]; then
    for notion_doc in README.md AGENTS.md ops/runner/repo-context.sh; do
      notion_path="$ROOT/$notion_doc"
      if [[ ! -f "$notion_path" ]] \
         || ! document_has_repo_hub_field "$notion_doc" "$notion_path" "$NOTION_REPO_URL" \
         || [[ "$(document_exact_url_count "$notion_path" "$NOTION_PROJECT_INFO_URL")" != 0 ]] \
         || [[ "$(grep -Fc '<NOTION_REPO_URL>' "$notion_path" || true)" != 0 ]]; then
        echo "✗ $notion_doc의 지정 field에는 repo hub URL이 있고 placeholder/Project Info URL은 없어야 함" >&2
        exit 1
      fi
    done
  fi
fi

PROJECT_TEMPLATE_OWNER=skku-heven
PROJECT_TEMPLATE_NUMBER=14
PROJECT_TEMPLATE_TITLE='[TEMPLATE] heven-common'
TARGET_PROJECT_TITLE="$REPO_NAME Roadmap"
declare -A EXPECTED_STATUS_OPTIONS=(
  [Backlog]=1
  [Ready]=1
  ['In progress']=1
  ['In review']=1
  [Done]=1
)
declare -A REQUIRED_PROJECT_WORKFLOWS=(
  ['Auto-add sub-issues to project']=1
  ['Auto-close issue']=1
  ['Code changes requested']=1
  ['Item added to project']=1
  ['Item closed']=1
  ['Pull request linked to issue']=1
  ['Pull request merged']=1
)

WARNS=0
ok()   { echo "✓ $*"; }
skip() { echo "· $*"; }
warn() { echo "⚠ $*" >&2; WARNS=$((WARNS+1)); }

command -v gh >/dev/null || { echo "✗ gh CLI 없음" >&2; exit 1; }

# 모든 mutation 전에 REST metadata를 read-only로 검증한다.
PREFLIGHT=$(gh api "repos/$REPO" \
  --jq '[.visibility, .default_branch, (.permissions.admin // false), (.template_repository.full_name // "")] | @tsv' \
  2>/dev/null) || { echo "✗ repo '$REPO' 접근 불가" >&2; exit 1; }
IFS=$'\t' read -r VISIBILITY DEFAULT_BRANCH IS_ADMIN TEMPLATE_REPOSITORY <<< "$PREFLIGHT"
VISIBILITY="${VISIBILITY,,}"
[[ "$VISIBILITY" == private ]] || {
  echo "✗ '$REPO' visibility=$VISIBILITY. 이 bootstrap은 private repo 전용." >&2
  exit 1
}
[[ "$DEFAULT_BRANCH" == dev ]] || {
  echo "✗ '$REPO' default branch=$DEFAULT_BRANCH. heven-common template의 dev가 필요." >&2
  exit 1
}
[[ "$IS_ADMIN" == true ]] || {
  echo "✗ '$REPO' repository admin 권한이 필요." >&2
  exit 1
}
[[ "$TEMPLATE_REPOSITORY" == skku-heven/heven-common ]] || {
  echo "✗ '$REPO' template source='$TEMPLATE_REPOSITORY'. skku-heven/heven-common에서 생성한 repo가 필요." >&2
  exit 1
}

if ((PREFLIGHT_ONLY)); then
  ok "read-only provenance preflight 완료: $REPO"
  exit 0
fi

PROJECT_TEMPLATE_PREFLIGHT=$(gh api graphql \
  -f query='query($login: String!, $number: Int!) { organization(login: $login) { projectV2(number: $number) { closed template title field(name: "Status") { __typename ... on ProjectV2SingleSelectField { name options { name } } } workflows(first: 100) { pageInfo { hasNextPage } nodes { name enabled } } } } }' \
  -F login="$PROJECT_TEMPLATE_OWNER" \
  -F number="$PROJECT_TEMPLATE_NUMBER" \
  --jq '
    .data.organization.projectV2 as $project |
    (["TEMPLATE",
      (if $project.closed == null then "MISSING" else ($project.closed | tostring) end),
      (if $project.template == null then "MISSING" else ($project.template | tostring) end),
      ($project.title // "")] | @tsv),
    (["STATUS_FIELD", ($project.field.__typename // "MISSING"),
      ($project.field.name // "")] | @tsv),
    ($project.field.options[]? | ["STATUS_OPTION", (.name // "")] | @tsv),
    (["WORKFLOW_PAGE",
      (if $project.workflows.pageInfo.hasNextPage == null then "MISSING" else ($project.workflows.pageInfo.hasNextPage | tostring) end)] | @tsv),
    ($project.workflows.nodes[]? |
      ["WORKFLOW", (.name // ""),
       (if .enabled == null then "MISSING" else (.enabled | tostring) end)] | @tsv)
  ' \
  2>/dev/null) || {
  echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER 조회 실패 (read:project 권한 확인)" >&2
  exit 1
}

PROJECT_TEMPLATE_METADATA_COUNT=0
PROJECT_TEMPLATE_STATUS_FIELD_COUNT=0
PROJECT_TEMPLATE_WORKFLOW_PAGE_COUNT=0
PROJECT_TEMPLATE_CLOSED=''
PROJECT_TEMPLATE_IS_TEMPLATE=''
PROJECT_TEMPLATE_ACTUAL_TITLE=''
PROJECT_TEMPLATE_STATUS_FIELD_TYPE=''
PROJECT_TEMPLATE_STATUS_FIELD_NAME=''
PROJECT_TEMPLATE_WORKFLOW_HAS_NEXT=''
declare -A PROJECT_TEMPLATE_STATUS_OPTIONS=()
declare -A PROJECT_TEMPLATE_WORKFLOW_NAMES=()
declare -A PROJECT_TEMPLATE_REQUIRED_WORKFLOWS=()
while IFS=$'\t' read -r record value1 value2 value3 extra; do
  [[ -n "$record" ]] || continue
  case "$record" in
    TEMPLATE)
      [[ -z "${extra:-}" ]] || {
        echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER metadata 응답 형식이 올바르지 않음" >&2
        exit 1
      }
      PROJECT_TEMPLATE_METADATA_COUNT=$((PROJECT_TEMPLATE_METADATA_COUNT + 1))
      PROJECT_TEMPLATE_CLOSED="$value1"
      PROJECT_TEMPLATE_IS_TEMPLATE="$value2"
      PROJECT_TEMPLATE_ACTUAL_TITLE="$value3"
      ;;
    STATUS_FIELD)
      [[ -z "${value3:-}${extra:-}" ]] || {
        echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER Status 응답 형식이 올바르지 않음" >&2
        exit 1
      }
      PROJECT_TEMPLATE_STATUS_FIELD_COUNT=$((PROJECT_TEMPLATE_STATUS_FIELD_COUNT + 1))
      PROJECT_TEMPLATE_STATUS_FIELD_TYPE="$value1"
      PROJECT_TEMPLATE_STATUS_FIELD_NAME="$value2"
      ;;
    STATUS_OPTION)
      [[ -n "$value1" && -z "${value2:-}${value3:-}${extra:-}" \
         && -z "${PROJECT_TEMPLATE_STATUS_OPTIONS[$value1]+x}" ]] || {
        echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER Status option 구성이 올바르지 않음" >&2
        exit 1
      }
      PROJECT_TEMPLATE_STATUS_OPTIONS["$value1"]=1
      ;;
    WORKFLOW_PAGE)
      [[ -z "${value2:-}${value3:-}${extra:-}" ]] || {
        echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER workflow 응답 형식이 올바르지 않음" >&2
        exit 1
      }
      PROJECT_TEMPLATE_WORKFLOW_PAGE_COUNT=$((PROJECT_TEMPLATE_WORKFLOW_PAGE_COUNT + 1))
      PROJECT_TEMPLATE_WORKFLOW_HAS_NEXT="$value1"
      ;;
    WORKFLOW)
      [[ -n "$value1" && "$value2" =~ ^(true|false)$ \
         && -z "${value3:-}${extra:-}" \
         && -z "${PROJECT_TEMPLATE_WORKFLOW_NAMES[$value1]+x}" ]] || {
        echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER workflow 구성이 올바르지 않음" >&2
        exit 1
      }
      PROJECT_TEMPLATE_WORKFLOW_NAMES["$value1"]=1
      if [[ "$value1" != 'Auto-add to project' ]]; then
        [[ "$value2" == true ]] || {
          echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER workflow '$value1'가 enabled가 아님" >&2
          exit 1
        }
        PROJECT_TEMPLATE_REQUIRED_WORKFLOWS["$value1"]=1
      fi
      ;;
    *)
      echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER 응답에 알 수 없는 record '$record'가 있음" >&2
      exit 1
      ;;
  esac
done <<< "$PROJECT_TEMPLATE_PREFLIGHT"
[[ "$PROJECT_TEMPLATE_CLOSED" == false \
   && "$PROJECT_TEMPLATE_IS_TEMPLATE" == true \
   && "$PROJECT_TEMPLATE_ACTUAL_TITLE" == "$PROJECT_TEMPLATE_TITLE" \
   && "$PROJECT_TEMPLATE_METADATA_COUNT" -eq 1 ]] || {
  echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER가 open template '$PROJECT_TEMPLATE_TITLE'과 일치하지 않음" >&2
  exit 1
}
[[ "$PROJECT_TEMPLATE_STATUS_FIELD_COUNT" -eq 1 \
   && "$PROJECT_TEMPLATE_STATUS_FIELD_TYPE" == ProjectV2SingleSelectField \
   && "$PROJECT_TEMPLATE_STATUS_FIELD_NAME" == Status \
   && "${#PROJECT_TEMPLATE_STATUS_OPTIONS[@]}" -eq "${#EXPECTED_STATUS_OPTIONS[@]}" ]] || {
  echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER Status field/options가 표준과 일치하지 않음" >&2
  exit 1
}
for status_name in "${!EXPECTED_STATUS_OPTIONS[@]}"; do
  [[ -n "${PROJECT_TEMPLATE_STATUS_OPTIONS[$status_name]+x}" ]] || {
    echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER Status option '$status_name'가 없음" >&2
    exit 1
  }
done
[[ "$PROJECT_TEMPLATE_WORKFLOW_PAGE_COUNT" -eq 1 \
   && "$PROJECT_TEMPLATE_WORKFLOW_HAS_NEXT" == false \
   && "${#PROJECT_TEMPLATE_REQUIRED_WORKFLOWS[@]}" -eq "${#REQUIRED_PROJECT_WORKFLOWS[@]}" ]] || {
  echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER workflow 구성이 표준과 일치하지 않음" >&2
  exit 1
}
for workflow_name in "${!REQUIRED_PROJECT_WORKFLOWS[@]}"; do
  [[ -n "${PROJECT_TEMPLATE_REQUIRED_WORKFLOWS[$workflow_name]+x}" ]] || {
    echo "✗ org Project #$PROJECT_TEMPLATE_NUMBER workflow '$workflow_name'가 없음" >&2
    exit 1
  }
done

PROJECT_LIST=$(gh project list \
  --owner "$REPO_OWNER" \
  --limit 1000 \
  --closed \
  --format json \
  --jq '.projects[] | [.number, .title] | @tsv') || {
  echo "✗ '$REPO_OWNER' Project 목록 조회 실패 (위 오류와 CLI의 read:org·project 권한 확인)" >&2
  exit 1
}
TARGET_PROJECT_NUMBERS=()
while IFS=$'\t' read -r project_number project_title; do
  if [[ "$project_title" == "$TARGET_PROJECT_TITLE" ]]; then
    TARGET_PROJECT_NUMBERS+=("$project_number")
  fi
done <<< "$PROJECT_LIST"
if ((${#TARGET_PROJECT_NUMBERS[@]} > 1)); then
  echo "✗ '$TARGET_PROJECT_TITLE' Project가 ${#TARGET_PROJECT_NUMBERS[@]}개라 대상을 결정할 수 없음" >&2
  exit 1
fi
TARGET_PROJECT_NUMBER="${TARGET_PROJECT_NUMBERS[0]:-}"

validate_target_project() {
  local TARGET_PROJECT_NUMBER="$1"
  [[ "$TARGET_PROJECT_NUMBER" =~ ^[0-9]+$ ]] || {
    echo "✗ '$TARGET_PROJECT_TITLE' Project 번호를 확인할 수 없음" >&2
    exit 1
  }
  local TARGET_PROJECT_PREFLIGHT
  TARGET_PROJECT_PREFLIGHT=$(gh api graphql \
    -f query='query($login: String!, $number: Int!) { organization(login: $login) { projectV2(number: $number) { closed field(name: "Status") { __typename ... on ProjectV2SingleSelectField { name options { name } } } workflows(first: 100) { pageInfo { hasNextPage } nodes { name enabled } } } } }' \
    -F login="$REPO_OWNER" \
    -F number="$TARGET_PROJECT_NUMBER" \
    --jq '
      .data.organization.projectV2 as $project |
      (["TARGET",
        (if $project.closed == null then "MISSING" else ($project.closed | tostring) end)] | @tsv),
      (["STATUS_FIELD", ($project.field.__typename // "MISSING"),
        ($project.field.name // "")] | @tsv),
      ($project.field.options[]? | ["STATUS_OPTION", (.name // "")] | @tsv),
      (["WORKFLOW_PAGE",
        (if $project.workflows.pageInfo.hasNextPage == null then "MISSING" else ($project.workflows.pageInfo.hasNextPage | tostring) end)] | @tsv),
      ($project.workflows.nodes[]? |
        ["WORKFLOW", (.name // ""),
         (if .enabled == null then "MISSING" else (.enabled | tostring) end)] | @tsv)
    ' \
    2>/dev/null) || {
    echo "✗ Project '$TARGET_PROJECT_TITLE' (#$TARGET_PROJECT_NUMBER) 구성 조회 실패" >&2
    exit 1
  }

  local TARGET_METADATA_COUNT=0
  local TARGET_STATUS_FIELD_COUNT=0
  local TARGET_WORKFLOW_PAGE_COUNT=0
  local TARGET_PROJECT_CLOSED=''
  local TARGET_STATUS_FIELD_TYPE=''
  local TARGET_STATUS_FIELD_NAME=''
  local TARGET_WORKFLOW_HAS_NEXT=''
  local -A TARGET_STATUS_OPTIONS=()
  local -A TARGET_PROJECT_WORKFLOW_NAMES=()
  local -A TARGET_REQUIRED_PROJECT_WORKFLOWS=()
  while IFS=$'\t' read -r record value1 value2 value3 extra; do
    [[ -n "$record" ]] || continue
    case "$record" in
      TARGET)
        [[ -z "${value2:-}${value3:-}${extra:-}" ]] || {
          echo "✗ Project '$TARGET_PROJECT_TITLE' metadata 응답 형식이 올바르지 않음" >&2
          exit 1
        }
        TARGET_METADATA_COUNT=$((TARGET_METADATA_COUNT + 1))
        TARGET_PROJECT_CLOSED="$value1"
        ;;
      STATUS_FIELD)
        [[ -z "${value3:-}${extra:-}" ]] || {
          echo "✗ Project '$TARGET_PROJECT_TITLE' Status 응답 형식이 올바르지 않음" >&2
          exit 1
        }
        TARGET_STATUS_FIELD_COUNT=$((TARGET_STATUS_FIELD_COUNT + 1))
        TARGET_STATUS_FIELD_TYPE="$value1"
        TARGET_STATUS_FIELD_NAME="$value2"
        ;;
      STATUS_OPTION)
        [[ -n "$value1" && -z "${value2:-}${value3:-}${extra:-}" \
           && -z "${TARGET_STATUS_OPTIONS[$value1]+x}" ]] || {
          echo "✗ Project '$TARGET_PROJECT_TITLE' Status option 구성이 올바르지 않음" >&2
          exit 1
        }
        TARGET_STATUS_OPTIONS["$value1"]=1
        ;;
      WORKFLOW_PAGE)
        [[ -z "${value2:-}${value3:-}${extra:-}" ]] || {
          echo "✗ Project '$TARGET_PROJECT_TITLE' workflow 응답 형식이 올바르지 않음" >&2
          exit 1
        }
        TARGET_WORKFLOW_PAGE_COUNT=$((TARGET_WORKFLOW_PAGE_COUNT + 1))
        TARGET_WORKFLOW_HAS_NEXT="$value1"
        ;;
      WORKFLOW)
        [[ -n "$value1" && "$value2" =~ ^(true|false)$ \
           && -z "${value3:-}${extra:-}" \
           && -z "${TARGET_PROJECT_WORKFLOW_NAMES[$value1]+x}" ]] || {
          echo "✗ Project '$TARGET_PROJECT_TITLE' workflow 구성이 올바르지 않음" >&2
          exit 1
        }
        TARGET_PROJECT_WORKFLOW_NAMES["$value1"]=1
        if [[ "$value1" != 'Auto-add to project' ]]; then
          [[ "$value2" == true ]] || {
            echo "✗ Project '$TARGET_PROJECT_TITLE' workflow '$value1'가 enabled가 아님" >&2
            exit 1
          }
          TARGET_REQUIRED_PROJECT_WORKFLOWS["$value1"]=1
        fi
        ;;
      *)
        echo "✗ Project '$TARGET_PROJECT_TITLE' 응답에 알 수 없는 record '$record'가 있음" >&2
        exit 1
        ;;
    esac
  done <<< "$TARGET_PROJECT_PREFLIGHT"

  [[ "$TARGET_METADATA_COUNT" -eq 1 && "$TARGET_PROJECT_CLOSED" == false ]] || {
    echo "✗ Project '$TARGET_PROJECT_TITLE' (#$TARGET_PROJECT_NUMBER)가 open 상태가 아님" >&2
    exit 1
  }
  [[ "$TARGET_STATUS_FIELD_COUNT" -eq 1 \
     && "$TARGET_STATUS_FIELD_TYPE" == ProjectV2SingleSelectField \
     && "$TARGET_STATUS_FIELD_NAME" == Status \
     && "${#TARGET_STATUS_OPTIONS[@]}" -eq "${#EXPECTED_STATUS_OPTIONS[@]}" ]] || {
    echo "✗ Project '$TARGET_PROJECT_TITLE' Status field/options가 표준과 일치하지 않음" >&2
    exit 1
  }
  for status_name in "${!EXPECTED_STATUS_OPTIONS[@]}"; do
    [[ -n "${TARGET_STATUS_OPTIONS[$status_name]+x}" ]] || {
      echo "✗ Project '$TARGET_PROJECT_TITLE' Status option '$status_name'가 없음" >&2
      exit 1
    }
  done
  [[ "$TARGET_WORKFLOW_PAGE_COUNT" -eq 1 \
     && "$TARGET_WORKFLOW_HAS_NEXT" == false \
     && "${#TARGET_REQUIRED_PROJECT_WORKFLOWS[@]}" -eq "${#REQUIRED_PROJECT_WORKFLOWS[@]}" ]] || {
    echo "✗ Project '$TARGET_PROJECT_TITLE' workflow 구성이 Project #$PROJECT_TEMPLATE_NUMBER와 일치하지 않음" >&2
    exit 1
  }
  for workflow_name in "${!REQUIRED_PROJECT_WORKFLOWS[@]}"; do
    [[ -n "${TARGET_REQUIRED_PROJECT_WORKFLOWS[$workflow_name]+x}" ]] || {
      echo "✗ Project '$TARGET_PROJECT_TITLE' workflow '$workflow_name'가 없음" >&2
      exit 1
    }
  done
}

if [[ -n "$TARGET_PROJECT_NUMBER" ]]; then
  validate_target_project "$TARGET_PROJECT_NUMBER"
fi

echo "=== $REPO private repo 부트스트랩 ==="

# 1. main 브랜치 (template은 default=dev만 복사)
if gh api "repos/$REPO/branches/main" >/dev/null 2>&1; then
  skip "main 이미 있음"
else
  DEVSHA=$(gh api "repos/$REPO/git/ref/heads/dev" --jq .object.sha)
  gh api "repos/$REPO/git/refs" -X POST \
    -f ref="refs/heads/main" -f sha="$DEVSHA" >/dev/null
  ok "main 브랜치 생성 (dev에서)"
fi

# 2. repo 설정
gh api "repos/$REPO" -X PATCH \
  -F delete_branch_on_merge=true \
  -F allow_squash_merge=true \
  -F default_branch=dev \
  -F has_wiki=false >/dev/null
ok "설정: default=dev, Wiki 비활성화, squash option, merge/rebase 유지, 머지 후 브랜치 삭제"

# 3. enabled 기능의 secret
if [[ -n "${PROJECT_PAT:-}" ]]; then
  if printf '%s' "$PROJECT_PAT" \
      | gh secret set PROJECT_PAT -R "$REPO" >/dev/null 2>&1; then
    ok "secret PROJECT_PAT"
  else
    warn "secret PROJECT_PAT 설정 실패 (권한·token 확인)"
  fi
else
  skip "PROJECT_PAT 미제공 — project-status 자동화를 쓰려면 PAT(scope: project+repo)로 수동 설정"
fi
if [[ -n "${DISCORD_WEBHOOK:-}" ]]; then
  if printf '%s' "$DISCORD_WEBHOOK" \
      | gh secret set DISCORD_WEBHOOK -R "$REPO" >/dev/null 2>&1; then
    ok "secret DISCORD_WEBHOOK"
  else
    warn "secret DISCORD_WEBHOOK 설정 실패 (권한·webhook 확인)"
  fi
else
  skip "Discord 알림 OFF"
fi

# 4. 기능 mode에 따른 repo variable
if [[ -n "${DISCORD_USER_MAP:-}" ]]; then
  if gh variable set DISCORD_USER_MAP -R "$REPO" \
      --body "$DISCORD_USER_MAP" >/dev/null 2>&1; then
    ok "repo variable DISCORD_USER_MAP"
  else
    warn "repo variable DISCORD_USER_MAP 설정 실패 (권한·JSON 확인)"
  fi
else
  skip "Discord 멘션 OFF"
fi

set_mode_variable() {
  local variable_name="$1"
  local mode="$2"
  local value=false
  [[ "$mode" == enabled ]] && value=true

  if gh variable set "$variable_name" -R "$REPO" \
      --body "$value" >/dev/null 2>&1; then
    ok "repo variable $variable_name=$value"
  else
    warn "repo variable $variable_name 설정 실패 (권한 확인)"
  fi
}

set_mode_variable ENABLE_DISCORD_NOTIFICATIONS "$DISCORD_NOTIFICATIONS"
set_mode_variable ENABLE_DISCORD_MENTIONS "$DISCORD_MENTIONS"
set_mode_variable ENABLE_CODEX_REVIEW "$CODEX_REVIEW"

# 5. Project #14를 repo별 Roadmap으로 복사/재사용하고 target repo에 연결한다.
if [[ -n "$TARGET_PROJECT_NUMBER" ]]; then
  skip "Project '$TARGET_PROJECT_TITLE' (#$TARGET_PROJECT_NUMBER) 이미 있음"
else
  TARGET_PROJECT_NUMBER=$(gh project copy "$PROJECT_TEMPLATE_NUMBER" \
    --source-owner "$PROJECT_TEMPLATE_OWNER" \
    --target-owner "$REPO_OWNER" \
    --title "$TARGET_PROJECT_TITLE" \
    --format json \
    --jq .number) || {
    echo "✗ Project #$PROJECT_TEMPLATE_NUMBER 복사 실패 (위 오류와 CLI의 read:org·project 권한 확인)" >&2
    exit 1
  }
  [[ "$TARGET_PROJECT_NUMBER" =~ ^[0-9]+$ ]] || {
    echo "✗ 복사된 Project 번호를 확인할 수 없음" >&2
    exit 1
  }
  validate_target_project "$TARGET_PROJECT_NUMBER"
  ok "Project '$TARGET_PROJECT_TITLE' (#$TARGET_PROJECT_NUMBER) 복사"
fi

if gh project link "$TARGET_PROJECT_NUMBER" \
    --owner "$REPO_OWNER" \
    --repo "$REPO_NAME" >/dev/null; then
  ok "Project #$TARGET_PROJECT_NUMBER를 $REPO에 연결"
else
  echo "✗ Project #$TARGET_PROJECT_NUMBER를 $REPO에 연결 실패 (위 오류와 CLI의 read:org·project 권한 확인)" >&2
  exit 1
fi

cat <<'EOF'

=== bootstrap 이후 확인 ===
이 shell은 GitHub API 단계만 수행하며 Notion과 README, AGENTS는 수정하지 않습니다.

1. Project 보드 (핵심):
   - 복사·연결된 '<repo> Roadmap'에서 Auto-add workflow만 설정:
     필터 'is:issue is:open' → Backlog
   - 나머지 Status와 workflow는 Project #14 template에서 자동 복사됨
   - 의도한 팀원이 private repo와 Project 모두에 접근 가능한지 확인
2. README/CI 구성:
   - repo README의 목적·환경·빌드·실행·검증 내용을 실제 값으로 교체하고, 빌드·테스트 CI를 추가
EOF
cat <<'EOF'
3. self-hosted 자동화 runner access와 배포:
   - bootstrap이 선택한 mode에 맞춰 쓴 repo variables/secrets의 이름과 존재 여부 확인
   - org runner group의 public repository access는 항상 차단
   - organization plan이 Selected repositories를 지원하면 이 private repo만 허용
EOF
RUNNER_ONLINE=$(gh api "repos/$REPO/actions/runners" \
  --jq '[.runners[] | select(.status == "online")] | length' 2>/dev/null || echo 0)
if [[ "$RUNNER_ONLINE" == 0 ]]; then
  echo "   ⚠ $REPO에 연결된 online runner 없음: 연결 전까지 Project 상태 전환이 대기합니다 (org runner 사용 시 무시)"
else
  echo "   ✓ 저장소 runner online ${RUNNER_ONLINE}개"
fi
cat <<'EOF'
자세히: heven-common repo의 README (새 저장소 만들기)
EOF
echo
if [[ "$WARNS" -gt 0 ]]; then
  echo "⚠ 경고 ${WARNS}건 — 위 ⚠ 줄들을 해결하고 재실행하세요 (멱등: 된 건 skip됨)" >&2
  exit 1
fi
ok "부트스트랩 API 단계 완료: $REPO"
