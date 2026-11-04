#!/usr/bin/env bash

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd -P)
SOURCE_SCRIPT="$ROOT/.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh"
SKILL="$ROOT/.claude/skills/bootstrap-repo/SKILL.md"
SOURCE_README="$ROOT/README.md"
SOURCE_AGENTS="$ROOT/AGENTS.md"
SOURCE_RUNNER_CONTEXT="$ROOT/ops/runner/repo-context.sh"
NOTIFY_WORKFLOW="$ROOT/.github/workflows/notify-discord.yml"
TARGET='skku-heven/example'
REPO_PURPOSE_VALUE='Example repository for bootstrap regression tests'
COMMON_NOTION_URL='https://app.notion.com/p/39c3bf068300807c9e5bcf6b469f94ee'
REPO_WIKI_URL='https://app.notion.com/p/39c3bf06830080b5a024c7ad91855240'
GETTING_STARTED_URL='https://app.notion.com/p/39c3bf06830081fb8e76d5c9a1be6d82'
TEAM_WORKFLOW_URL='https://app.notion.com/p/39c3bf068300810db8adc15fce65450d'
BRANCHING_MERGE_URL='https://app.notion.com/p/39c3bf06830081a9b1b5c3ba62d26ff6'
CODE_CONVENTIONS_URL='https://app.notion.com/p/39c3bf06830081f08325d07294e5fb08'
REPOSITORY_STRUCTURE_URL='https://app.notion.com/p/39c3bf06830081e4b319efa1ed8f9527'
OPERATIONS_URL='https://app.notion.com/p/39c3bf068300815fb880e1d20602e662'
COMMON_PROJECT_INFO_URL='https://app.notion.com/p/39c3bf0683008121b6e7dea451003a13'
NOTION_REPO_URL_VALUE='https://app.notion.com/p/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'
NOTION_PROJECT_INFO_URL_VALUE='https://app.notion.com/p/bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb'
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/bin"

# Functional tests run the copied template as a derived repository: bootstrap
# must never accept the common guide merely because it already appears in the
# template, and every versioned placeholder must be replaced before mutation.
DERIVED_ROOT="$TMP/derived"
SCRIPT="$DERIVED_ROOT/.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh"
README="$DERIVED_ROOT/README.md"
AGENTS="$DERIVED_ROOT/AGENTS.md"
RUNNER_CONTEXT="$DERIVED_ROOT/ops/runner/repo-context.sh"
mkdir -p "$(dirname "$SCRIPT")" "$DERIVED_ROOT/ops/runner"
cp "$SOURCE_SCRIPT" "$SCRIPT"
cp "$SOURCE_README" "$README"
cp "$SOURCE_AGENTS" "$AGENTS"
cp "$SOURCE_RUNNER_CONTEXT" "$RUNNER_CONTEXT"
for derived_doc in "$README" "$AGENTS" "$RUNNER_CONTEXT"; do
  sed -i "s|<NOTION_REPO_URL>|$NOTION_REPO_URL_VALUE|g" "$derived_doc"
done

export GH_CALL_LOG="$TMP/gh-calls.log"
export GH_MUTATION_LOG="$TMP/gh-mutations.log"
export GH_SECRET_STDIN_LOG="$TMP/gh-secret-stdin.log"

cat > "$TMP/bin/gh" <<'FAKE_GH'
#!/usr/bin/env bash
set -Eeuo pipefail

printf '%q ' "$@" >> "$GH_CALL_LOG"
printf '\n' >> "$GH_CALL_LOG"

args=("$@")
mutation=0
if [[ "${1:-}" == api ]]; then
  for ((i = 0; i < ${#args[@]}; i++)); do
    if [[ "${args[$i]}" == -X && "${args[$((i + 1))]:-GET}" != GET ]]; then
      mutation=1
    fi
  done
elif [[ "${1:-} ${2:-}" == 'label create' \
     || "${1:-} ${2:-}" == 'secret set' \
     || "${1:-} ${2:-}" == 'variable set' \
     || "${1:-} ${2:-}" == 'project copy' \
     || "${1:-} ${2:-}" == 'project link' ]]; then
  mutation=1
fi
if ((mutation)); then
  printf '%q ' "$@" >> "$GH_MUTATION_LOG"
  printf '\n' >> "$GH_MUTATION_LOG"
fi

case "${1:-} ${2:-}" in
  'repo view')
    visibility="${FAKE_VISIBILITY:-private}"
    printf '%s\n' "${visibility^^}"
    exit 0
    ;;
  'label list')
    exit 0
    ;;
  'label create')
    exit 0
    ;;
  'secret set')
    name="${3:?secret name missing}"
    value=''
    body_in_argv=0
    for ((i = 0; i < ${#args[@]}; i++)); do
      if [[ "${args[$i]}" == --body ]]; then
        value="${args[$((i + 1))]:-}"
        body_in_argv=1
      fi
    done
    if ((body_in_argv == 0)); then
      value=$(cat)
    fi
    printf '%s\t%s\n' "$name" "$value" >> "$GH_SECRET_STDIN_LOG"
    [[ "${FAKE_FAIL_WRITES:-0}" != 1 ]] || exit 71
    exit 0
    ;;
  'variable set')
    [[ "${FAKE_FAIL_WRITES:-0}" != 1 ]] || exit 72
    exit 0
    ;;
  'project list')
    printf '%s' "${FAKE_EXISTING_PROJECTS:-}"
    exit 0
    ;;
  'project copy')
    if [[ "${FAKE_FAIL_PROJECT_COPY:-0}" == 1 ]]; then
      echo 'gh: copy requires read:org' >&2
      exit 73
    fi
    printf '%s\n' "${FAKE_COPIED_PROJECT_NUMBER:-99}"
    exit 0
    ;;
  'project link')
    if [[ "${FAKE_FAIL_PROJECT_LINK:-0}" == 1 ]]; then
      echo 'gh: repository is not accessible for linking' >&2
      exit 74
    fi
    exit 0
    ;;
esac

if [[ "${1:-}" == api && "${2:-}" == */actions/runners ]]; then
  printf '%s\n' "${FAKE_RUNNERS_ONLINE:-0}"
  exit 0
fi

if [[ "${1:-}" == api ]]; then
  if [[ "${2:-}" == graphql ]]; then
    [[ " $* " == *' login=skku-heven '* ]] || {
      printf 'Project template owner was not skku-heven\n' >&2
      exit 92
    }
    project_number=''
    for arg in "${args[@]}"; do
      [[ "$arg" == number=* ]] && project_number="${arg#number=}"
    done

    if [[ " $* " == *' workflows(first: 100) '* ]]; then
      default_workflows=$'Auto-add to project\ttrue\nAuto-add sub-issues to project\ttrue\nAuto-close issue\ttrue\nCode changes requested\ttrue\nItem added to project\ttrue\nItem closed\ttrue\nPull request linked to issue\ttrue\nPull request merged\ttrue'
      case "$project_number" in
        14)
          [[ "${FAKE_TEMPLATE_CONFIG_QUERY_FAIL:-0}" != 1 ]] || exit 94
          printf 'TEMPLATE\t%s\t%s\t%s\n' \
            "${FAKE_TEMPLATE_PROJECT_CLOSED:-false}" \
            "${FAKE_TEMPLATE_PROJECT_TEMPLATE:-true}" \
            "${FAKE_TEMPLATE_PROJECT_TITLE:-[TEMPLATE] heven-common}"
          if [[ " $* " == *' field(name: "Status") '* ]]; then
            printf 'STATUS_FIELD\t%s\t%s\n' \
              "${FAKE_TEMPLATE_STATUS_FIELD_TYPE:-ProjectV2SingleSelectField}" \
              "${FAKE_TEMPLATE_STATUS_FIELD_NAME:-Status}"
            template_status_options=$'Backlog\nReady\nIn progress\nIn review\nDone'
            if [[ -n "${FAKE_TEMPLATE_STATUS_OPTIONS+x}" ]]; then
              template_status_options="$FAKE_TEMPLATE_STATUS_OPTIONS"
            fi
            while IFS= read -r status_name; do
              [[ -n "$status_name" ]] || continue
              printf 'STATUS_OPTION\t%s\n' "$status_name"
            done <<< "$template_status_options"
          fi
          printf 'WORKFLOW_PAGE\t%s\n' \
            "${FAKE_TEMPLATE_WORKFLOW_HAS_NEXT:-false}"
          template_workflows="$default_workflows"
          if [[ -n "${FAKE_TEMPLATE_WORKFLOWS+x}" ]]; then
            template_workflows="$FAKE_TEMPLATE_WORKFLOWS"
          fi
          while IFS=$'\t' read -r workflow_name workflow_enabled; do
            [[ -n "$workflow_name" ]] || continue
            printf 'WORKFLOW\t%s\t%s\n' "$workflow_name" "$workflow_enabled"
          done <<< "$template_workflows"
          exit 0
          ;;
        42)
          [[ "${FAKE_TARGET_CONFIG_QUERY_FAIL:-0}" != 1 ]] || exit 95
          printf 'TARGET\t%s\n' "${FAKE_TARGET_PROJECT_CLOSED:-false}"
          printf 'STATUS_FIELD\t%s\t%s\n' \
            "${FAKE_TARGET_STATUS_FIELD_TYPE:-ProjectV2SingleSelectField}" \
            "${FAKE_TARGET_STATUS_FIELD_NAME:-Status}"
          target_status_options=$'Backlog\nReady\nIn progress\nIn review\nDone'
          if [[ -n "${FAKE_TARGET_STATUS_OPTIONS+x}" ]]; then
            target_status_options="$FAKE_TARGET_STATUS_OPTIONS"
          fi
          while IFS= read -r status_name; do
            [[ -n "$status_name" ]] || continue
            printf 'STATUS_OPTION\t%s\n' "$status_name"
          done <<< "$target_status_options"
          printf 'WORKFLOW_PAGE\t%s\n' \
            "${FAKE_TARGET_WORKFLOW_HAS_NEXT:-false}"
          target_workflows="$default_workflows"
          if [[ -n "${FAKE_TARGET_WORKFLOWS+x}" ]]; then
            target_workflows="$FAKE_TARGET_WORKFLOWS"
          fi
          while IFS=$'\t' read -r workflow_name workflow_enabled; do
            [[ -n "$workflow_name" ]] || continue
            printf 'WORKFLOW\t%s\t%s\n' "$workflow_name" "$workflow_enabled"
          done <<< "$target_workflows"
          exit 0
          ;;
        99)
          [[ "${FAKE_COPIED_CONFIG_QUERY_FAIL:-0}" != 1 ]] || exit 97
          printf 'TARGET\t%s\n' "${FAKE_COPIED_PROJECT_CLOSED:-false}"
          printf 'STATUS_FIELD\t%s\t%s\n' \
            "${FAKE_COPIED_STATUS_FIELD_TYPE:-ProjectV2SingleSelectField}" \
            "${FAKE_COPIED_STATUS_FIELD_NAME:-Status}"
          copied_status_options=$'Backlog\nReady\nIn progress\nIn review\nDone'
          if [[ -n "${FAKE_COPIED_STATUS_OPTIONS+x}" ]]; then
            copied_status_options="$FAKE_COPIED_STATUS_OPTIONS"
          fi
          while IFS= read -r status_name; do
            [[ -n "$status_name" ]] || continue
            printf 'STATUS_OPTION\t%s\n' "$status_name"
          done <<< "$copied_status_options"
          printf 'WORKFLOW_PAGE\t%s\n' \
            "${FAKE_COPIED_WORKFLOW_HAS_NEXT:-false}"
          copied_workflows="$default_workflows"
          if [[ -n "${FAKE_COPIED_WORKFLOWS+x}" ]]; then
            copied_workflows="$FAKE_COPIED_WORKFLOWS"
          fi
          while IFS=$'\t' read -r workflow_name workflow_enabled; do
            [[ -n "$workflow_name" ]] || continue
            printf 'WORKFLOW\t%s\t%s\n' "$workflow_name" "$workflow_enabled"
          done <<< "$copied_workflows"
          exit 0
          ;;
        *)
          printf 'Unexpected Project number for config query: %s\n' \
            "$project_number" >&2
          exit 96
          ;;
      esac
    fi

    [[ "$project_number" == 14 ]] || {
      printf 'Project template number was not 14\n' >&2
      exit 93
    }
    printf '%s\t%s\t%s\n' \
      "${FAKE_TEMPLATE_PROJECT_CLOSED:-false}" \
      "${FAKE_TEMPLATE_PROJECT_TEMPLATE:-true}" \
      "${FAKE_TEMPLATE_PROJECT_TITLE:-[TEMPLATE] heven-common}"
    exit 0
  fi

  endpoint=''
  method=GET
  for ((i = 0; i < ${#args[@]}; i++)); do
    [[ "${args[$i]}" == repos/* ]] && endpoint="${args[$i]}"
    if [[ "${args[$i]}" == -X ]]; then
      method="${args[$((i + 1))]:-GET}"
    fi
  done

  case "$endpoint" in
    "repos/skku-heven/example")
      if [[ "$method" == GET ]]; then
        if [[ " $* " == *template_repository* ]]; then
          printf '%s\t%s\t%s\t%s\n' \
            "${FAKE_VISIBILITY:-private}" \
            "${FAKE_DEFAULT_BRANCH:-dev}" \
            "${FAKE_ADMIN:-true}" \
            "${FAKE_TEMPLATE_REPOSITORY-skku-heven/heven-common}"
        else
          printf '%s\t%s\t%s\n' \
            "${FAKE_VISIBILITY:-private}" \
            "${FAKE_DEFAULT_BRANCH:-dev}" \
            "${FAKE_ADMIN:-true}"
        fi
      fi
      exit 0
      ;;
    "repos/skku-heven/example/branches/main")
      [[ "${FAKE_MAIN_EXISTS:-0}" == 1 ]]
      exit
      ;;
    "repos/skku-heven/example/git/ref/heads/dev")
      printf 'fake-dev-sha\n'
      exit 0
      ;;
    "repos/skku-heven/example/git/refs")
      exit 0
      ;;
    *)
      printf 'unexpected gh api endpoint: %s\n' "$endpoint" >&2
      exit 90
      ;;
  esac
fi

printf 'unexpected gh call:' >&2
printf ' %q' "$@" >&2
printf '\n' >&2
exit 91
FAKE_GH
chmod +x "$TMP/bin/gh"
export PATH="$TMP/bin:$PATH"

DISABLED_CONTEXT_ENV=(
  REPO_PURPOSE="$REPO_PURPOSE_VALUE"
  NOTION_MODE=enabled
  NOTION_REPO_URL="$NOTION_REPO_URL_VALUE"
  NOTION_PROJECT_INFO_URL="$NOTION_PROJECT_INFO_URL_VALUE"
  CODEX_REVIEW=disabled
  DISCORD_NOTIFICATIONS=disabled
  DISCORD_MENTIONS=disabled
  PROJECT_PAT='runner-project-secret'
  DISCORD_WEBHOOK=
  DISCORD_USER_MAP=
)

TARGET_WORKFLOWS_WITHOUT_AUTO_ADD=$'Auto-add sub-issues to project\ttrue\nAuto-close issue\ttrue\nCode changes requested\ttrue\nItem added to project\ttrue\nItem closed\ttrue\nPull request linked to issue\ttrue\nPull request merged\ttrue'
TARGET_WORKFLOWS_MISSING_MERGED=$'Auto-add sub-issues to project\ttrue\nAuto-close issue\ttrue\nCode changes requested\ttrue\nItem added to project\ttrue\nItem closed\ttrue\nPull request linked to issue\ttrue'
TARGET_WORKFLOWS_EXTRA=$'Auto-add sub-issues to project\ttrue\nAuto-close issue\ttrue\nCode changes requested\ttrue\nItem added to project\ttrue\nItem closed\ttrue\nPull request linked to issue\ttrue\nPull request merged\ttrue\nCustom triage workflow\ttrue'
TARGET_WORKFLOWS_DISABLED_CLOSED=$'Auto-add sub-issues to project\ttrue\nAuto-close issue\ttrue\nCode changes requested\ttrue\nItem added to project\ttrue\nItem closed\tfalse\nPull request linked to issue\ttrue\nPull request merged\ttrue'

failures=0
fail() {
  printf 'not ok - %s\n' "$1" >&2
  failures=$((failures + 1))
}

reset_logs() {
  : > "$GH_CALL_LOG"
  : > "$GH_MUTATION_LOG"
  : > "$GH_SECRET_STDIN_LOG"
}

expect_script_rejected_without_mutation() {
  local label="$1"
  local script="$2"
  shift 2
  local output="$TMP/${label// /-}.out"

  reset_logs
  if env \
      "${DISABLED_CONTEXT_ENV[@]}" \
      FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
      FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
      FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
      FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
      FAKE_EXISTING_PROJECTS= FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=0 \
      FAKE_MAIN_EXISTS=0 FAKE_FAIL_WRITES=0 \
      "$@" "$script" "$TARGET" > "$output" 2>&1; then
    fail "$label unexpectedly succeeded"
  fi
  if [[ -s "$GH_MUTATION_LOG" ]]; then
    printf '%s\n' "$(<"$GH_MUTATION_LOG")" >&2
    fail "$label performed mutation-capable gh calls"
  fi
  printf 'ok - %s rejected before mutation\n' "$label"
}

expect_rejected_without_mutation() {
  local label="$1"
  shift
  expect_script_rejected_without_mutation "$label" "$SCRIPT" "$@"
}

make_missing_notion_fixture() {
  local label="$1"
  local relative_doc="$2"
  local fixture_root="$TMP/notion-fixture-$label"
  local fixture_script="$fixture_root/.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh"

  mkdir -p "$(dirname "$fixture_script")" "$fixture_root/ops/runner"
  cp "$SCRIPT" "$fixture_script"
  cp "$README" "$fixture_root/README.md"
  cp "$AGENTS" "$fixture_root/AGENTS.md"
  cp "$RUNNER_CONTEXT" "$fixture_root/ops/runner/repo-context.sh"
  sed -i "s|$NOTION_REPO_URL_VALUE|https://example.invalid/missing-notion-url|g" \
    "$fixture_root/$relative_doc"
  printf '%s\n' "$fixture_script"
}

expect_rejected_without_mutation 'missing REPO_PURPOSE' REPO_PURPOSE=
expect_rejected_without_mutation 'missing NOTION_REPO_URL' NOTION_REPO_URL=
expect_rejected_without_mutation 'missing NOTION_PROJECT_INFO_URL' \
  NOTION_PROJECT_INFO_URL=
expect_rejected_without_mutation 'missing DISCORD_NOTIFICATIONS' DISCORD_NOTIFICATIONS=
expect_rejected_without_mutation 'missing DISCORD_MENTIONS' DISCORD_MENTIONS=
expect_rejected_without_mutation 'invalid CODEX_REVIEW mode' CODEX_REVIEW=unknown
expect_rejected_without_mutation 'invalid DISCORD_NOTIFICATIONS mode' \
  DISCORD_NOTIFICATIONS=yes
expect_rejected_without_mutation 'invalid DISCORD_MENTIONS mode' \
  DISCORD_MENTIONS=yes
expect_rejected_without_mutation 'enabled notifications without webhook' \
  DISCORD_NOTIFICATIONS=enabled
expect_rejected_without_mutation 'enabled mentions without user map' \
  DISCORD_NOTIFICATIONS=enabled DISCORD_WEBHOOK='https://discord.invalid/webhook' \
  DISCORD_MENTIONS=enabled
expect_rejected_without_mutation 'enabled runner without PROJECT_PAT' \
  PROJECT_PAT=
expect_rejected_without_mutation 'webhook supplied while notifications disabled' \
  DISCORD_WEBHOOK='https://discord.invalid/webhook'
expect_rejected_without_mutation 'user map supplied while mentions disabled' \
  DISCORD_USER_MAP='{"member":"123"}'
expect_rejected_without_mutation 'mentions enabled while notifications disabled' \
  DISCORD_MENTIONS=enabled DISCORD_USER_MAP='{"member":"123"}'

while IFS='|' read -r common_page_label common_page_url; do
  common_hub_fixture_root="$TMP/notion-fixture-$common_page_label"
  common_hub_fixture="$common_hub_fixture_root/.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh"
  mkdir -p "$(dirname "$common_hub_fixture")" "$common_hub_fixture_root/ops/runner"
  cp "$SCRIPT" "$common_hub_fixture"
  cp "$README" "$common_hub_fixture_root/README.md"
  cp "$AGENTS" "$common_hub_fixture_root/AGENTS.md"
  cp "$RUNNER_CONTEXT" "$common_hub_fixture_root/ops/runner/repo-context.sh"
  for common_hub_doc in \
      "$common_hub_fixture_root/README.md" \
      "$common_hub_fixture_root/AGENTS.md" \
      "$common_hub_fixture_root/ops/runner/repo-context.sh"; do
    sed -i "s|$NOTION_REPO_URL_VALUE|$common_page_url|g" "$common_hub_doc"
  done
  expect_script_rejected_without_mutation "$common_page_label used as repo hub" \
    "$common_hub_fixture" NOTION_REPO_URL="$common_page_url"
done <<EOF
repo-wiki-parent|$REPO_WIKI_URL
common-guide|$COMMON_NOTION_URL
getting-started|$GETTING_STARTED_URL
team-workflow|$TEAM_WORKFLOW_URL
branching-and-merge|$BRANCHING_MERGE_URL
code-conventions|$CODE_CONVENTIONS_URL
repository-structure|$REPOSITORY_STRUCTURE_URL
operations|$OPERATIONS_URL
common-project-info|$COMMON_PROJECT_INFO_URL
EOF
expect_rejected_without_mutation 'repo hub equals Project Info URL' \
  NOTION_PROJECT_INFO_URL="$NOTION_REPO_URL_VALUE"
expect_script_rejected_without_mutation 'template placeholder left unchanged' \
  "$SOURCE_SCRIPT" NOTION_REPO_URL="$COMMON_NOTION_URL"

project_info_fixture_root="$TMP/notion-fixture-project-info-versioned"
project_info_fixture="$project_info_fixture_root/.claude/skills/bootstrap-repo/scripts/bootstrap-repo.sh"
mkdir -p "$(dirname "$project_info_fixture")" "$project_info_fixture_root/ops/runner"
cp "$SCRIPT" "$project_info_fixture"
cp "$README" "$project_info_fixture_root/README.md"
cp "$AGENTS" "$project_info_fixture_root/AGENTS.md"
cp "$RUNNER_CONTEXT" "$project_info_fixture_root/ops/runner/repo-context.sh"
printf '\nProject Info: %s\n' "$NOTION_PROJECT_INFO_URL_VALUE" \
  >> "$project_info_fixture_root/README.md"
expect_script_rejected_without_mutation 'Project Info URL versioned in README' \
  "$project_info_fixture"

reset_logs
preflight_output="$TMP/preflight-only.out"
if ! env \
    -u REPO_PURPOSE -u NOTION_REPO_URL -u NOTION_PROJECT_INFO_URL \
    -u DISCORD_NOTIFICATIONS -u DISCORD_MENTIONS \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS= \
    "$SCRIPT" --preflight-only "$TARGET" > "$preflight_output" 2>&1; then
  cat "$preflight_output" >&2
  fail 'valid preflight-only target failed without bootstrap inputs'
fi
if [[ -s "$GH_MUTATION_LOG" ]]; then
  fail 'preflight-only mode performed mutation-capable gh calls'
fi
if grep -Eq 'graphql|project (list|copy|link)' "$GH_CALL_LOG"; then
  fail 'preflight-only mode went beyond target repository provenance'
fi
grep -q 'preflight.*완료' "$preflight_output" \
  || fail 'preflight-only success message missing'

readme_fixture=$(make_missing_notion_fixture readme README.md)
expect_script_rejected_without_mutation 'README missing Notion URL' "$readme_fixture"
agents_fixture=$(make_missing_notion_fixture agents AGENTS.md)
expect_script_rejected_without_mutation 'AGENTS missing Notion URL' "$agents_fixture"
runner_fixture=$(make_missing_notion_fixture runner ops/runner/repo-context.sh)
expect_script_rejected_without_mutation 'runner context missing Notion URL' "$runner_fixture"
expect_rejected_without_mutation 'partial common-prefix Notion URL' \
  NOTION_REPO_URL='https://app.notion.com/p'
expect_rejected_without_mutation 'multiline Notion URL with extra text' \
  NOTION_REPO_URL="$NOTION_REPO_URL_VALUE"$'\nextra text'

expect_rejected_without_mutation 'public target' FAKE_VISIBILITY=public
expect_rejected_without_mutation 'internal target' FAKE_VISIBILITY=internal
expect_rejected_without_mutation 'non-admin caller' FAKE_ADMIN=false
expect_rejected_without_mutation 'wrong default branch' FAKE_DEFAULT_BRANCH=main
expect_rejected_without_mutation 'missing template provenance' \
  FAKE_TEMPLATE_REPOSITORY=
expect_rejected_without_mutation 'wrong template provenance' \
  FAKE_TEMPLATE_REPOSITORY=some-owner/some-template
expect_rejected_without_mutation 'closed Project #14' \
  FAKE_TEMPLATE_PROJECT_CLOSED=true
expect_rejected_without_mutation 'non-template Project #14' \
  FAKE_TEMPLATE_PROJECT_TEMPLATE=false
expect_rejected_without_mutation 'wrong-title Project #14' \
  FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] some-other-project'
expect_rejected_without_mutation 'wrong template Status options' \
  FAKE_TEMPLATE_STATUS_OPTIONS=$'Backlog\nReady\nIn progress\nDone'
expect_rejected_without_mutation 'missing required template workflow' \
  FAKE_TEMPLATE_WORKFLOWS="$TARGET_WORKFLOWS_MISSING_MERGED"
expect_rejected_without_mutation 'extra template workflow' \
  FAKE_TEMPLATE_WORKFLOWS="$TARGET_WORKFLOWS_EXTRA"
expect_rejected_without_mutation 'closed existing target Project' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_PROJECT_CLOSED=true
expect_rejected_without_mutation 'wrong target Status options' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_STATUS_OPTIONS=$'Backlog\nReady\nIn progress\nIn review\nDone\nBlocked'
expect_rejected_without_mutation 'missing required target workflow' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_WORKFLOWS="$TARGET_WORKFLOWS_MISSING_MERGED"
expect_rejected_without_mutation 'extra target workflow' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_WORKFLOWS="$TARGET_WORKFLOWS_EXTRA"
expect_rejected_without_mutation 'disabled required target workflow' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_WORKFLOWS="$TARGET_WORKFLOWS_DISABLED_CLOSED"
expect_rejected_without_mutation 'paginated target workflows' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
  FAKE_TARGET_WORKFLOW_HAS_NEXT=true

reset_logs
valid_output="$TMP/valid.out"
project_secret='project-secret-value'
discord_secret='discord-webhook-value'
discord_map='{"member":"123"}'
if ! env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS= FAKE_COPIED_PROJECT_NUMBER=99 \
    FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=0 \
    FAKE_MAIN_EXISTS=0 FAKE_FAIL_WRITES=0 \
    CODEX_REVIEW=enabled \
    DISCORD_NOTIFICATIONS=enabled \
    DISCORD_MENTIONS=enabled \
    PROJECT_PAT="$project_secret" \
    DISCORD_WEBHOOK="$discord_secret" \
    DISCORD_USER_MAP="$discord_map" \
    "$SCRIPT" "$TARGET" > "$valid_output" 2>&1; then
  cat "$valid_output" >&2
  fail 'valid private template bootstrap failed'
else
  printf 'ok - valid private template bootstrap\n'
fi

grep -Fq 'README (새 저장소 만들기)' "$valid_output" \
  || fail 'bootstrap output does not point to the current README heading'
grep -Fq '=== bootstrap 이후 확인 ===' "$valid_output" \
  || fail 'bootstrap completion heading is stale'
grep -Fq 'self-hosted 자동화 runner access와 배포' "$valid_output" \
  || fail 'enabled runner completion checklist missing'
if rg -q 'repo wiki 아래에.*hub를 (생성|복제)|repo secrets 설정:' "$valid_output"; then
  fail 'bootstrap output repeats work that the bootstrap already completed'
fi

if grep -Fq 'contents/' "$GH_CALL_LOG"; then
  fail 'marker files were still used as the provenance oracle'
fi
grep -q 'git/refs.*POST.*sha=fake-dev-sha' "$GH_MUTATION_LOG" \
  || fail 'main was not created from the dev SHA'
grep -q 'PATCH.*default_branch=dev.*has_wiki=false' "$GH_MUTATION_LOG" \
  || fail 'private repository settings mutation is incomplete'
if grep -Eq 'allow_(merge_commit|rebase_merge)=false' "$GH_MUTATION_LOG"; then
  fail 'bootstrap disabled a merge method while enabling squash'
fi

for secret in "$project_secret" "$discord_secret"; do
  if grep -Fq "$secret" "$GH_CALL_LOG" \
      || grep -Fq "$secret" "$GH_MUTATION_LOG" \
      || grep -Fq "$secret" "$valid_output"; then
    fail 'secret value leaked through argv or output'
  fi
done
grep -Fq $'PROJECT_PAT\tproject-secret-value' "$GH_SECRET_STDIN_LOG" \
  || fail 'PROJECT_PAT was not delivered through stdin'
grep -Fq $'DISCORD_WEBHOOK\tdiscord-webhook-value' "$GH_SECRET_STDIN_LOG" \
  || fail 'DISCORD_WEBHOOK was not delivered through stdin'
grep -q 'variable set DISCORD_USER_MAP.*--body' "$GH_MUTATION_LOG" \
  || fail 'DISCORD_USER_MAP was not written as a repo variable'
grep -q 'variable set ENABLE_CODEX_REVIEW.*--body true' "$GH_MUTATION_LOG" || fail 'Codex review ON not recorded'
grep -q 'variable set ENABLE_DISCORD_NOTIFICATIONS.*--body true' \
  "$GH_MUTATION_LOG" \
  || fail 'enabled notifications did not set ENABLE_DISCORD_NOTIFICATIONS=true'
grep -q 'variable set ENABLE_DISCORD_MENTIONS.*--body true' \
  "$GH_MUTATION_LOG" \
  || fail 'enabled mentions did not set ENABLE_DISCORD_MENTIONS=true'
grep -q 'project copy 14 .*--source-owner skku-heven .*--target-owner skku-heven .*--title example\\ Roadmap' \
  "$GH_MUTATION_LOG" || fail 'Project #14 was not copied with the exact target title'
grep -q 'project link 99 .*--owner skku-heven .*--repo example' \
  "$GH_MUTATION_LOG" || fail 'copied Project was not linked to the target repository'

reset_logs
copied_drift_output="$TMP/copied-project-drift.out"
if env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS= FAKE_COPIED_PROJECT_NUMBER=99 \
    FAKE_COPIED_STATUS_OPTIONS=$'Backlog\nReady\nIn progress\nDone' \
    FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=0 \
    FAKE_MAIN_EXISTS=1 FAKE_FAIL_WRITES=0 \
    PROJECT_PAT='runner-project-secret' DISCORD_WEBHOOK= DISCORD_USER_MAP= \
    "$SCRIPT" "$TARGET" > "$copied_drift_output" 2>&1; then
  fail 'copied Project schema drift unexpectedly returned success'
fi
grep -q 'project copy 14 ' "$GH_MUTATION_LOG" \
  || fail 'copied Project drift case did not reach project copy'
if grep -q 'project link 99 ' "$GH_MUTATION_LOG"; then
  fail 'copied Project was linked before schema readback passed'
fi

reset_logs
existing_output="$TMP/existing-project.out"
existing_projects=$'41\texample Roadmap Extra\n42\texample Roadmap\n'
if ! env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS="$existing_projects" \
    FAKE_TARGET_WORKFLOWS="$TARGET_WORKFLOWS_WITHOUT_AUTO_ADD" \
    FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=0 \
    FAKE_MAIN_EXISTS=1 FAKE_FAIL_WRITES=0 \
    PROJECT_PAT='runner-project-secret' DISCORD_WEBHOOK= DISCORD_USER_MAP= \
    "$SCRIPT" "$TARGET" > "$existing_output" 2>&1; then
  cat "$existing_output" >&2
  fail 'existing target Project was not reused'
fi
if grep -q 'project copy' "$GH_MUTATION_LOG"; then
  fail 'existing target Project triggered another copy'
fi
grep -q 'project link 42 .*--owner skku-heven .*--repo example' \
  "$GH_MUTATION_LOG" || fail 'existing target Project was not linked'
grep -q 'variable set ENABLE_DISCORD_NOTIFICATIONS.*--body false' \
  "$GH_MUTATION_LOG" \
  || fail 'disabled notifications did not set ENABLE_DISCORD_NOTIFICATIONS=false'
grep -q 'variable set ENABLE_DISCORD_MENTIONS.*--body false' \
  "$GH_MUTATION_LOG" \
  || fail 'disabled mentions did not set ENABLE_DISCORD_MENTIONS=false'
grep -q 'variable set ENABLE_CODEX_REVIEW.*--body false' "$GH_MUTATION_LOG" || fail 'Codex review OFF not recorded'
grep -q 'secret set PROJECT_PAT' "$GH_MUTATION_LOG" || fail 'status token missing'

expect_rejected_without_mutation 'ambiguous target Projects' \
  FAKE_MAIN_EXISTS=1 \
  FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n43\texample Roadmap\n'

reset_logs
copy_failure_output="$TMP/project-copy-failure.out"
copy_failure_secret='copy-failure-secret'
if env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS= FAKE_FAIL_PROJECT_COPY=1 FAKE_FAIL_PROJECT_LINK=0 \
    FAKE_MAIN_EXISTS=1 FAKE_FAIL_WRITES=0 \
    PROJECT_PAT="$copy_failure_secret" DISCORD_WEBHOOK= DISCORD_USER_MAP= \
    "$SCRIPT" "$TARGET" > "$copy_failure_output" 2>&1; then
  fail 'Project copy failure unexpectedly returned success'
fi
grep -Fq 'gh: copy requires read:org' "$copy_failure_output" || fail 'Project copy error was hidden'
if grep -Fq "$copy_failure_secret" "$GH_CALL_LOG" \
    || grep -Fq "$copy_failure_secret" "$GH_MUTATION_LOG" \
    || grep -Fq "$copy_failure_secret" "$copy_failure_output"; then
  fail 'Project copy failure leaked a secret value'
fi
grep -q 'project copy 14 ' "$GH_MUTATION_LOG" \
  || fail 'Project copy failure case did not reach project copy'

reset_logs
link_failure_output="$TMP/project-link-failure.out"
link_failure_secret='link-failure-secret'
if env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS= FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=1 \
    FAKE_COPIED_PROJECT_NUMBER=99 FAKE_MAIN_EXISTS=1 FAKE_FAIL_WRITES=0 \
    PROJECT_PAT="$link_failure_secret" DISCORD_WEBHOOK= DISCORD_USER_MAP= \
    "$SCRIPT" "$TARGET" > "$link_failure_output" 2>&1; then
  fail 'Project link failure unexpectedly returned success'
fi
grep -Fq 'gh: repository is not accessible for linking' "$link_failure_output" || fail 'Project link error was hidden'
if grep -Fq "$link_failure_secret" "$GH_CALL_LOG" \
    || grep -Fq "$link_failure_secret" "$GH_MUTATION_LOG" \
    || grep -Fq "$link_failure_secret" "$link_failure_output"; then
  fail 'Project link failure leaked a secret value'
fi
grep -q 'project link 99 ' "$GH_MUTATION_LOG" \
  || fail 'Project link failure case did not reach project link'

reset_logs
failed_write_output="$TMP/failed-writes.out"
if env \
    "${DISABLED_CONTEXT_ENV[@]}" \
    FAKE_VISIBILITY=private FAKE_DEFAULT_BRANCH=dev FAKE_ADMIN=true \
    FAKE_TEMPLATE_REPOSITORY=skku-heven/heven-common \
    FAKE_TEMPLATE_PROJECT_CLOSED=false FAKE_TEMPLATE_PROJECT_TEMPLATE=true \
    FAKE_TEMPLATE_PROJECT_TITLE='[TEMPLATE] heven-common' \
    FAKE_EXISTING_PROJECTS=$'42\texample Roadmap\n' \
    FAKE_FAIL_PROJECT_COPY=0 FAKE_FAIL_PROJECT_LINK=0 \
    FAKE_MAIN_EXISTS=1 FAKE_FAIL_WRITES=1 \
    DISCORD_NOTIFICATIONS=enabled \
    DISCORD_MENTIONS=enabled \
    PROJECT_PAT='failed-project-secret' \
    DISCORD_WEBHOOK='failed-discord-secret' \
    DISCORD_USER_MAP='{"failed":"map"}' \
    "$SCRIPT" "$TARGET" > "$failed_write_output" 2>&1; then
  fail 'supplied write failures unexpectedly returned success'
fi
grep -q 'PROJECT_PAT.*실패' "$failed_write_output" \
  || fail 'PROJECT_PAT write failure warning missing'
grep -q 'DISCORD_WEBHOOK.*실패' "$failed_write_output" \
  || fail 'DISCORD_WEBHOOK write failure warning missing'
grep -q 'DISCORD_USER_MAP.*실패' "$failed_write_output" \
  || fail 'DISCORD_USER_MAP write failure warning missing'
for secret in failed-project-secret failed-discord-secret; do
  if grep -Fq "$secret" "$GH_CALL_LOG" \
      || grep -Fq "$secret" "$GH_MUTATION_LOG" \
      || grep -Fq "$secret" "$failed_write_output"; then
    fail 'failed secret value leaked through argv or output'
  fi
done

grep -Fq "vars.ENABLE_DISCORD_NOTIFICATIONS == 'true'" "$NOTIFY_WORKFLOW" \
  || fail 'Discord workflow is not gated by notification mode'
grep -Fq 'ENABLE_DISCORD_MENTIONS' "$NOTIFY_WORKFLOW" \
  || fail 'Discord workflow does not gate mention expansion by mode'
if rg -n 'marker' "$SKILL"; then
  fail 'skill still describes marker files as provenance'
fi
if rg -n 'ProjectV2를 생성|새 Project.*생성' "$SKILL"; then
  fail 'skill still tells operators to create a Project manually'
fi

if rg -n 'PROJECT_ID|STATUS_FIELD_ID|STATUS_OPT_' "$SCRIPT" "$SKILL" "$README"; then
  fail 'obsolete runtime-derived Project IDs remain in bootstrap guidance'
fi

# Notion opt-out must work with an untouched template and no Notion URLs.
reset_logs
if ! env "${DISABLED_CONTEXT_ENV[@]}" NOTION_MODE=disabled \
    NOTION_REPO_URL= NOTION_PROJECT_INFO_URL= \
    "$SOURCE_SCRIPT" "$TARGET" > "$TMP/no-notion.out" 2>&1; then
  cat "$TMP/no-notion.out" >&2
  fail 'Notion disabled bootstrap rejected template without hub fields'
fi
grep -q 'variable set ENABLE_DISCORD_NOTIFICATIONS.*--body false' "$GH_MUTATION_LOG" \
  || fail 'Notion disabled did not complete GitHub configuration'
if grep -q 'secret set DISCORD_WEBHOOK' "$GH_MUTATION_LOG"; then
  fail 'disabled Discord wrote a webhook'
fi
grep -q 'secret set PROJECT_PAT' "$GH_MUTATION_LOG" || fail 'required Project token was not written'
# Notion stays off unless it is selected explicitly.
reset_logs
if ! env REPO_PURPOSE="$REPO_PURPOSE_VALUE" CODEX_REVIEW=disabled \
    DISCORD_NOTIFICATIONS=disabled DISCORD_MENTIONS=disabled PROJECT_PAT='runner-project-secret' \
    "$SOURCE_SCRIPT" "$TARGET" > "$TMP/default-off.out" 2>&1; then
  cat "$TMP/default-off.out" >&2
  fail 'omitted NOTION_MODE did not default to disabled'
fi
printf 'ok - Notion defaults to disabled\n'

if grep -q 'label create' "$GH_MUTATION_LOG"; then
  fail 'bootstrap still creates planner labels'
fi
if grep -q 'RUNNER_SCRIPTS' "$GH_MUTATION_LOG" "$TMP/default-off.out"; then
  fail 'bootstrap still depends on deployed runner scripts'
fi
grep -q 'online runner 없음' "$TMP/default-off.out" \
  || fail 'bootstrap did not warn about a missing runner'
env REPO_PURPOSE="$REPO_PURPOSE_VALUE" CODEX_REVIEW=disabled FAKE_RUNNERS_ONLINE=1 \
  DISCORD_NOTIFICATIONS=disabled DISCORD_MENTIONS=disabled PROJECT_PAT='runner-project-secret' \
  "$SOURCE_SCRIPT" "$TARGET" > "$TMP/runner-online.out" 2>&1 \
  || fail 'bootstrap with an online runner failed'
if grep -q 'online runner 없음' "$TMP/runner-online.out"; then
  fail 'bootstrap warned although a runner is online'
fi
printf 'ok - bootstrap reports runner availability without planner labels\n'

expect_rejected_without_mutation 'invalid Notion mode' NOTION_MODE=unknown
expect_rejected_without_mutation 'disabled Notion with stale URL' NOTION_MODE=disabled
expect_rejected_without_mutation 'disabled Notion still requires private repo' \
  NOTION_MODE=disabled NOTION_REPO_URL= NOTION_PROJECT_INFO_URL= FAKE_VISIBILITY=public
expect_rejected_without_mutation 'disabled Notion still requires template provenance' \
  NOTION_MODE=disabled NOTION_REPO_URL= NOTION_PROJECT_INFO_URL= FAKE_TEMPLATE_REPOSITORY=other/repo

if ((failures > 0)); then
  printf 'FAILED: %d bootstrap contract expectation(s)\n' "$failures" >&2
  exit 1
fi

printf 'ok - all bootstrap preflight and write-safety regressions\n'
