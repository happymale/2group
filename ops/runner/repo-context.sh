#!/usr/bin/env bash
# Prompt template 변수. source해서 사용한다.

: "${GH_REPO:=${GITHUB_REPOSITORY:-}}"
: "${GH_REPO:?GH_REPO 또는 GITHUB_REPOSITORY 필요}"

export REPO_FULL_NAME="$GH_REPO"
# Repo-specific human documentation. Bootstrap replaces the placeholder with the
# exact read-back repo hub URL, never the Project Info child URL. Automation
# fetches the repo hub only through an available MCP.
export NOTION_GUIDE_URL="${NOTION_GUIDE_URL:-<NOTION_REPO_URL>}"
export OPEN_MILESTONES="[]"

if command -v gh >/dev/null 2>&1; then
  OPEN_MILESTONES=$(gh api "repos/${REPO_FULL_NAME}/milestones?state=open" \
    --jq '[.[] | {number, title, description, due_on}]' 2>/dev/null || echo "[]")
  export OPEN_MILESTONES
else
  export OPEN_MILESTONES="[]"
fi
