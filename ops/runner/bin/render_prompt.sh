#!/usr/bin/env bash
# Render a prompt template with repo-context variables.
# Usage: render_prompt.sh <template> <context>

set -Eeuo pipefail

TEMPLATE="${1:?template path required}"
CONTEXT="${2:?context path required}"

[[ -f "$TEMPLATE" ]] || { echo "template not found: $TEMPLATE" >&2; exit 1; }
[[ -f "$CONTEXT" ]]  || { echo "context not found: $CONTEXT" >&2; exit 1; }

# shellcheck disable=SC1090
source "$CONTEXT"

envsubst '
${REPO_FULL_NAME}
${PR_NUMBER}
${NOTION_GUIDE_URL}
${OPEN_MILESTONES}
' < "$TEMPLATE"
