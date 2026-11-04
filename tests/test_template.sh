#!/usr/bin/env bash
# Structural smoke check; script behavior is covered by each component's tests.
set -Eeuo pipefail
cd "$(dirname "$0")/.."
for file in README.md AGENTS.md CLAUDE.md Makefile heven.toml tools/requirements.txt docs/agents/issue-tracker.md; do
  [[ -f "$file" ]] || { echo "Missing: $file" >&2; exit 1; }
done
for skill in bootstrap-repo idea-to-requirements requirements-to-issues issue-worker grilling grill-me domain-modeling to-tickets handoff teach process-improvement; do
  [[ -f ".claude/skills/$skill/SKILL.md" ]] || { echo "Missing skill: $skill" >&2; exit 1; }
done
inventory=$(mktemp)
trap 'rm -f "$inventory"' EXIT
git ls-files -z > "$inventory"
while IFS= read -r -d '' file; do
  [[ -f "$file" ]] || continue
  IFS= read -r first_line < "$file" || true
  if [[ "$file" == *.sh || "$first_line" =~ ^\#!.*(bash|/sh)([[:space:]]|$) ]]; then
    bash -n "$file"
  fi
done < "$inventory"
# The template stays general: project and machine names belong in derived repositories.
if git grep -n -I -i -E 'hackathon|해커톤|morai|ioniq|heven-right|heven_common_test|시뮬레이터' -- . \
    ':!THIRD_PARTY_NOTICES.md' ':!tests/test_template.sh'; then
  echo 'Project-specific text found in the template' >&2
  exit 1
fi
echo 'template structure and shell syntax: ok'
