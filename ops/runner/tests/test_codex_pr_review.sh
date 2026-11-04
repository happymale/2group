#!/usr/bin/env bash
# The review agent reads untrusted PR content, so it must not receive GitHub tokens.

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)
WORKFLOW="$ROOT/.github/workflows/codex-pr-review.yml"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/home/.local/bin" "$TMP/pr"

cat > "$TMP/home/.local/bin/gh" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$GH_CALLS"
case "$1 $2" in
  'pr view')
    # gh 2.4 on the runner does not know closingIssuesReferences.
    [[ "$*" != *closingIssuesReferences* ]] || { echo 'Unknown JSON field: "closingIssuesReferences"' >&2; exit 1; }
    printf '{"title":"Example PR","body":"Resolves #1"}\n' ;;
  'api graphql') printf '1\n' ;;
  'pr diff') printf 'diff --git a/x b/x\n' ;;
  'issue view') printf '{"title":"Example issue","body":"Goal"}\n' ;;
  'api '*) printf '[]\n' ;;
esac
EOF
cat > "$TMP/home/.local/bin/codex" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "${GH_TOKEN-__UNSET__}|${GITHUB_TOKEN-__UNSET__}|${PROJECT_PAT-__UNSET__}" > "$CODEX_TOKENS"
while (($#)); do
  case "$1" in
    -C) printf '%s\n' "$2" > "$CODEX_CHECKOUT"; shift 2 ;;
    --output-last-message) printf 'review body\n' > "$2"; shift 2 ;;
    *) printf '%s\n' "$1" >> "$CODEX_ARGS"; shift ;;
  esac
done
EOF
chmod +x "$TMP/home/.local/bin/gh" "$TMP/home/.local/bin/codex"

env HOME="$TMP/home" PATH="$TMP/home/.local/bin:/usr/bin:/bin" \
  GITHUB_REPOSITORY=example/repo PR_CHECKOUT="$TMP/pr" \
  GH_TOKEN=workflow-token GITHUB_TOKEN=workflow-token PROJECT_PAT=project-token \
  GH_CALLS="$TMP/gh.log" CODEX_TOKENS="$TMP/tokens" CODEX_CHECKOUT="$TMP/checkout" CODEX_ARGS="$TMP/args" \
  "$ROOT/ops/runner/bin/codex-pr-review.sh" 17 > "$TMP/out" 2>&1 || { cat "$TMP/out" >&2; exit 1; }

[[ "$(<"$TMP/tokens")" == '__UNSET__|__UNSET__|__UNSET__' ]] \
  || { echo 'FAIL: codex received a GitHub token' >&2; exit 1; }
[[ "$(<"$TMP/checkout")" == "$TMP/pr" ]] || { echo 'FAIL: codex used the wrong checkout' >&2; exit 1; }
grep -q 'pr comment 17' "$TMP/gh.log" || { echo 'FAIL: review was not posted' >&2; exit 1; }
grep -q 'read-only' "$TMP/args" || { echo 'FAIL: codex is not read-only' >&2; exit 1; }
grep -Fq 'diff --git a/x b/x' "$TMP/args" || { echo 'FAIL: prompt lacks the PR diff' >&2; exit 1; }
grep -Fq 'Example issue' "$TMP/args" || { echo 'FAIL: prompt lacks the linked issue' >&2; exit 1; }

types=$(grep -E '^[[:space:]]+types:' "$WORKFLOW")
if grep -q synchronize <<< "$types"; then
  echo 'FAIL: codex review reruns on every push' >&2
  exit 1
fi
if grep -q RUNNER_SCRIPTS "$WORKFLOW"; then
  echo 'FAIL: codex review still depends on a manually deployed script' >&2
  exit 1
fi
printf 'ok - codex review runs without GitHub tokens on selected events\n'
