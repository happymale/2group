#!/usr/bin/env bash
# prepare-derived.sh records bootstrap choices and removes template-only files.

set -Eeuo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd -P)
SCRIPT="$ROOT/.claude/skills/bootstrap-repo/scripts/prepare-derived.sh"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/bin"
cat > "$TMP/bin/make" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$MAKE_LOG"
EOF
chmod +x "$TMP/bin/make"

fail() { printf 'not ok - %s\n' "$1" >&2; exit 1; }

fixture() {
  local repo="$TMP/$1"
  mkdir -p "$repo"
  git -C "$repo" init -q
  git -C "$repo" config user.name test
  git -C "$repo" config user.email test@example.com
  git -C "$repo" remote add origin "https://github.com/skku-heven/${2:-derived}.git"
  mkdir -p "$repo/tests" "$repo/ops/runner/tests" "$repo/.github/workflows" \
    "$repo/.claude/skills/bootstrap-repo/tests" "$repo/.claude/skills/issue-worker/tests"
  cp "$ROOT/heven.toml" "$repo/heven.toml"
  for file in tests/test_template.sh tests/test_requirements.py ops/runner/tests/a.sh \
      .github/workflows/ci.yml .github/workflows/keep.yml \
      .claude/skills/bootstrap-repo/tests/a.sh .claude/skills/issue-worker/tests/a.sh \
      tests/project_test.py; do
    printf 'x\n' > "$repo/$file"
  done
  git -C "$repo" add -A
  git -C "$repo" commit -qm init
  printf '%s\n' "$repo"
}

run() {
  local repo="$1"; shift
  (cd "$repo" && env PATH="$TMP/bin:$PATH" MAKE_LOG="$repo/make.log" "$@" "$SCRIPT")
}

repo=$(fixture off)
run "$repo" TEMPLATE_COMMIT=abc1234 STRICTDOC=disabled REQUIREMENT_LEVELS= > "$TMP/off.out" 2>&1 \
  || { cat "$TMP/off.out" >&2; fail 'OFF preparation failed'; }
grep -qx 'commit = "abc1234"' "$repo/heven.toml" || fail 'template commit not recorded'
grep -qx 'enabled = false' "$repo/heven.toml" || fail 'StrictDoc OFF not recorded'
grep -qx 'levels = \[\]' "$repo/heven.toml" || fail 'OFF kept requirement levels'
[[ ! -e "$repo/make.log" ]] || fail 'OFF ran requirements init'
for removed in tests/test_template.sh tests/test_requirements.py ops/runner/tests/a.sh \
    .github/workflows/ci.yml .claude/skills/bootstrap-repo/tests/a.sh .claude/skills/issue-worker/tests/a.sh; do
  [[ ! -e "$repo/$removed" ]] || fail "template-only file kept: $removed"
done
[[ -e "$repo/tests/project_test.py" ]] || fail 'project test was removed'
[[ -e "$repo/.github/workflows/keep.yml" ]] || fail 'other workflow was removed'
printf 'ok - OFF records choices and removes template-only files\n'

run "$repo" TEMPLATE_COMMIT=abc1234 STRICTDOC=disabled REQUIREMENT_LEVELS= > "$TMP/rerun.out" 2>&1 \
  || fail 'rerun failed after files were already removed'
printf 'ok - preparation can rerun\n'

repo=$(fixture on)
run "$repo" TEMPLATE_COMMIT=def5678 STRICTDOC=enabled REQUIREMENT_LEVELS=stakeholder,software,hardware \
  > "$TMP/on.out" 2>&1 || { cat "$TMP/on.out" >&2; fail 'ON preparation failed'; }
grep -qx 'enabled = true' "$repo/heven.toml" || fail 'StrictDoc ON not recorded'
grep -qx 'levels = \["stakeholder", "software", "hardware"\]' "$repo/heven.toml" || fail 'levels not recorded'
grep -qx 'requirements-init' "$repo/make.log" || fail 'ON did not initialize requirement documents'
printf 'ok - ON records selected levels and initializes documents\n'

for bad in 'STRICTDOC=enabled REQUIREMENT_LEVELS=' 'STRICTDOC=enabled REQUIREMENT_LEVELS=software,unknown' \
    'STRICTDOC=maybe REQUIREMENT_LEVELS=' 'STRICTDOC=disabled REQUIREMENT_LEVELS=software'; do
  repo=$(fixture "bad-$RANDOM")
  before=$(cat "$repo/heven.toml")
  # shellcheck disable=SC2086
  if run "$repo" TEMPLATE_COMMIT=abc1234 $bad > "$TMP/bad.out" 2>&1; then
    fail "invalid choice accepted: $bad"
  fi
  [[ "$before" == "$(cat "$repo/heven.toml")" ]] || fail "invalid choice changed heven.toml: $bad"
  [[ -e "$repo/tests/test_template.sh" ]] || fail "invalid choice removed files: $bad"
done
printf 'ok - invalid choices change nothing\n'

repo=$(fixture template heven-common)
if run "$repo" TEMPLATE_COMMIT=abc1234 STRICTDOC=disabled REQUIREMENT_LEVELS= > "$TMP/self.out" 2>&1; then
  fail 'preparation ran on heven-common itself'
fi
[[ -e "$repo/tests/test_template.sh" ]] || fail 'preparation changed heven-common itself'
printf 'ok - preparation refuses the template repository\n'

printf 'ok - all derived preparation regressions\n'
