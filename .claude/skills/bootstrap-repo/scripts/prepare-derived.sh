#!/usr/bin/env bash
#
# prepare-derived.sh
#
# 파생 저장소 clone의 root에서 실행한다. bootstrap 선택을 heven.toml에 기록하고,
# 템플릿 자체를 검증하는 파일을 제거한다. commit은 호출한 쪽에서 한다.
#
# env:
#   TEMPLATE_COMMIT — 기준이 되는 heven-common 커밋
#   STRICTDOC — enabled 또는 disabled
#   REQUIREMENT_LEVELS — enabled일 때 필수. 쉼표로 구분
#     (business, stakeholder, system, software, hardware)

set -Eeuo pipefail

die() { echo "✗ $*" >&2; exit 1; }

: "${TEMPLATE_COMMIT:?TEMPLATE_COMMIT 필요}"
: "${STRICTDOC:?STRICTDOC 필요 (enabled 또는 disabled)}"
REQUIREMENT_LEVELS="${REQUIREMENT_LEVELS:-}"

ROOT=$(git rev-parse --show-toplevel)
cd "$ROOT"
[[ -f heven.toml ]] || die "heven.toml 없음"
if git remote get-url origin 2>/dev/null | grep -Eq '/heven-common(\.git)?$'; then
  die "heven-common 자체에는 실행하지 않음"
fi
[[ "$TEMPLATE_COMMIT" =~ ^[0-9a-f]{7,40}$ ]] || die "TEMPLATE_COMMIT 형식 오류"

case "$STRICTDOC" in
  disabled)
    [[ -z "$REQUIREMENT_LEVELS" ]] || die "STRICTDOC=disabled일 때 REQUIREMENT_LEVELS를 줄 수 없음"
    ENABLED=false
    LEVELS='[]'
    ;;
  enabled)
    [[ -n "$REQUIREMENT_LEVELS" ]] || die "STRICTDOC=enabled에는 REQUIREMENT_LEVELS가 필요"
    ENABLED=true
    LEVELS=''
    IFS=',' read -r -a selected <<< "$REQUIREMENT_LEVELS"
    for level in "${selected[@]}"; do
      case "$level" in
        business|stakeholder|system|software|hardware) ;;
        *) die "알 수 없는 요구사항 계층: $level" ;;
      esac
      LEVELS+="${LEVELS:+, }\"$level\""
    done
    LEVELS="[$LEVELS]"
    ;;
  *) die "STRICTDOC은 enabled 또는 disabled" ;;
esac

sed -i \
  -e "s/^commit = .*/commit = \"$TEMPLATE_COMMIT\"/" \
  -e "s/^enabled = .*/enabled = $ENABLED/" \
  -e "s/^levels = .*/levels = $LEVELS/" \
  heven.toml

git rm -rq --ignore-unmatch -- \
  .claude/skills/bootstrap-repo/tests \
  .claude/skills/issue-worker/tests \
  ops/runner/tests \
  tests/test_requirements.py \
  tests/test_template.sh \
  .github/workflows/ci.yml

if [[ "$ENABLED" == true ]]; then
  make requirements-init
fi

git add heven.toml
echo "✓ heven.toml 기록: template=$TEMPLATE_COMMIT, StrictDoc=$STRICTDOC, levels=$LEVELS"
