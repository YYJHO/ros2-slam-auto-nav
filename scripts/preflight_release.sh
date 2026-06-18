#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

failures=0
warnings=0

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  warnings=$((warnings + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  failures=$((failures + 1))
  printf '[FAIL] %s\n' "$1"
}

run_check() {
  local label="$1"
  shift
  if "$@"; then
    pass "$label"
  else
    fail "$label"
  fi
}

printf 'Virtual Indoor Navigation release preflight\n'
printf 'Project: %s\n\n' "${PROJECT_ROOT}"

cd "${PROJECT_ROOT}"

required_files=(
  "README.md"
  "INSTALL.md"
  "LICENSE"
  "SECURITY.md"
  "CONTRIBUTING.md"
  ".gitignore"
  "workspace/src/virtual_indoor_nav/package.xml"
  "workspace/src/virtual_indoor_nav/setup.py"
)

for file in "${required_files[@]}"; do
  if [ -f "${file}" ]; then
    pass "required file exists: ${file}"
  else
    fail "required file missing: ${file}"
  fi
done

printf '\nShell syntax\n'
while IFS= read -r script; do
  run_check "bash syntax: ${script}" bash -n "${script}"
done < <(find scripts -maxdepth 1 -type f -name '*.sh' | sort)

printf '\nPython syntax\n'
if command -v python3 >/dev/null 2>&1; then
  while IFS= read -r pyfile; do
    run_check "python compile: ${pyfile}" python3 -m py_compile "${pyfile}"
  done < <(find workspace/src/virtual_indoor_nav -type f -name '*.py' | sort)
else
  fail "python3 command is not available"
fi

printf '\nPackage metadata\n'
if command -v python3 >/dev/null 2>&1; then
  run_check "package.xml parses as XML" python3 -c 'import xml.etree.ElementTree as ET; ET.parse("workspace/src/virtual_indoor_nav/package.xml")'
fi

if grep -q '<license>MIT</license>' workspace/src/virtual_indoor_nav/package.xml \
  && grep -q 'license="MIT"' workspace/src/virtual_indoor_nav/setup.py; then
  pass "package license metadata is MIT"
else
  fail "package license metadata is not consistently MIT"
fi

if grep -q 'dome@localhost' workspace/src/virtual_indoor_nav/package.xml workspace/src/virtual_indoor_nav/setup.py; then
  warn "maintainer email still uses dome@localhost; replace it before public release"
else
  pass "maintainer email does not use localhost"
fi

if grep -q '你的用户名' README.md; then
  warn "README clone URL still contains placeholder username"
else
  pass "README clone URL does not contain the Chinese username placeholder"
fi

if grep -q '<your-user>' README.md docs/GITHUB_RELEASE.md; then
  warn "repository URL still contains <your-user> placeholder"
else
  pass "repository URL placeholder has been replaced"
fi

printf '\nIgnore rules\n'
ignored_paths=(
  "workspace/build/test"
  "workspace/install/test"
  "workspace/log/test"
  "runtime/maps/generated_map.yaml"
  "runtime/maps/generated_map.pgm"
  "runtime/rooms.yaml"
  "runtime/diagnostics/test.log"
)

if command -v git >/dev/null 2>&1 && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  for path in "${ignored_paths[@]}"; do
    if git check-ignore -q "${path}"; then
      pass "ignored by git: ${path}"
    else
      fail "not ignored by git: ${path}"
    fi
  done
else
  warn "not inside a valid Git repository; skipped git check-ignore tests"
fi

printf '\nSecret scan\n'
if command -v rg >/dev/null 2>&1; then
  if rg -n --hidden -S '(password|passwd|secret|token|api[_-]?key|access[_-]?key|private[_-]?key|BEGIN (RSA|OPENSSH|PRIVATE)|AKIA|ghp_|github_pat_|AIza|sk-[A-Za-z0-9])' \
    -g '!.git/**' \
    -g '!workspace/build/**' \
    -g '!workspace/install/**' \
    -g '!workspace/log/**' \
    -g '!runtime/diagnostics/**' \
    -g '!SECURITY.md' \
    -g '!docs/GITHUB_RELEASE.md' \
    -g '!.github/ISSUE_TEMPLATE/**' \
    -g '!scripts/preflight_release.sh' \
    .; then
    fail "possible secret-like strings found; review matches above"
  else
    pass "no common secret-like strings found"
  fi
else
  warn "ripgrep is not available; skipped local secret pattern scan"
fi

printf '\nSummary: %d failure(s), %d warning(s)\n' "${failures}" "${warnings}"

if [ "${failures}" -gt 0 ]; then
  exit 1
fi
