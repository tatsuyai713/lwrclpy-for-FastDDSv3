#!/usr/bin/env bash
# Purpose (macOS/Homebrew):
#   1) Ensure SWIG >= 4.1 is available via Homebrew.
#   2) Generate & build Fast DDS Python message bindings (delegates to gen_python_types.sh).
#   3) Install generated Python packages into /opt/fast-dds-v3-libs/python/src (delegates to install_python_types.sh).
#   4) Collect all generated lib*.so / lib*.dylib into /opt/fast-dds-v3-libs/lib (idempotent).
#   5) Export env vars for current shell (DYLD_LIBRARY_PATH / PYTHONPATH) and optionally persist to ~/.zshrc.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"
PERSIST_ENV="${PERSIST_ENV:-0}"   # set to 1 to append exports to ~/.zshrc
EXPORT_ENV="${EXPORT_ENV:-0}"     # set to 1 to export DYLD/PYTHONPATH for this shell (off by default to avoid SIP kills)

# --- 0) sanity (macOS only) ---
if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "[FATAL] This script is for macOS. Detected: $(uname -s)" >&2
  exit 1
fi

# --- 1) SWIG (Homebrew) ---
echo "[INFO] Installing swig via Homebrew (requires Homebrew preinstalled)…"
if ! command -v brew >/dev/null 2>&1; then
  echo "[FATAL] Homebrew not found. Install from https://brew.sh/ and retry." >&2
  exit 1
fi
brew update
brew install swig || true

echo "[INFO] Checking swig version…"
hash -r
which swig
swig -version | sed -n '1,8p'
# Recommend SWIG 4.1.x (4.0/4.2 may work, but pin if your env needs it).
SWIG_VER_RAW="$(swig -version | awk -F'[ ]' '/SWIG Version/ {print $3; exit}')"
SWIG_MAJ="${SWIG_VER_RAW%%.*}"
SWIG_MIN_PATCH="${SWIG_VER_RAW#*.}"            # e.g. "1.1"
SWIG_MIN="${SWIG_MIN_PATCH%%.*}"               # e.g. "1"
if [[ "${SWIG_MAJ}" -lt 4 ]] || { [[ "${SWIG_MAJ}" -eq 4 ]] && [[ "${SWIG_MIN}" -lt 1 ]]; }; then
  echo "[WARN] Detected SWIG ${SWIG_VER_RAW}. SWIG >= 4.1 is recommended."
  echo "      If you specifically need 4.1.x, install an older brew version or build from source."
fi

# --- 2) generate & build all message bindings ---
echo "[INFO] Running gen_python_types.sh …"
bash "${SCRIPT_DIR}/mac_gen_python_types.sh"

# --- 3) install generated packages into target prefix ---
current_dir="${ROOT_DIR}"
echo "[DBG] Current dir: ${current_dir}"

BUILD_ROOT="${ROOT_DIR}/._types_python_build_v3"
echo "[DBG] BUILD_ROOT: ${BUILD_ROOT}"

sudo INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \
     BUILD_ROOT="${BUILD_ROOT}" \
     bash "${SCRIPT_DIR}/mac_install_python_types.sh"

# --- 3.1) add ROS 2-style service aliases to generated Python packages ---
PY_ALIAS_SCRIPT="${ROOT_DIR}/third_party/ros-data-types-for-fastdds/scripts/add_service_aliases.py"
if [[ -f "${PY_ALIAS_SCRIPT}" ]]; then
  echo "[INFO] Injecting Request/Response service aliases into generated Python packages..."
  ALIAS_TARGET="/opt/fast-dds-v3-libs/python/src"
  if [[ -w "${ALIAS_TARGET}" ]]; then
    python3 "${PY_ALIAS_SCRIPT}" "${ALIAS_TARGET}" || echo "[WARN] Failed to install service aliases"
  else
    sudo python3 "${PY_ALIAS_SCRIPT}" "${ALIAS_TARGET}" || echo "[WARN] Failed to install service aliases"
  fi
fi

# --- 3.2) Ensure fastdds Python module is importable (macOS default builds emit .dylib) ---
ensure_fastdds_python_so() {
  local fastdds_root="$1"
  [[ -d "${fastdds_root}" ]] || return
  while IFS= read -r pkg_dir; do
    [[ -d "${pkg_dir}" ]] || continue
    local has_so
    if compgen -G "${pkg_dir}/_fastdds_python*.so" >/dev/null; then
      continue
    fi
    if compgen -G "${pkg_dir}/_fastdds_python*.dylib" >/dev/null; then
      while IFS= read -r dylib; do
        [[ -f "${dylib}" ]] || continue
        local so="${dylib%.dylib}.so"
        if [[ -e "${so}" ]]; then
          continue
        fi
        echo "[INFO] Copying $(basename "${dylib}") -> $(basename "${so}") for Python import"
        if [[ -w "${pkg_dir}" ]]; then
          /bin/cp -p "${dylib}" "${so}"
        else
          sudo /bin/cp -p "${dylib}" "${so}"
        fi
      done < <(find "${pkg_dir}" -maxdepth 1 -type f -name '_fastdds_python*.dylib' -print)
    fi
  done < <(find "${fastdds_root}/lib" -maxdepth 3 -type d -path "*/python*/site-packages/fastdds" -print 2>/dev/null)
}

ensure_fastdds_python_so "${PREFIX_V3}" || true

# --- 3.5) collect all lib*.so / lib*.dylib to a single location (idempotent) ---
TARGET_LIB_DIR="/opt/fast-dds-v3-libs/lib"
echo "[INFO] Collecting generated shared libs to ${TARGET_LIB_DIR} …"
sudo mkdir -p "${TARGET_LIB_DIR}"
# broken symlinks cleanup
sudo find "${TARGET_LIB_DIR}" -type l ! -exec test -e {} \; -delete || true

# Collect both .so and .dylib (Python extensions usually .so, core libs often .dylib)
find /opt/fast-dds-v3-libs/python/src -type f \( -name 'lib*.so' -o -name 'lib*.dylib' \) -print0 \
  | while IFS= read -r -d '' sofile; do
      bn="$(basename "$sofile")"
      dst="${TARGET_LIB_DIR}/${bn}"
      if [[ ! -e "${dst}" ]]; then
        sudo ln -s "$sofile" "${dst}"
      fi
    done

if [[ "${EXPORT_ENV}" -eq 1 ]]; then
  echo "[INFO] Exporting environment variables for current shell…"
  ADD_DYLD_DIRS=""
  _tmp_dirs="$(mktemp)"
  find /opt/fast-dds-v3-libs/python/src -type f \( -name 'lib*.so' -o -name 'lib*.dylib' \) -print0 \
    | while IFS= read -r -d '' f; do
        dirname "$f"
      done | sort -u > "${_tmp_dirs}"
  if [[ -s "${_tmp_dirs}" ]]; then
    ADD_DYLD_DIRS="$(tr '\n' ':' < "${_tmp_dirs}")"
    ADD_DYLD_DIRS="${ADD_DYLD_DIRS%:}"
  fi
  rm -f "${_tmp_dirs}"

  DYLD_COMBINED="${ADD_DYLD_DIRS}"
  [[ -n "${DYLD_COMBINED}" ]] && DYLD_COMBINED="${DYLD_COMBINED}:"
  DYLD_COMBINED="${DYLD_COMBINED}${TARGET_LIB_DIR}"
  if [[ -n "${DYLD_LIBRARY_PATH:-}" ]]; then
    DYLD_COMBINED="${DYLD_COMBINED}:${DYLD_LIBRARY_PATH}"
  fi
  export DYLD_LIBRARY_PATH="${DYLD_COMBINED}"
  export PYTHONPATH="/opt/fast-dds-v3-libs/python/src:${PYTHONPATH:-}"

  PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' ':')"
  if [[ -n "${PY_SITE_PACK}" ]]; then
    export PYTHONPATH="${PY_SITE_PACK}:${PYTHONPATH}"
  fi

  echo "[INFO] Skipping inline Python sanity check to avoid SIP-related kills. Run your own check after reviewing env."
else
  echo "[INFO] EXPORT_ENV=0; not exporting DYLD/PYTHONPATH (recommended on macOS)."
fi

# --- 5) persist environment variables in ~/.zshrc (idempotent on macOS) ---
ZSHRC="$HOME/.zshrc"
touch "$ZSHRC"

add_line_once() {
  local line="$1"
  local file="$2"
  grep -qxF "$line" "$file" || echo "$line" >> "$file"
}

if [[ "${PERSIST_ENV}" -eq 1 ]]; then
  echo "[INFO] Persisting environment to ${ZSHRC} …"
  add_line_once 'export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH' "$ZSHRC"
  add_line_once 'export PYTHONPATH="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr '\'' '\'' :):$PYTHONPATH"' "$ZSHRC"
  add_line_once 'export DYLD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:$DYLD_LIBRARY_PATH' "$ZSHRC"
  add_line_once 'export DYLD_LIBRARY_PATH="$(find /opt/fast-dds-v3-libs/python/src -type f \( -name '\''lib*.so'\'' -o -name '\''lib*.dylib'\'' \) -print0 | xargs -0 -I{} dirname {} | sort -u | paste -sd: -):$DYLD_LIBRARY_PATH"' "$ZSHRC"
else
  echo "[INFO] PERSIST_ENV=0; not writing to ${ZSHRC}"
fi

cat <<'NOTE'
[NOTE]
- DYLD_LIBRARY_PATH is ignored for GUI apps due to SIP (Terminal sessions honor it).
- This script relies on Homebrew swig; switch via `brew unlink swig@X && brew link swig@Y` if needed.
- Environment exports can be appended to ~/.zshrc when PERSIST_ENV=1. To apply immediately:
    source ~/.zshrc
  Or export manually for the current shell (optional, may trigger SIP kills if DYLD is set):
    export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH
    export DYLD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:$DYLD_LIBRARY_PATH
NOTE

echo "[DONE] All set."
