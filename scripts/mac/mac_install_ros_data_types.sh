#!/usr/bin/env bash
# Purpose (macOS/Homebrew):
#   1) Ensure SWIG >= 4.1 is available via Homebrew.
#   2) Generate & build Fast DDS Python message bindings (delegates to gen_python_types.sh).
#   3) Install generated Python packages into /opt/fast-dds-v3-libs/python/src (delegates to install_python_types.sh).
#   4) Collect all generated lib*.so / lib*.dylib into /opt/fast-dds-v3-libs/lib (idempotent).
#   5) Export env vars for current shell (DYLD_LIBRARY_PATH / PYTHONPATH) and persist to ~/.zshrc.
set -euo pipefail

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
# ソフトに 4.1 系を推奨。4.0 や 4.2 でも通る構成が多いが、必要ならここでバージョンを厳密チェック。
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
bash mac_gen_python_types.sh

# --- 3) install generated packages into target prefix ---
current_dir="$(pwd)"
echo "[DBG] Current dir: ${current_dir}"

BUILD_ROOT="${current_dir}/../._types_python_build_v3"
echo "[DBG] BUILD_ROOT: ${BUILD_ROOT}"

sudo INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \
     BUILD_ROOT="${BUILD_ROOT}" \
     bash mac_install_python_types.sh

# --- 3.5) collect all lib*.so / lib*.dylib to a single location (idempotent) ---
TARGET_LIB_DIR="/opt/fast-dds-v3-libs/lib"
echo "[INFO] Collecting generated shared libs to ${TARGET_LIB_DIR} …"
sudo mkdir -p "${TARGET_LIB_DIR}"
# broken symlinks cleanup
sudo find "${TARGET_LIB_DIR}" -type l ! -exec test -e {} \; -delete || true

# BSD find 互換で .so / .dylib の両方を集約
# (Python拡張は .so、C++共有ライブラリは .dylib のことが多い)
find /opt/fast-dds-v3-libs/python/src -type f \( -name 'lib*.so' -o -name 'lib*.dylib' \) -print0 \
  | while IFS= read -r -d '' sofile; do
      bn="$(basename "$sofile")"
      dst="${TARGET_LIB_DIR}/${bn}"
      if [[ ! -e "${dst}" ]]; then
        sudo ln -s "$sofile" "${dst}"
      fi
    done

# --- 4) export runtime paths for this shell session (macOS uses DYLD_LIBRARY_PATH) ---
# 集約ディレクトリ + 各生成libの所在ディレクトリを追加
echo "[INFO] Exporting environment variables for current shell…"
# 生成された lib*.so/.dylib を含むディレクトリ一覧を組み立て（BSD find 互換）
ADD_DYLD_DIRS=""
# ユニーク化のため一旦一時ファイルへ
_tmp_dirs="$(mktemp)"
find /opt/fast-dds-v3-libs/python/src -type f \( -name 'lib*.so' -o -name 'lib*.dylib' \) -print0 \
  | while IFS= read -r -d '' f; do
      dirname "$f"
    done | sort -u > "${_tmp_dirs}"
if [[ -s "${_tmp_dirs}" ]]; then
  # 改行を : に変換
  ADD_DYLD_DIRS="$(tr '\n' ':' < "${_tmp_dirs}")"
  # 末尾の : を落とす
  ADD_DYLD_DIRS="${ADD_DYLD_DIRS%:}"
fi
rm -f "${_tmp_dirs}"

export DYLD_LIBRARY_PATH="${TARGET_LIB_DIR}${ADD_DYLD_DIRS:+:${ADD_DYLD_DIRS}}:${DYLD_LIBRARY_PATH:-}"
export PYTHONPATH="/opt/fast-dds-v3-libs/python/src:${PYTHONPATH:-}"

# Fast-DDS Python (merge-install 側) があれば追加
PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' ':')"
if [[ -n "${PY_SITE_PACK}" ]]; then
  export PYTHONPATH="${PY_SITE_PACK}:${PYTHONPATH}"
fi

# --- 4.5) sanity check import path resolution ---
python3 - <<'PY'
import sys, os
print("[DBG] PYTHONPATH=", os.environ.get("PYTHONPATH",""))
print("[DBG] DYLD_LIBRARY_PATH=", os.environ.get("DYLD_LIBRARY_PATH",""))
try:
    from std_msgs.msg import String
    s = String()
    print("OK:", type(s))
except Exception as e:
    print("IMPORT-ERROR:", e)
    raise
PY

# --- 5) persist environment variables in ~/.zshrc (idempotent on macOS) ---
ZSHRC="$HOME/.zshrc"
touch "$ZSHRC"

add_line_once() {
  local line="$1"
  local file="$2"
  # そのままの行が既にあれば追記しない
  grep -qxF "$line" "$file" || echo "$line" >> "$file"
}

echo "[INFO] Persisting environment to ${ZSHRC} …"
# (a) Generated Python packages
add_line_once 'export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH' "$ZSHRC"

# (b) Fast-DDS Python site-packages (if available at runtime)
add_line_once 'export PYTHONPATH="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr '\'' '\'' :):$PYTHONPATH"' "$ZSHRC"

# (c) Shared libraries: central dir + (fallback) all lib dirs (BSD find 互換版)
add_line_once 'export DYLD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:$DYLD_LIBRARY_PATH' "$ZSHRC"
add_line_once 'export DYLD_LIBRARY_PATH="$(find /opt/fast-dds-v3-libs/python/src -type f \( -name '\''lib*.so'\'' -o -name '\''lib*.dylib'\'' \) -print0 | xargs -0 -I{} dirname {} | sort -u | paste -sd: -):$DYLD_LIBRARY_PATH"' "$ZSHRC"

cat <<'NOTE'
[NOTE]
- macOS では GUI アプリからは SIP の影響で DYLD_LIBRARY_PATH が無視されます（Terminal 上は有効）。
- このスクリプトは Homebrew の swig を使います。必要なら `brew unlink swig@X && brew link swig@Y` で切替えてください。
- 反映を永続化したので、新しいシェルで有効になります。すぐ反映したい場合は:
    source ~/.zshrc
NOTE

echo "[DONE] All set."
