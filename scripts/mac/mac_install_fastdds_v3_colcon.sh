#!/usr/bin/env bash
# Install Fast DDS v3 core + fastddsgen + fastdds_python via colcon into /opt/fast-dds-v3 (macOS).
# Applies a macOS-specific Fast DDS patch to drop unsupported thread-affinity calls and
# force-uses an old standalone Asio (1.12.x) via -I include precedence,
# so legacy API like asio::io_service / address::from_string / obj.post(...) resolves.
set -euo pipefail

# ===== Defaults (override via env or CLI flags) =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"        # merge-install prefix for the v3 runtime
GEN_PREFIX="${GEN_PREFIX:-/opt/fast-dds-gen-v3}"  # installation prefix for fastddsgen launcher
WS="${WS:-$HOME/fastdds_python_ws}"               # colcon workspace
REPOS_FILE="${REPOS_FILE:-$WS/fastdds_python.repos}"
PYBIN="${PYBIN:-python3}"                         # Python used to create the venv
JOBS="${JOBS:-$(/usr/sbin/sysctl -n hw.ncpu 2>/dev/null || echo 4)}"
ASIO_VER="${ASIO_VER:-1.12.2}"                    # <- old standalone Asio (keeps io_service etc.)

# ===== CLI flags =====
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix-v3)   PREFIX_V3="$2"; shift 2;;
    --gen-prefix)  GEN_PREFIX="$2"; shift 2;;
    --ws)          WS="$2"; shift 2;;
    --repos)       REPOS_FILE="$2"; shift 2;;
    --python)      PYBIN="$2"; shift 2;;
    -j|--jobs)     JOBS="$2"; shift 2;;
    --asio-ver)    ASIO_VER="$2"; shift 2;;
    *) echo "[WARN] Unknown arg: $1"; shift;;
  esac
done

# ===== Helpers =====
log(){ echo -e "\033[1;36m[INFO]\033[0m $*"; }
warn(){ echo -e "\033[1;33m[WARN]\033[0m $*" >&2; }
die(){ echo -e "\033[1;31m[FATAL]\033[0m $*" >&2; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "'$1' not found"; }

# ===== Homebrew & toolchain =====
if ! command -v brew >/dev/null 2>&1; then
  die "Homebrew not found. Install: https://brew.sh/"
fi
BREW_PREFIX="$(brew --prefix)"

log "Installing build deps via Homebrew…"
brew update
# NOTE: skip standalone 'asio' (newer APIs break legacy Fast DDS usage)
brew install cmake ninja git pkg-config tinyxml2 wget curl swig gradle openssl@3

# Ensure Python build dependencies
"${PYBIN}" -m pip install --upgrade pip setuptools wheel || true

# Java (for Fast-DDS-Gen)
if ! /usr/libexec/java_home -V >/dev/null 2>&1; then
  brew install openjdk@17
  sudo ln -sf "${BREW_PREFIX}/opt/openjdk@17/libexec/openjdk.jdk" /Library/Java/JavaVirtualMachines/openjdk-17.jdk || true
fi
export JAVA_HOME="$("/usr/libexec/java_home" -v 17 2>/dev/null || /usr/libexec/java_home)"

need git; need curl; need "${PYBIN}"

# ===== Workspace & venv =====
log "Preparing workspace at: ${WS}"
rm -rf "${WS}"
mkdir -p "${WS}/src" "${WS}/.deps"
cd "${WS}"

log "Creating venv (.venv) with ${PYBIN}…"
"${PYBIN}" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -U pip wheel
python -m pip install -U colcon-common-extensions vcstool empy

# ===== Fetch repos =====
if [[ ! -f "${REPOS_FILE}" ]]; then
  log "Fetching default repos file (Fast-DDS-python v2.2.0)…"
  curl -fsSL -o "${REPOS_FILE}" \
    https://raw.githubusercontent.com/eProsima/Fast-DDS-python/v2.2.0/fastdds_python.repos
fi
[[ -f "${REPOS_FILE}" ]] || die "repos file not found: ${REPOS_FILE}"

log "Importing repos into src/…"
vcs import --recursive src < "${REPOS_FILE}"

# ===== Bring old standalone Asio headers (1.12.x) =====
# Asio 1.12.2 tarball URL: https://sourceforge.net/projects/asio/files/asio/1.12.2%20%28Stable%29/asio-1.12.2.tar.bz2/download
# Alternatively: https://github.com/chriskohlhoff/asio/releases/tag/asio-1-12-2
ASIO_DIR="${WS}/.deps/asio-${ASIO_VER}"
if [[ ! -d "${ASIO_DIR}/include" ]]; then
  log "Fetching standalone Asio ${ASIO_VER} headers…"
  case "${ASIO_VER}" in
    1.12.*)
      curl -fL -o ".deps/asio-${ASIO_VER}.tar.bz2" \
        "https://sourceforge.net/projects/asio/files/asio/${ASIO_VER//./.}%20%28Stable%29/asio-${ASIO_VER}.tar.bz2/download" \
        || curl -fL -o ".deps/asio-${ASIO_VER}.tar.gz" \
        "https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-${ASIO_VER//./-}.tar.gz"
      ;;
    *)
      # Match GitHub tag naming scheme
      curl -fL -o ".deps/asio-${ASIO_VER}.tar.gz" \
        "https://github.com/chriskohlhoff/asio/archive/refs/tags/asio-${ASIO_VER//./-}.tar.gz"
      ;;
  esac
  if [[ -f ".deps/asio-${ASIO_VER}.tar.bz2" ]]; then
    tar -C ".deps" -xjf ".deps/asio-${ASIO_VER}.tar.bz2"
  else
    tar -C ".deps" -xzf ".deps/asio-${ASIO_VER}.tar.gz"
  fi
  # Normalize folder name variations
  if [[ ! -d "${ASIO_DIR}" ]]; then
    ASIO_DIR_FALLBACK="$(find ".deps" -maxdepth 1 -type d -name "asio-*" | grep "${ASIO_VER//./-}\|${ASIO_VER}" | head -n1 || true)"
    [[ -n "${ASIO_DIR_FALLBACK}" ]] && ASIO_DIR="${ASIO_DIR_FALLBACK}"
  fi
fi
[[ -d "${ASIO_DIR}/include" ]] || die "Asio ${ASIO_VER} include not found at: ${ASIO_DIR}/include"
log "Using standalone Asio from: ${ASIO_DIR}"

# ===== Locate Fast-DDS source tree (for info only) =====
log "Locating Fast-DDS source tree under ${WS}/src…"
FASTDDS_DIR="$(find "${WS}/src" -maxdepth 2 -type d \( -iname fastdds -o -iname 'Fast-DDS' \) | head -n1 || true)"
[ -n "${FASTDDS_DIR}" ] && [ -d "${FASTDDS_DIR}" ] || die "fastdds source tree not found under ${WS}/src"
log "Found fastdds at: ${FASTDDS_DIR}"

# ===== macOS-specific Fast DDS patching =====
MAC_AFFINITY_FILE="${FASTDDS_DIR}/src/cpp/utils/threading/threading_osx.ipp"
if [[ -f "${MAC_AFFINITY_FILE}" ]]; then
  if grep -q "LWRCLPY_DISABLE_OSX_AFFINITY_PATCH" "${MAC_AFFINITY_FILE}"; then
    log "macOS affinity patch already applied."
  else
    log "Patching Fast DDS to disable unsupported macOS thread affinity…"
    export MAC_AFFINITY_FILE
    python <<'PY'
from pathlib import Path
import os
import sys

target_path = Path(os.environ["MAC_AFFINITY_FILE"])
marker = "LWRCLPY_DISABLE_OSX_AFFINITY_PATCH"
text = target_path.read_text()
if marker in text:
    sys.exit(0)

needle = "static void configure_current_thread_affinity"
start = text.find(needle)
if start == -1:
    print(f"[patch] Unable to find '{needle}' inside {target_path}", file=sys.stderr)
    sys.exit(1)

brace_pos = text.find('{', start)
if brace_pos == -1:
    print("[patch] Unable to locate opening brace for affinity function", file=sys.stderr)
    sys.exit(1)

depth = 0
end = None
for idx in range(brace_pos, len(text)):
    char = text[idx]
    if char == '{':
        depth += 1
    elif char == '}':
        depth -= 1
        if depth == 0:
            end = idx
            break

if end is None:
    print("[patch] Unable to locate closing brace for affinity function", file=sys.stderr)
    sys.exit(1)

replacement = """static void configure_current_thread_affinity(
        const char* thread_name,
        uint64_t affinity)
{
    (void) thread_name;
    (void) affinity;
    // LWRCLPY_DISABLE_OSX_AFFINITY_PATCH: macOS lacks supported APIs to pin DDS threads, leave as no-op.
}
"""

target_path.write_text(text[:start] + replacement + text[end + 1:])
PY
    unset MAC_AFFINITY_FILE
    log "Applied macOS thread affinity patch to Fast DDS."
  fi
else
  warn "macOS affinity patch skipped; file not found: ${MAC_AFFINITY_FILE}"
fi

# ===== fastddsgen build & install =====
GEN_SRC_DIR="$(find "${WS}/src" -maxdepth 2 -type d \( -iname 'fastddsgen' -o -iname 'Fast-DDS-Gen' \) | head -n 1 || true)"
[[ -n "${GEN_SRC_DIR}" ]] || die "Fast-DDS-Gen repo not found under ${WS}/src"
log "Building fastddsgen from: ${GEN_SRC_DIR}"

sudo mkdir -p "${GEN_PREFIX}"
pushd "${GEN_SRC_DIR}" >/dev/null
  ./gradlew --no-daemon clean assemble
  sudo ./gradlew --no-daemon install --install_path="${GEN_PREFIX}"
popd >/dev/null

export PATH="${GEN_PREFIX}/bin:${PATH}"
log "fastddsgen: $(command -v fastddsgen)"

# ===== Probe fastddsgen -python =====
log "Probing fastddsgen -python by real generation…"
PROBE_DIR="$(mktemp -d)"
trap 'rm -rf "${PROBE_DIR}"' EXIT
OUT_DIR="${PROBE_DIR}/out"
mkdir -p "${OUT_DIR}"
cat > "${PROBE_DIR}/Probe.idl" <<'IDL'
module probe { struct Foo { long x; }; };
IDL
set +e
fastddsgen -python -d "${OUT_DIR}" -I "${PROBE_DIR}" -replace "${PROBE_DIR}/Probe.idl" >"${PROBE_DIR}/gen.log" 2>&1
RC=$?
set -e
if [[ $RC -ne 0 ]] || ! find "${OUT_DIR}" -name '*.i' -print -quit | grep -q .; then
  sed -n '1,160p' "${PROBE_DIR}/gen.log" || true
  die "fastddsgen -python probe failed (see above)"
fi
log "fastddsgen -python OK (.i generated)"

# ===== Build the v3 stack + fastdds_python via colcon =====
log "Building with colcon → install to ${PREFIX_V3}"
sudo mkdir -p "${PREFIX_V3}"
sudo chown "$(id -u)":"$(id -g)" "${PREFIX_V3}"

PY_EXEC="$(command -v python)"
CMAKE_PREFIX_PATH="${PREFIX_V3}:${BREW_PREFIX}"

# Best-guess defaults (initial run may be empty)
FOONATHAN_DIR_CAND1="${PREFIX_V3}/lib/foonathan_memory/cmake"
FOONATHAN_DIR_CAND2="${PREFIX_V3}/lib/cmake/foonathan_memory"
FOONATHAN_DIR="${FOONATHAN_DIR_CAND1}"; [[ -d "${FOONATHAN_DIR_CAND2}" ]] && FOONATHAN_DIR="${FOONATHAN_DIR_CAND2}"

FASTCDR_DIR_CAND1="${PREFIX_V3}/lib/cmake/fastcdr"
FASTCDR_DIR_CAND2="${PREFIX_V3}/share/fastcdr/cmake"
FASTCDR_DIR="${FASTCDR_DIR_CAND1}"; [[ -d "${FASTCDR_DIR_CAND2}" ]] && FASTCDR_DIR="${FASTCDR_DIR_CAND2}"

log "Using foonathan_memory_DIR=${FOONATHAN_DIR}"
log "Using fastcdr_DIR=${FASTCDR_DIR}"

# Ensure old standalone Asio headers take precedence via explicit -I.
# Fast DDS sets ASIO_STANDALONE, so standalone headers must be found first.
# Disable SHM transport for stability; re-enable later if needed.
CMAKE_COMMON_ARGS=(
  -G Ninja
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${PREFIX_V3}"
  -DCMAKE_INSTALL_RPATH="${PREFIX_V3}/lib;${PREFIX_V3}/lib64"
  -DPython3_EXECUTABLE="${PY_EXEC}"
  -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
  -Dfoonathan_memory_DIR="${FOONATHAN_DIR}"
  -Dfastcdr_DIR="${FASTCDR_DIR}"
  -DCMAKE_MACOSX_RPATH=ON
  # Key point: prioritize legacy Asio headers
  -DCMAKE_CXX_FLAGS="-I${ASIO_DIR}/include"
  -DCMAKE_C_FLAGS="-I${ASIO_DIR}/include"
  -DFASTDDS_SHM_TRANSPORT_DEFAULT=OFF
)

colcon build \
  --base-paths src \
  --merge-install \
  --install-base "${PREFIX_V3}" \
  --cmake-args "${CMAKE_COMMON_ARGS[@]}" \
  --event-handlers console_cohesion+ status+ \
  --executor sequential \
  --packages-up-to fastdds_python fastdds foonathan_memory_vendor fastcdr \
  --parallel-workers "${JOBS}"

# ===== Export instructions =====
cat <<'EOF'

========================================
✅ Installation completed (macOS, no patch; pinned standalone Asio 1.12.x)

# Add these to your shell (~/.zshrc recommended)
export PATH="$PATH:__GEN_PREFIX__/bin"
export CMAKE_PREFIX_PATH="__PREFIX_V3__:$CMAKE_PREFIX_PATH"
export DYLD_LIBRARY_PATH="__PREFIX_V3__/lib:__PREFIX_V3__/lib64:$DYLD_LIBRARY_PATH"
export PYTHONPATH="__PREFIX_V3__/lib/python$(python -c 'import sys;print("{}.{}".format(*sys.version_info[:2]))')/site-packages:$PYTHONPATH"

# Quick sanity checks
which fastddsgen && fastddsgen -version
python - <<'PY'
import fastdds
print("[OK] fastdds Python available")
print("KEEP_* symbols:", [n for n in dir(fastdds) if "KEEP" in n][:6], "…")
PY

# Notes:
# - We DO NOT install Homebrew 'asio'. We pin standalone Asio to 1.12.x and put its include first via -I.
# - If you previously installed 'asio' via Homebrew, it won't hurt because -I${ASIO_DIR}/include wins over -isystem /opt/homebrew/include.
# - If you need SHM transport, rebuild with -DFASTDDS_SHM_TRANSPORT_DEFAULT=ON after confirming stability.
# - If you experimented earlier, clean: rm -rf build install log && re-run this script.
# - Change Asio version with:  --asio-ver 1.12.2  (default) / 1.12.1 etc.
========================================
EOF | sed "s|__GEN_PREFIX__|${GEN_PREFIX}|g; s|__PREFIX_V3__|${PREFIX_V3}|g"
