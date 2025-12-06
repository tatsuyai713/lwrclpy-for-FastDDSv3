#!/usr/bin/env bash
# Install Fast DDS v3 core + fastddsgen + fastdds_python via colcon into /opt/fast-dds-v3.
# Dependencies are aligned with fastdds_python.repos. Tested on Ubuntu 22.04/24.04.
set -euo pipefail

# ===== Defaults (override via env or CLI flags) =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"      # install/merge-install prefix for the v3 runtime
GEN_PREFIX="${GEN_PREFIX:-/opt/fast-dds-gen-v3}"   # installation prefix for fastddsgen launcher
WS="${WS:-$HOME/fastdds_python_ws}"             # colcon workspace
REPOS_FILE="${REPOS_FILE:-$WS/fastdds_python.repos}"
PYBIN="${PYBIN:-python3}"                        # Python used to create the venv
JOBS="${JOBS:-$(command -v nproc >/dev/null && nproc || echo 4)}"

# ===== CLI flags =====
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix-v3)   PREFIX_V3="$2"; shift 2;;
    --gen-prefix)  GEN_PREFIX="$2"; shift 2;;
    --ws)          WS="$2"; shift 2;;
    --repos)       REPOS_FILE="$2"; shift 2;;
    --python)      PYBIN="$2"; shift 2;;
    -j|--jobs)     JOBS="$2"; shift 2;;
    *) echo "[WARN] Unknown arg: $1"; shift;;
  esac
done

# Pretty log helpers
log(){ echo -e "\033[1;36m[INFO]\033[0m $*"; }
warn(){ echo -e "\033[1;33m[WARN]\033[0m $*" >&2; }
die(){ echo -e "\033[1;31m[FATAL]\033[0m $*" >&2; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "'$1' not found"; }

# ===== System dependencies (apt) =====
log "Installing system dependencies (apt)…"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential git pkg-config \
  "${PYBIN%-*}-venv" "${PYBIN%-*}-dev" \
  "${PYBIN%-*}-setuptools" "${PYBIN%-*}-wheel" \
  unzip wget curl \
  libasio-dev libtinyxml2-dev libssl-dev \
  cmake ninja-build python3-dev python3-pip \
  patchelf

# Java is required to build Fast-DDS-Gen (gradle)
if ! dpkg -l | grep -qw openjdk-11-jre || ! dpkg -l | grep -qw openjdk-11-jdk; then
  sudo apt-get update
  sudo apt-get install -y openjdk-11-jre openjdk-11-jdk
fi

# SWIG 4.1 is recommended (avoid 4.2+ unless you have patches)
sudo apt-get install -y swig4.1 || true

need git
need curl
need "${PYBIN}"

# ===== Workspace & virtualenv =====
log "Preparing workspace at: ${WS}"
rm -rf "${WS}"
mkdir -p "${WS}/src"
cd "${WS}"

log "Creating venv (.venv) with ${PYBIN}…"
"${PYBIN}" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -U pip wheel
python -m pip install -U colcon-common-extensions vcstool empy

# ===== Fetch repos (vcstool) =====
# Use a pinned Fast-DDS-python repos file for reproducibility
if [[ ! -f "${REPOS_FILE}" ]]; then
  log "Fetching default repos file (Fast-DDS-python v2.2.0)…"
  curl -fsSL -o "${REPOS_FILE}" \
    https://raw.githubusercontent.com/eProsima/Fast-DDS-python/v2.2.0/fastdds_python.repos
fi
[[ -f "${REPOS_FILE}" ]] || die "repos file not found: ${REPOS_FILE}"

log "Importing repos into src/…"
vcs import --recursive src < "${REPOS_FILE}"

# ===== Build & install fastddsgen (Gradle) into ${GEN_PREFIX} =====
GEN_SRC_DIR="$(find "${WS}/src" -maxdepth 2 -type d \( -iname 'fastddsgen' -o -iname 'Fast-DDS-Gen' \) | head -n 1 || true)"
[[ -n "${GEN_SRC_DIR}" ]] || die "Fast-DDS-Gen repo not found under ${WS}/src (check your repos file)"
log "Building fastddsgen from: ${GEN_SRC_DIR}"

# Ensure Java 11 is used for Gradle
export JAVA_HOME="/usr/lib/jvm/java-11-openjdk-amd64"
log "Using JAVA_HOME=${JAVA_HOME}"

pushd "${GEN_SRC_DIR}" >/dev/null
  ./gradlew --no-daemon clean assemble
  sudo JAVA_HOME="${JAVA_HOME}" ./gradlew --no-daemon install --install_path="${GEN_PREFIX}"
popd >/dev/null

# Make fastddsgen available in PATH for this shell
export PATH="${GEN_PREFIX}/bin:${PATH}"
log "fastddsgen: $(command -v fastddsgen)"

# ===== Probe fastddsgen -python (sanity check) =====
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

# ===== Build the v3 stack + fastdds_python via colcon into ${PREFIX_V3} =====
log "Building with colcon → install to ${PREFIX_V3}"
sudo mkdir -p "${PREFIX_V3}"
sudo chown "$(id -u)":"$(id -g)" "${PREFIX_V3}"

PY_EXEC="$(command -v python)"

# Important: expose foonathan_memory and fastcdr CMake config paths to Fast-DDS
CMAKE_PREFIX_PATH="${PREFIX_V3}"
FOONATHAN_DIR_CAND1="${PREFIX_V3}/lib/foonathan_memory/cmake"
FOONATHAN_DIR_CAND2="${PREFIX_V3}/lib/cmake/foonathan_memory"
FOONATHAN_DIR="${FOONATHAN_DIR_CAND1}"
[[ -d "${FOONATHAN_DIR_CAND2}" ]] && FOONATHAN_DIR="${FOONATHAN_DIR_CAND2}"

FASTCDR_DIR_CAND1="${PREFIX_V3}/lib/cmake/fastcdr"
FASTCDR_DIR_CAND2="${PREFIX_V3}/share/fastcdr/cmake"
FASTCDR_DIR="${FASTCDR_DIR_CAND1}"
[[ -d "${FASTCDR_DIR_CAND2}" ]] && FASTCDR_DIR="${FASTCDR_DIR_CAND2}"

log "Using foonathan_memory_DIR=${FOONATHAN_DIR}"
log "Using fastcdr_DIR=${FASTCDR_DIR}"

CMAKE_COMMON_ARGS=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${PREFIX_V3}"
  -DCMAKE_INSTALL_RPATH="${PREFIX_V3}/lib;${PREFIX_V3}/lib64"
  -DPython3_EXECUTABLE="${PY_EXEC}"
  -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
  -Dfoonathan_memory_DIR="${FOONATHAN_DIR}"
  -Dfastcdr_DIR="${FASTCDR_DIR}"
)

# Build a minimal set of packages required to get fastdds_python working
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
cat <<EOF

========================================
✅ Installation completed

# Add these to your shell (~/.bashrc recommended)
export PATH=\$PATH:${GEN_PREFIX}/bin
export CMAKE_PREFIX_PATH="${PREFIX_V3}:\$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="${PREFIX_V3}/lib:${PREFIX_V3}/lib64:\$LD_LIBRARY_PATH"
export PYTHONPATH="${PREFIX_V3}/lib/python\$(python -c 'import sys;print("{}.{}".format(*sys.version_info[:2]))')/site-packages:\$PYTHONPATH"

# Quick sanity checks
which fastddsgen && fastddsgen -version
python - <<'PY'
import fastdds
print("[OK] fastdds Python available")
print("KEEP_* symbols:", [n for n in dir(fastdds) if "KEEP" in n][:6], "…")
PY

# Rebuild example if you change options/deps:
#   source ${WS}/.venv/bin/activate
#   colcon build --merge-install --install-base "${PREFIX_V3}" --cmake-args ${CMAKE_COMMON_ARGS[@]}
========================================
EOF
