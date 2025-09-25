#!/usr/bin/env bash
# Fast DDS v3 + fastddsgen + fastdds_python を colcon で /opt/fast-dds-v3 にインストール
# 依存は fastdds_python.repos に準拠。Ubuntu 22.04/24.04 想定。
set -euo pipefail

# ===== 既定設定 =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"      # colcon の install 先（v3 ランタイム）
GEN_PREFIX="${GEN_PREFIX:-/opt/fast-dds-gen}"   # fastddsgen のインストール先
WS="${WS:-$HOME/fastdds_python_ws}"             # colcon ワークスペース
REPOS_FILE="${REPOS_FILE:-$WS/fastdds_python.repos}"
PYBIN="${PYBIN:-python3}"                        # venv のベースにする Python
JOBS="${JOBS:-$(command -v nproc >/dev/null && nproc || echo 4)}"

# ===== 任意引数 =====
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

log(){ echo -e "\033[1;36m[INFO]\033[0m $*"; }
warn(){ echo -e "\033[1;33m[WARN]\033[0m $*" >&2; }
die(){ echo -e "\033[1;31m[FATAL]\033[0m $*" >&2; exit 1; }
need(){ command -v "$1" >/dev/null 2>&1 || die "'$1' not found"; }

# ===== 依存導入 =====
log "Installing system dependencies (apt)…"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential git pkg-config \
  "${PYBIN%-*}-venv" "${PYBIN%-*}-dev" \
  unzip wget curl \
  libasio-dev libtinyxml2-dev \
  cmake ninja-build python3-dev python3-pip

if ! dpkg -l | grep -qw openjdk-11-jre || ! dpkg -l | grep -qw openjdk-11-jdk; then
    sudo apt update
    sudo apt purge -y openjdk-* default-jdk default-jre --autoremove
    sudo apt install -y openjdk-11-jre openjdk-11-jdk
fi

# SWIG は 4.2 未満を推奨（4.1など）。
sudo apt-get install -y swig4.1 || true

need git
need curl
need "${PYBIN}"

# ===== ワークスペース & venv =====
log "Preparing workspace at: ${WS}"
mkdir -p "${WS}/src"
cd "${WS}"

log "Creating venv (.venv) with ${PYBIN}…"
"${PYBIN}" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install -U pip wheel
python -m pip install -U colcon-common-extensions vcstool empy


# ===== repos 取得 =====
if [[ ! -f "${REPOS_FILE}" ]]; then
  log "Fetching default repos file (Fast-DDS-python v2.3.0)…"
  curl -fsSL -o "${REPOS_FILE}" \
    https://raw.githubusercontent.com/eProsima/Fast-DDS-python/v2.3.0/fastdds_python.repos
fi
[[ -f "${REPOS_FILE}" ]] || die "repos file not found: ${REPOS_FILE}"

log "Importing repos into src/…"
vcs import --recursive src < "${REPOS_FILE}"

# ===== fastddsgen (v3系) のビルド & インストール =====
GEN_SRC_DIR="$(find "${WS}/src" -maxdepth 2 -type d \( -iname 'fastddsgen' -o -iname 'Fast-DDS-Gen' \) | head -n 1 || true)"
[[ -n "${GEN_SRC_DIR}" ]] || die "Fast-DDS-Gen repo not found under ${WS}/src (check your repos file)"
log "Building fastddsgen from: ${GEN_SRC_DIR}"

pushd "${GEN_SRC_DIR}" >/dev/null
  ./gradlew --no-daemon clean assemble
  sudo ./gradlew --no-daemon install --install_path="${GEN_PREFIX}"
popd >/dev/null

# PATH に追加（このシェル内）
export PATH="${GEN_PREFIX}/bin:${PATH}"
log "fastddsgen: $(command -v fastddsgen)"

# ===== fastddsgen -python プローブ =====
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

# ===== colcon で v3 スタック + fastdds_python を ${PREFIX_V3} にインストール =====
log "Building with colcon → install to ${PREFIX_V3}"
sudo mkdir -p "${PREFIX_V3}"
sudo chown "$(id -u)":"$(id -g)" "${PREFIX_V3}"

PY_EXEC="$(command -v python)"

# 重要: CMAKE_PREFIX_PATH と個別 DIR を明示（foonathan/fastcdr を fastdds から見えるように）
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

# colcon 実行
colcon build \
  --base-paths src \
  --merge-install \
  --install-base "${PREFIX_V3}" \
  --cmake-args "${CMAKE_COMMON_ARGS[@]}" \
  --event-handlers console_cohesion+ status+ \
  --executor sequential \
  --packages-up-to fastdds_python fastdds foonathan_memory_vendor fastcdr \
  --parallel-workers "${JOBS}"

# ===== エクスポート案内 =====
cat <<EOF

========================================
✅ インストール完了

# シェルに反映（~/.bashrc 等に追記推奨）
export PATH=\$PATH:${GEN_PREFIX}/bin
export CMAKE_PREFIX_PATH="${PREFIX_V3}:\$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="${PREFIX_V3}/lib:${PREFIX_V3}/lib64:\$LD_LIBRARY_PATH"
export PYTHONPATH="${PREFIX_V3}/lib/python$(python -c 'import sys;print("{}.{}".format(*sys.version_info[:2]))')/site-packages:\$PYTHONPATH"

# 動作確認
which fastddsgen && fastddsgen -version
python - <<'PY'
import fastdds
print("[OK] fastdds Python available")
print("KEEP_* symbols:", [n for n in dir(fastdds) if "KEEP" in n][:6], "...")
PY

# 再ビルド例（依存やオプションを変えたとき）
#   source ${WS}/.venv/bin/activate
#   colcon build --merge-install --install-base "${PREFIX_V3}" --cmake-args "${CMAKE_COMMON_ARGS[@]}"
========================================
EOF