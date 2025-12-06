#!/usr/bin/env bash
# gen_python_types.sh (Fast DDS v3 + fastddsgen 4.x)
# Pipeline overview:
#   1) Mirror the IDL tree into BUILD_ROOT/src while preserving structure.
#   2) For each IDL, generate Python bindings into a sibling "<TypeName>/" dir (structure-preserving).
#   3) Patch the generated SWIG .i for Fast DDS v3 (idempotent).
#   4) Ensure CMakeLists.txt adds include dirs and SWIG -I (so cross-type includes resolve).
#   5) Build each generated directory independently.

set -euo pipefail

# ===== Ensure build dependencies =====
echo "[INFO] Ensuring build dependencies..."
python3 -m pip install --upgrade pip setuptools wheel || true

# ===== User-tunable settings =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"                   # Fast-DDS v3 install prefix (CMake packages live here)
FASTDDSGEN_BIN="${FASTDDSGEN_BIN:-/opt/fast-dds-gen-v3/bin/fastddsgen}"  # fastddsgen v4.x launcher

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_TYPES_ROOT="${ROS_TYPES_ROOT:-${ROOT_DIR}/third_party/ros-data-types-for-fastdds}"

BUILD_ROOT="${BUILD_ROOT:-${ROOT_DIR}/._types_python_build_v3}"
GEN_SRC_ROOT="${BUILD_ROOT}/src"            # Where the mirrored IDL tree will live
INC_STAGE_ROOT="${BUILD_ROOT}/include/src"  # Staging for .i/.hpp to be included by other generated types
PATCH_PY="${PATCH_PY:-${SCRIPT_DIR}/patch_fastdds_swig_v3.py}"  # SWIG .i patcher adapted for v3

# Worker count (portable detection)
detect_cores(){ command -v nproc >/dev/null && nproc || (command -v sysctl >/dev/null && sysctl -n hw.ncpu) || echo 4; }
JOBS="${JOBS:-$(detect_cores)}"

# Simple command presence check
need(){ command -v "$1" >/dev/null 2>&1 || { echo "[FATAL] '$1' not found" >&2; exit 1; }; }

# ===== Prerequisites =====
need cmake
need swig
need python3
need java
[[ -x "${FASTDDSGEN_BIN}" ]] || { echo "[FATAL] fastddsgen not found: ${FASTDDSGEN_BIN}"; exit 1; }
[[ -d "${ROS_TYPES_ROOT}/src" ]] || { echo "[FATAL] ROS_TYPES_ROOT/src not found: ${ROS_TYPES_ROOT}/src"; exit 1; }
[[ -f "${PATCH_PY}" ]] || { echo "[FATAL] patch script missing: ${PATCH_PY}"; exit 1; }

mkdir -p "${BUILD_ROOT}" "${GEN_SRC_ROOT}" "${INC_STAGE_ROOT}"

# ===== Auto-detect CMake package dirs for fastdds / fastcdr =====
# Looks in common locations under PREFIX_V3. Keeps code paths portable across distros.
autodetect_pkg_dir() {
  local prefix="$1" pkg="$2" cand
  cand="$(find "${prefix}/share/${pkg}/cmake" -maxdepth 1 -name "${pkg}-config.cmake" -print -quit 2>/dev/null || true)"
  [[ -n "${cand}" ]] && { dirname "${cand}"; return 0; }
  cand="$(find "${prefix}/lib/cmake/${pkg}" -maxdepth 1 -name "${pkg}-config.cmake" -print -quit 2>/dev/null || true)"
  [[ -n "${cand}" ]] && { dirname "${cand}"; return 0; }
  cand="$(find "${prefix}" -type f -name "${pkg}-config.cmake" -print -quit 2>/dev/null || true)"
  [[ -n "${cand}" ]] && { dirname "${cand}"; return 0; }
  return 1
}
if ! FASTDDS_CMAKE_DIR="$(autodetect_pkg_dir "${PREFIX_V3}" fastdds)"; then
  echo "[FATAL] fastdds-config.cmake not found under ${PREFIX_V3}"; exit 1; fi
if ! FASTCDR_CMAKE_DIR="$(autodetect_pkg_dir "${PREFIX_V3}" fastcdr)"; then
  echo "[FATAL] fastcdr-config.cmake not found under ${PREFIX_V3}"; exit 1; fi
echo "[INFO] fastdds_DIR=${FASTDDS_CMAKE_DIR}"
echo "[INFO] fastcdr_DIR=${FASTCDR_CMAKE_DIR}"

# ===== Mirror the IDL tree into GEN_SRC_ROOT (structure-preserving) =====
echo "[INFO] Mirroring IDL tree → ${GEN_SRC_ROOT}"
rsync -a --delete "${ROS_TYPES_ROOT}/src/" "${GEN_SRC_ROOT}/"

# Workaround for reserved word 'FIXED' in gazebo_msgs: rename to '_FIXED' only in the mirrored copy
mapfile -t GZ_IDLS < <(find "${GEN_SRC_ROOT}/gazebo_msgs" -type f -name "*.idl" 2>/dev/null || true)
for f in "${GZ_IDLS[@]}"; do
  sed -i -E 's/(^|[^A-Za-z0-9_])FIXED([^A-Za-z0-9_])/\1_FIXED\2/g' "$f" || true
done

# ===== Enumerate IDL files (optional FILTER=… to limit the scope) =====
if [[ -n "${FILTER:-}" ]]; then
  mapfile -t IDLS < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" | sed "s|^${GEN_SRC_ROOT}/||" | grep -i "${FILTER}" | sort)
else
  mapfile -t IDLS < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" | sed "s|^${GEN_SRC_ROOT}/||" | sort)
fi
[[ ${#IDLS[@]} -gt 0 ]] || { echo "[ERR] No IDL files (FILTER='${FILTER:-}')"; exit 1; }

# ===== Ensure CMakeLists includes our staging include paths (so SWIG/C++ can find sibling types) =====
patch_cmakelists_min() {
  local dir="$1"      # e.g. .../src/<pkg>/<ns>/<Type>
  local dir_rel="$2"  # e.g. <pkg>/<ns>
  local cml="${dir}/CMakeLists.txt"
  [[ -f "$cml" ]] || return 0
  if ! grep -q "__FASTDDS_INC_STAGE_ADDED__" "$cml"; then
    {
      echo "# __FASTDDS_INC_STAGE_ADDED__"
      echo "set(FASTDDS_GEN_INCLUDE_STAGE \"${INC_STAGE_ROOT}\")"
      echo "include_directories(\"\${FASTDDS_GEN_INCLUDE_STAGE}\")"
      echo "set(CMAKE_SWIG_FLAGS \${CMAKE_SWIG_FLAGS} \"-I\${FASTDDS_GEN_INCLUDE_STAGE}\")"
      echo "set(SWIG_INCLUDE_DIRS \"\${SWIG_INCLUDE_DIRS};\${FASTDDS_GEN_INCLUDE_STAGE}\")"
      echo "# __FASTDDS_SUBINC_ADDED__"
      echo "include_directories(\"${INC_STAGE_ROOT}/${dir_rel}\")"
      echo "set(CMAKE_SWIG_FLAGS \${CMAKE_SWIG_FLAGS} \"-I${INC_STAGE_ROOT}/${dir_rel}\")"
      echo "set(SWIG_INCLUDE_DIRS \"\${SWIG_INCLUDE_DIRS};${INC_STAGE_ROOT}/${dir_rel}\")"
    } | cat - "$cml" > "$cml.tmp" && mv "$cml.tmp" "$cml"
  fi
}

# ===== Generate → Patch → Stage includes for each IDL =====
FAILED_GEN=()

gen_one() {
  local rel="$1"                               # e.g. builtin_interfaces/msg/Time.idl
  local idl_path="${GEN_SRC_ROOT}/${rel}"
  local dir_rel="$(dirname "${rel}")"          # e.g. builtin_interfaces/msg
  local base="$(basename "${rel}" .idl)"       # e.g. Time
  local outdir="${GEN_SRC_ROOT}/${dir_rel}/${base}"  # e.g. src/builtin_interfaces/msg/Time/…

  rm -rf "${outdir}"
  mkdir -p "${outdir}"

  echo "[GEN] ${rel} -> ${outdir}"
  # -cs: generate TypeObject support; -typeros2: ROS2-friendly types; -language c++
  if ! "${FASTDDSGEN_BIN}" -python -cs -typeros2 -language c++ \
        -d "${outdir}" \
        -I "${GEN_SRC_ROOT}" \
        -replace "${idl_path}" > "${outdir}/_gen.log" 2>&1; then
    echo "[ERR]  fastddsgen failed: ${rel} (see ${outdir}/_gen.log head)"
    sed -n '1,120p' "${outdir}/_gen.log" || true
    FAILED_GEN+=("${rel}")
    return
  fi
  
  
  [[ -f "${outdir}/${base}.i" ]] || { echo "[ERR]  ${base}.i not generated"; FAILED_GEN+=("${rel}"); return; }

  echo "[PATCH.i] ${rel}"
  python3 "${PATCH_PY}" "${outdir}/${base}.i"

  # Stage includes (.i/.hpp/TypeObjectSupport/PubSubTypes) into a central include tree
  local inc_dst_dir="${INC_STAGE_ROOT}/${dir_rel}"
  mkdir -p "${inc_dst_dir}"
  find "${outdir}" -maxdepth 1 -type f \
    \( -name '*.i' -o -name '*.hpp' -o -name '*TypeObjectSupport.*' -o -name '*PubSubTypes.*' \) \
    -exec cp -f {} "${inc_dst_dir}/" \;

  # Ensure each generated CMakeLists sees the staging includes
  patch_cmakelists_min "${outdir}" "${dir_rel}"
}

for rel in "${IDLS[@]}"; do
  gen_one "${rel}"
done

if [[ ${#FAILED_GEN[@]} -gt 0 ]]; then
  echo "[WARN] fastddsgen failed on ${#FAILED_GEN[@]} file(s); continuing with remaining builds"
  printf ' - %s\n' "${FAILED_GEN[@]}"
fi

# ===== Build all generated packages =====
echo "[INFO] Building each generated package…"
FAILED_BUILD=()

build_one() {
  local outdir="$1"
  echo "[CMAKE] ${outdir#${BUILD_ROOT}/}"
  pushd "${outdir}" >/dev/null
    mkdir -p build && cd build
    CMAKE_ARGS=(
      -DCMAKE_BUILD_TYPE=Release
      -DCMAKE_CXX_STANDARD=17
      -DCMAKE_PREFIX_PATH="${PREFIX_V3}"
      -Dfastdds_DIR="${FASTDDS_CMAKE_DIR}"
      -Dfastcdr_DIR="${FASTCDR_CMAKE_DIR}"
      -DPython3_EXECUTABLE="$(command -v python3)"
    )
    if ! cmake .. "${CMAKE_ARGS[@]}" > _cmake_configure.log 2>&1; then
      echo "[ERR]  cmake configure failed: ${outdir#${BUILD_ROOT}/}"
      sed -n '1,200p' _cmake_configure.log || true
      popd >/dev/null; FAILED_BUILD+=("${outdir} (configure)"); return
    fi
    if ! cmake --build . -j "${JOBS}" > _cmake_build.log 2>&1; then
      echo "[ERR]  cmake build failed: ${outdir#${BUILD_ROOT}/}"
      sed -n '1,200p' _cmake_build.log || true
      popd >/dev/null; FAILED_BUILD+=("${outdir} (build)"); return
    fi
  popd >/dev/null
}

# Discover all generated leaf dirs that own a CMakeLists.txt
mapfile -t OUTDIRS < <(find "${GEN_SRC_ROOT}" -mindepth 4 -maxdepth 4 -type f -name CMakeLists.txt -printf '%h\n' | sort -u)
for d in "${OUTDIRS[@]}"; do
  # Skip those with explicit generator error markers
  [[ -f "${d}/_gen.log" ]] && grep -q "ERROR" "${d}/_gen.log" && continue
  build_one "${d}"
done

echo
if [[ ${#FAILED_BUILD[@]} -eq 0 ]]; then
  echo "[OK] All builds finished successfully."
else
  echo "[DONE] Finished with build failures (${#FAILED_BUILD[@]})"
  printf ' - %s\n' "${FAILED_BUILD[@]}"
  exit 2
fi
