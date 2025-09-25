#!/usr/bin/env bash
# gen_python_types.sh (Fast DDS v3 + fastddsgen 4.x)
# 1) IDL ツリーを BUILD_ROOT/src に構成維持コピー
# 2) 各 IDL を「同じディレクトリ直下の <TypeName>/」へ -python 生成（構成維持）
# 3) 生成ディレクトリごとに .i を v3 用パッチ（重複回避）
# 4) CMakeLists に include ディレクトリと SWIG -I を必ず追加
# 5) 各ディレクトリを個別ビルド

set -euo pipefail

# ===== ユーザ調整可 =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"
FASTDDSGEN_BIN="${FASTDDSGEN_BIN:-/opt/fast-dds-gen/bin/fastddsgen}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_TYPES_ROOT="${ROS_TYPES_ROOT:-${ROOT_DIR}/third_party/ros-data-types-for-fastdds}"

BUILD_ROOT="${BUILD_ROOT:-${ROOT_DIR}/._types_python_build_v3}"
GEN_SRC_ROOT="${BUILD_ROOT}/src"           # ← 元の src/ をミラー
INC_STAGE_ROOT="${BUILD_ROOT}/include/src" # ← .i/.hpp の参照をこのルートで解決
PATCH_PY="${PATCH_PY:-${SCRIPT_DIR}/patch_fastdds_swig_v3.py}"

detect_cores(){ command -v nproc >/dev/null && nproc || (command -v sysctl >/dev/null && sysctl -n hw.ncpu) || echo 4; }
JOBS="${JOBS:-$(detect_cores)}"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "[FATAL] '$1' not found" >&2; exit 1; }; }

# ===== 前提チェック =====
need cmake
need swig
need python3
need java
[[ -x "${FASTDDSGEN_BIN}" ]] || { echo "[FATAL] fastddsgen not found: ${FASTDDSGEN_BIN}"; exit 1; }
[[ -d "${ROS_TYPES_ROOT}/src" ]] || { echo "[FATAL] ROS_TYPES_ROOT/src not found: ${ROS_TYPES_ROOT}/src"; exit 1; }
[[ -f "${PATCH_PY}" ]] || { echo "[FATAL] patch script missing: ${PATCH_PY}"; exit 1; }

mkdir -p "${BUILD_ROOT}" "${GEN_SRC_ROOT}" "${INC_STAGE_ROOT}"

# ===== fastdds / fastcdr CMake Config の自動検出 =====
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

# ===== IDL ツリーを構成維持でコピー =====
echo "[INFO] Mirroring IDL tree → ${GEN_SRC_ROOT}"
rsync -a --delete "${ROS_TYPES_ROOT}/src/" "${GEN_SRC_ROOT}/"

# 予約語 FIXED 対応（gazebo_msgs のみ、コピー側だけ）
mapfile -t GZ_IDLS < <(find "${GEN_SRC_ROOT}/gazebo_msgs" -type f -name "*.idl" 2>/dev/null || true)
for f in "${GZ_IDLS[@]}"; do
  sed -i -E 's/(^|[^A-Za-z0-9_])FIXED([^A-Za-z0-9_])/\1_FIXED\2/g' "$f" || true
done

# ===== IDL 一覧 =====
if [[ -n "${FILTER:-}" ]]; then
  mapfile -t IDLS < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" | sed "s|^${GEN_SRC_ROOT}/||" | grep -i "${FILTER}" | sort)
else
  mapfile -t IDLS < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" | sed "s|^${GEN_SRC_ROOT}/||" | sort)
fi
[[ ${#IDLS[@]} -gt 0 ]] || { echo "[ERR] No IDL files (FILTER='${FILTER:-}')"; exit 1; }

# ===== CMakeLists に include を必ず通す =====
patch_cmakelists_min() {
  local dir="$1"      # .../src/<pkg>/<ns>/<Type>
  local dir_rel="$2"  # <pkg>/<ns>
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

# ===== 1件生成 → パッチ → ステージング =====
FAILED_GEN=()

gen_one() {
  local rel="$1"                               # 例) builtin_interfaces/msg/Time.idl
  local idl_path="${GEN_SRC_ROOT}/${rel}"
  local dir_rel="$(dirname "${rel}")"          # builtin_interfaces/msg
  local base="$(basename "${rel}" .idl)"       # Time
  local outdir="${GEN_SRC_ROOT}/${dir_rel}/${base}"  # src/builtin_interfaces/msg/Time/…

  rm -rf "${outdir}"
  mkdir -p "${outdir}"

  echo "[GEN] ${rel} -> ${outdir}"
  if ! "${FASTDDSGEN_BIN}" -python -cs -typeros2 \
        -d "${outdir}" \
        -I "${GEN_SRC_ROOT}" \
        -replace "${idl_path}" > "${outdir}/_gen.log" 2>&1; then
    echo "[ERR]  fastddsgen failed: ${rel}（ログ: ${outdir}/_gen.log 先頭）"
    sed -n '1,120p' "${outdir}/_gen.log" || true
    FAILED_GEN+=("${rel}")
    return
  fi
  [[ -f "${outdir}/${base}.i" ]] || { echo "[ERR]  ${base}.i not generated"; FAILED_GEN+=("${rel}"); return; }

  echo "[PATCH.i] ${rel}"
  python3 "${PATCH_PY}" "${outdir}/${base}.i"

  # 参照解決用に .i/.hpp を構成維持でステージ
  local inc_dst_dir="${INC_STAGE_ROOT}/${dir_rel}"
  mkdir -p "${inc_dst_dir}"
  find "${outdir}" -maxdepth 1 -type f \
    \( -name '*.i' -o -name '*.hpp' -o -name '*TypeObjectSupport.*' -o -name '*PubSubTypes.*' \) \
    -exec cp -f {} "${inc_dst_dir}/" \;

  patch_cmakelists_min "${outdir}" "${dir_rel}"
}

for rel in "${IDLS[@]}"; do
  gen_one "${rel}"
done

if [[ ${#FAILED_GEN[@]} -gt 0 ]]; then
  echo "[WARN] fastddsgen 失敗: ${#FAILED_GEN[@]} 件（生成できた分のみビルド続行）"
  printf ' - %s\n' "${FAILED_GEN[@]}"
fi

# ===== ビルド =====
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

mapfile -t OUTDIRS < <(find "${GEN_SRC_ROOT}" -mindepth 4 -maxdepth 4 -type f -name CMakeLists.txt -printf '%h\n' | sort -u)
for d in "${OUTDIRS[@]}"; do
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