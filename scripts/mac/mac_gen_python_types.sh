#!/usr/bin/env bash
# gen_python_types.sh — Fast DDS v3 + fastddsgen 4.x（macOS/BSD対応・再現性パッチ込み）
#  - long/unsigned long 系→固定幅(int64_t/uint64_t)へ正規化
#  - SWIG *_wrap.cxx の iterator/__wrap_iter なども統一・<cstdint>/<cstddef>注入
#  - SWIG ヘルパ重複は #if 0 … #endif で無効化（安全）
#  - 不足する SWIG ヘルパ（size_t/ptrdiff_t 系）を plain C で自動注入
#  - SWIGINTERN / SWIGINTERNINLINE の重複 static を除去（duplicate 'static' 対策）
#  - 生成→SWIGだけ先ビルド→wrap正規化→本ビルド（失敗時 -j1 リトライ）

set -euo pipefail

# ===== User-tunable =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"
FASTDDSGEN_BIN="${FASTDDSGEN_BIN:-/opt/fast-dds-gen-v3/bin/fastddsgen}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_TYPES_ROOT="${ROS_TYPES_ROOT:-${ROOT_DIR}/third_party/ros-data-types-for-fastdds}"

BUILD_ROOT="${BUILD_ROOT:-${ROOT_DIR}/._types_python_build_v3}"
GEN_SRC_ROOT="${BUILD_ROOT}/src"
INC_STAGE_ROOT="${BUILD_ROOT}/include/src"

PATCH_PY="${PATCH_PY:-${SCRIPT_DIR}/patch_fastdds_swig_v3.py}"   # 任意（存在すれば .i にも適用）

detect_cores(){ if command -v sysctl >/dev/null 2>&1; then sysctl -n hw.ncpu; elif command -v nproc >/dev/null 2>&1; then nproc; else echo 4; fi; }
JOBS="${JOBS:-$(detect_cores)}"
need(){ command -v "$1" >/dev/null 2>&1 || { echo "[FATAL] '$1' not found" >&2; exit 1; }; }

# ===== Prechecks =====
need cmake; need swig; need python3; need java; need rsync
[[ -x "${FASTDDSGEN_BIN}" ]] || { echo "[FATAL] fastddsgen not found: ${FASTDDSGEN_BIN}"; exit 1; }
[[ -d "${ROS_TYPES_ROOT}/src" ]] || { echo "[FATAL] ROS_TYPES_ROOT/src not found: ${ROS_TYPES_ROOT}/src"; exit 1; }

mkdir -p "${BUILD_ROOT}" "${GEN_SRC_ROOT}" "${INC_STAGE_ROOT}"

# ===== CMake package dirs =====
autodetect_pkg_dir() {
  local prefix="$1" pkg="$2"
  local p1="${prefix}/share/${pkg}/cmake/${pkg}-config.cmake"
  local p2="${prefix}/lib/cmake/${pkg}/${pkg}-config.cmake"
  if [[ -f "$p1" ]]; then dirname "$p1"; return 0; fi
  if [[ -f "$p2" ]]; then dirname "$p2"; return 0; fi
  local cand
  cand="$(find "${prefix}" -type f -name "${pkg}-config.cmake" -print -quit 2>/dev/null || true)"
  [[ -n "${cand}" ]] && { dirname "${cand}"; return 0; }
  return 1
}
FASTDDS_CMAKE_DIR="$(autodetect_pkg_dir "${PREFIX_V3}" fastdds)" || { echo "[FATAL] fastdds-config.cmake not found under ${PREFIX_V3}"; exit 1; }
FASTCDR_CMAKE_DIR="$(autodetect_pkg_dir "${PREFIX_V3}" fastcdr)" || { echo "[FATAL] fastcdr-config.cmake not found under ${PREFIX_V3}"; exit 1; }
echo "[INFO] fastdds_DIR=${FASTDDS_CMAKE_DIR}"
echo "[INFO] fastcdr_DIR=${FASTCDR_CMAKE_DIR}"

# ===== Mirror IDL =====
echo "[INFO] Mirroring IDL tree → ${GEN_SRC_ROOT}"
rsync -a --delete "${ROS_TYPES_ROOT}/src/" "${GEN_SRC_ROOT}/"

# Drop upstream CMakeLists
if find "${GEN_SRC_ROOT}" -mindepth 2 -maxdepth 2 -type f -name CMakeLists.txt -print | grep -q .; then
  while IFS= read -r cml; do mv "${cml}" "${cml}.upstream" || true; done \
    < <(find "${GEN_SRC_ROOT}" -mindepth 2 -maxdepth 2 -type f -name CMakeLists.txt -print)
fi

# gazebo_msgs: FIXED → _FIXED
if [[ -d "${GEN_SRC_ROOT}/gazebo_msgs" ]]; then
  while IFS= read -r f; do
    python3 - "$f" <<'PY'
import sys,re
p=sys.argv[1]; s=open(p,'r',encoding='utf-8',errors='ignore').read()
s=re.sub(r'(^|[^A-Za-z0-9_])FIXED([^A-Za-z0-9_])',r'\1_FIXED\2',s)
open(p,'w',encoding='utf-8').write(s)
PY
  done < <(find "${GEN_SRC_ROOT}/gazebo_msgs" -type f -name "*.idl" -print 2>/dev/null || true)
fi

# ===== Collect IDLs =====
IDLS=()
if [[ -n "${FILTER:-}" ]]; then
  while IFS= read -r line; do
    if echo "$line" | grep -qi -- "${FILTER}"; then IDLS+=("$line"); fi
  done < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" -print | sed "s|^${GEN_SRC_ROOT}/||" | sort)
else
  while IFS= read -r line; do IDLS+=("$line"); done \
    < <(find "${GEN_SRC_ROOT}" -type f -name "*.idl" -print | sed "s|^${GEN_SRC_ROOT}/||" | sort)
fi
[[ ${#IDLS[@]} -gt 0 ]] || { echo "[ERR] No IDL files (FILTER='${FILTER:-}')"; exit 1; }

# ===== CMakeLists minimal patch =====
patch_cmakelists_min() {
  local outdir="$1" dir_rel="$2" cml="${outdir}/CMakeLists.txt"
  [[ -f "$cml" ]] || return 0
  if ! grep -q "__FASTDDS_INC_STAGE_ADDED__" "$cml"; then
    {
      echo "# __FASTDDS_INC_STAGE_ADDED__"
      echo "set(FASTDDS_GEN_INCLUDE_STAGE \"${INC_STAGE_ROOT}\")"
      echo "include_directories(\"\${FASTDDS_GEN_INCLUDE_STAGE}\")"
      echo "# __FASTDDS_SUBINC_ADDED__"
      echo "include_directories(\"${INC_STAGE_ROOT}/${dir_rel}\")"
    } | cat - "$cml" > "$cml.tmp" && mv "$cml.tmp" "$cml"
  fi
}

# ===== Python: normalize & fix wrap helpers =====
normalize_tree_py() {
python3 - "$@" <<'PY'
import sys, re, pathlib

def sub_many(s, rules):
    for pat, rep, flags in rules:
        s = re.sub(pat, rep, s, flags=flags)
    return s

def comment_dups_with_if0(buf: str, func_pat: str) -> str:
    # 関数定義ブロックを 2個目以降 #if 0 … #endif で囲む
    pat = re.compile(rf'^\s*(?:static\s+inline|static|SWIGINTERNINLINE|SWIGINTERN)\s+[^\n]*\b{func_pat}\s*\([^)]*\)\s*\{{.*?\n\}}', re.DOTALL | re.MULTILINE)
    ms = list(pat.finditer(buf))
    if len(ms) <= 1:
        return buf
    delta = 0
    for m in ms[1:]:
        a, b = m.start()+delta, m.end()+delta
        block = buf[a:b]
        rep = "#if 0 /* __SWIG_DUP_REMOVED__ */\n" + block + "\n#endif /* __SWIG_DUP_REMOVED__ */\n"
        buf = buf[:a] + rep + buf[b:]
        delta += len(rep) - (b - a)
    return buf

def kill_helper_dups(buf: str) -> str:
    targets = [
        r'SWIG_AsVal_uint64_t', r'SWIG_AsVal_int64_t',
        r'SWIG_From_uint64_t',  r'SWIG_From_int64_t',
        r'SWIG_AsVal_size_t',   r'SWIG_From_size_t',
        r'SWIG_AsVal_ptrdiff_t',r'SWIG_From_ptrdiff_t',
    ]
    for t in targets:
        buf = comment_dups_with_if0(buf, t)
    return buf

def ensure_header_includes(buf: str) -> str:
    # 最初の #include の直後に <cstdint>/<cstddef> を注入
    m = re.search(r'^\s*#\s*include[^\n]*\n', buf, re.M)
    if m:
        pos = m.end()
        add = []
        if '<cstdint>' not in buf: add.append('#include <cstdint>\n')
        if '<cstddef>' not in buf: add.append('#include <cstddef>\n')
        if add:
            buf = buf[:pos] + ''.join(add) + buf[pos:]
    return buf

def fix_swig_macros(buf: str) -> str:
    # duplicate 'static' を避けるため SWIGINTERN の static を外す / SWIGINTERNINLINE は SWIGINLINE のみ
    buf = re.sub(r'(^\s*#\s*define\s+SWIGINTERN\s+)static(\s+SWIGUNUSED\s*$)', r'\1\2', buf, flags=re.M)
    buf = re.sub(r'(^\s*#\s*define\s+SWIGINTERNINLINE\s+)SWIGINTERN\s+SWIGINLINE\s*$', r'\1SWIGINLINE', buf, flags=re.M)
    return buf

HELPER_SIZE_T = r"""
/* __SWIG_HELPER_INJECTED__: size_t */
static int SWIG_AsVal_size_t (PyObject *obj, size_t *val) {
  if (PyLong_Check(obj)) {
    unsigned long long v = PyLong_AsUnsignedLongLong(obj);
    if (val) *val = (size_t)v;
    return SWIG_OK;
  }
  return SWIG_TypeError;
}
static inline PyObject * SWIG_From_size_t (size_t value) {
  return PyLong_FromUnsignedLongLong((unsigned long long)value);
}
"""

HELPER_PTRDIFF_T = r"""
/* __SWIG_HELPER_INJECTED__: ptrdiff_t */
static int SWIG_AsVal_ptrdiff_t (PyObject *obj, ptrdiff_t *val) {
  if (PyLong_Check(obj)) {
    long long v = PyLong_AsLongLong(obj);
    if (val) *val = (ptrdiff_t)v;
    return SWIG_OK;
  }
  return SWIG_TypeError;
}
static inline PyObject * SWIG_From_ptrdiff_t (ptrdiff_t value) {
  return PyLong_FromLongLong((long long)value);
}
"""

def append_helper_if_missing(buf: str, name: str, code: str) -> str:
    # 参照あり・定義無しなら末尾へ追加
    ref = re.search(rf'\b{name}\s*\(', buf) is not None
    defd = re.search(rf'^\s*(?:static\s+inline|static|SWIGINTERNINLINE|SWIGINTERN)\s+[A-Za-z_][A-Za-z_0-9:\s\*]*\b{name}\s*\(', buf, re.M) is not None
    if ref and not defd:
        if code not in buf:
            buf += "\n\n" + code + "\n"
    return buf

def normalize_text(buf: str, is_wrap: bool) -> str:
    # (0) マングル名先に正規化
    buf = sub_many(buf, [
        (r'std_vector_Sl_long',                 r'std_vector_Sl_int64_t', 0),
        (r'std_vector_Sl_unsigned_long',        r'std_vector_Sl_uint64_t',0),
        (r'std_array_Sl_long',                  r'std_array_Sl_int64_t',  0),
        (r'std_array_Sl_unsigned_long',         r'std_array_Sl_uint64_t', 0),
        (r'std__allocatorT_long_t',             r'std__allocatorT_int64_t_t', 0),
        (r'std__allocatorT_unsigned_long_t',    r'std__allocatorT_uint64_t_t',0),
        (r'SWIGTYPE_p_std__allocatorT_long_t',          r'SWIGTYPE_p_std__allocatorT_int64_t_t', 0),
        (r'SWIGTYPE_p_std__allocatorT_unsigned_long_t', r'SWIGTYPE_p_std__allocatorT_uint64_t_t',0),
        (r'_Sl_long_Sg_',                       r'_Sl_int64_t_Sg_', 0),
        (r'_Sl_unsigned_long_Sg_',              r'_Sl_uint64_t_Sg_',0),
    ])

    # (1) STLテンプレ/反復子
    buf = sub_many(buf, [
        (r'std::array<\s*long(?:\s+long)?\s*,',          r'std::array<int64_t,', 0),
        (r'std::array<\s*unsigned\s+long(?:\s+long)?\s*,',r'std::array<uint64_t,',0),
        (r'std::vector<\s*long(?:\s+long)?(\s*,\s*[^>]+)?>',           r'std::vector<int64_t\1>', 0),
        (r'std::vector<\s*unsigned\s+long(?:\s+long)?(\s*,\s*[^>]+)?>',r'std::vector<uint64_t\1>',0),
        (r'std::allocator<\s*long(?:\s+long)?\s*>',       r'std::allocator<int64_t>',  0),
        (r'std::allocator<\s*unsigned\s+long(?:\s+long)?\s*>', r'std::allocator<uint64_t>', 0),
        (r'__wrap_iter<\s*long(?:\s+long)?\s*\*>',        r'__wrap_iter<int64_t *>',  0),
        (r'__wrap_iter<\s*unsigned\s+long(?:\s+long)?\s*\*>', r'__wrap_iter<uint64_t *>', 0),
        (r'(?<!\w)reverse_iterator<\s*long(?:\s+long)?\s*\*>',  r'reverse_iterator<int64_t *>', 0),
        (r'(?<!\w)reverse_iterator<\s*unsigned\s+long(?:\s+long)?\s*\*>', r'reverse_iterator<uint64_t *>',0),
        (r'(?<!\w)iterator<\s*long(?:\s+long)?\s*\*>',          r'iterator<int64_t *>',  0),
        (r'(?<!\w)iterator<\s*unsigned\s+long(?:\s+long)?\s*\*>',r'iterator<uint64_t *>', 0),
    ])

    # (2) long 系を固定幅へ
    buf = sub_many(buf, [
        (r'\bunsigned\s+long\s+long\b', 'uint64_t', 0),
        (r'\bunsigned\s+long\b',        'uint64_t', 0),
        (r'\blong\s+long\b',            'int64_t',  0),
        (r'(?<![A-Za-z_])long(?![A-Za-z_])', 'int64_t', 0),
        (r'\bunsigned\s+int64_t\b',     'uint64_t', 0),
        (r'\bsigned\s+int64_t\b',       'int64_t',  0),
        (r'\bunsigned\s+uint64_t\b',    'uint64_t', 0),
        (r'\bsigned\s+uint64_t\b',      'uint64_t', 0),
    ])

    # (3) SWIG の long/unsigned long 別名を 64bit 名へ
    buf = sub_many(buf, [
        (r'SWIG_AsVal_unsigned_SS_long_SS_long', r'SWIG_AsVal_uint64_t', 0),
        (r'SWIG_AsVal_unsigned_SS_long',         r'SWIG_AsVal_uint64_t', 0),
        (r'SWIG_From_unsigned_SS_long_SS_long',  r'SWIG_From_uint64_t',  0),
        (r'SWIG_From_unsigned_SS_long',          r'SWIG_From_uint64_t',  0),
        (r'SWIG_AsVal_long_SS_long',             r'SWIG_AsVal_int64_t',  0),
        (r'SWIG_AsVal_long',                     r'SWIG_AsVal_int64_t',  0),
        (r'SWIG_From_long_SS_long',              r'SWIG_From_int64_t',   0),
        (r'SWIG_From_long',                      r'SWIG_From_int64_t',   0),
    ])

    if is_wrap:
        buf = fix_swig_macros(buf)      # duplicate static 修正
        buf = ensure_header_includes(buf)
        buf = kill_helper_dups(buf)     # 重複定義を #if 0 で無効化
        # 不足ヘルパ補完
        buf = append_helper_if_missing(buf, 'SWIG_AsVal_size_t', HELPER_SIZE_T)
        buf = append_helper_if_missing(buf, 'SWIG_From_size_t',  HELPER_SIZE_T)
        buf = append_helper_if_missing(buf, 'SWIG_AsVal_ptrdiff_t', HELPER_PTRDIFF_T)
        buf = append_helper_if_missing(buf, 'SWIG_From_ptrdiff_t',  HELPER_PTRDIFF_T)

    return buf

def should_process(p: pathlib.Path) -> bool:
    if not p.is_file(): return False
    if p.name.endswith('_wrap.cxx'): return True
    return p.suffix in ('.i','.hpp','.h','.hh','.hxx','.cxx')

def process_file(p: pathlib.Path):
    try:
        s = p.read_text(encoding='utf-8', errors='ignore')
    except Exception:
        return
    out = normalize_text(s, is_wrap=p.name.endswith('_wrap.cxx'))
    if out != s:
        p.write_text(out, encoding='utf-8')

def walk(root: pathlib.Path):
    for p in root.rglob('*'):
        if should_process(p):
            process_file(p)

if __name__ == '__main__':
    for arg in sys.argv[1:]:
        walk(pathlib.Path(arg))
PY
}

assert_no_long_containers() {
  local tree="$1"; [[ -d "$tree" ]] || return 0
  local pat='(std::array<\s*(unsigned\s+)?long(\s+long)?\s*,)|(std::vector<\s*(unsigned\s+)?long(\s+long)?(\s*,\s*[^>]+)?>)|(std::allocator<\s*(unsigned\s+)?long(\s+long)?\s*>)|(__wrap_iter<\s*(unsigned\s+)?long(\s+long)?\s*\*>)|((^|[^A-Za-z_])reverse_iterator<\s*(unsigned\s+)?long(\s+long)?\s*\*>)|((^|[^A-Za-z_])iterator<\s*(unsigned\s+)?long(\s+long)?\s*\*>)'
  if grep -RsnE --include='*.i' --include='*.hpp' --include='*.h' --include='*.hh' --include='*.hxx' --include='*.cxx' --include='*_wrap.cxx' "$pat" "$tree" >/dev/null 2>&1; then
    echo "[FATAL] long/long long の残骸: ${tree}" >&2
    grep -RsnE --include='*.i' --include='*.hpp' --include='*.h' --include='*.hh' --include='*.hxx' --include='*.cxx' --include='*_wrap.cxx' "$pat" "$tree" | sed 's/^/  /' >&2 || true
    exit 1
  fi
}

# ===== Generate → Normalize → Stage =====
FAILED_GEN=()

gen_one() {
  local rel="$1"
  local idl_path="${GEN_SRC_ROOT}/${rel}"
  local dir_rel; dir_rel="$(dirname "${rel}")"
  local base; base="$(basename "${rel}" .idl)"
  local outdir="${GEN_SRC_ROOT}/${dir_rel}/${base}"

  rm -rf "${outdir}"; mkdir -p "${outdir}"

  echo "[GEN] ${rel} -> ${outdir}"
  if ! "${FASTDDSGEN_BIN}" -python -cs -typeros2 -language c++ \
        -d "${outdir}" -I "${GEN_SRC_ROOT}" -replace "${idl_path}" \
        > "${outdir}/_gen.log" 2>&1; then
    echo "[ERR]  fastddsgen failed: ${rel} (see ${outdir}/_gen.log)"
    sed -n '1,160p' "${outdir}/_gen.log" || true
    FAILED_GEN+=("${rel}")
    return
  fi
  [[ -f "${outdir}/${base}.i" ]] || { echo "[ERR]  ${base}.i not generated"; FAILED_GEN+=("${rel}"); return; }

  normalize_tree_py "${outdir}"

  if [[ -f "${PATCH_PY}" ]]; then
    echo "[PATCH.i] ${rel}"
    python3 "${PATCH_PY}" "${outdir}/${base}.i" || true
    normalize_tree_py "${outdir}"
  fi

  local inc_dst_dir="${INC_STAGE_ROOT}/${dir_rel}"
  mkdir -p "${inc_dst_dir}"
  find "${outdir}" -maxdepth 1 -type f \
    \( -name '*.i' -o -name '*.hpp' -o -name '*TypeObjectSupport.*' -o -name '*PubSubTypes.*' \) \
    -exec cp -f {} "${inc_dst_dir}/" \;

  normalize_tree_py "${inc_dst_dir}"

  patch_cmakelists_min "${outdir}" "${dir_rel}"

  assert_no_long_containers "${outdir}"
  assert_no_long_containers "${inc_dst_dir}"
}

for rel in "${IDLS[@]}"; do gen_one "${rel}"; done

if [[ ${#FAILED_GEN[@]} -gt 0 ]]; then
  echo "[WARN] fastddsgen failed on ${#FAILED_GEN[@]} file(s)"
  printf ' - %s\n' "${FAILED_GEN[@]}"
fi

# ===== Build =====
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

    # 先に SWIG だけ回し、wrap を正規化 → その後本ビルド
    local base; base="$(basename "$(ls -1 ../*.i | head -n1)" .i 2>/dev/null || echo '')"
    if [[ -n "$base" ]]; then
      cmake --build . --target "${base}Wrapper_swig_compilation" > _cmake_swig.log 2>&1 || true
      normalize_tree_py "$(pwd)"
    fi

    if ! cmake --build . -j "${JOBS}" > _cmake_build.log 2>&1; then
      normalize_tree_py "$(pwd)"
      echo "[ERR]  cmake build failed: ${outdir#${BUILD_ROOT}/} — retrying -j1 after normalize"
      cmake --build . -j 1 >> _cmake_build.log 2>&1 || {
        sed -n '1,220p' _cmake_build.log || true
        tail -n 120 _cmake_build.log || true
        popd >/dev/null; FAILED_BUILD+=("${outdir} (build)"); return
      }
    fi
  popd >/dev/null
}

OUTDIRS=()
tmp_list="$(mktemp)"
find "${GEN_SRC_ROOT}" -type f -name "*.i" -print | while IFS= read -r p; do dirname "$p"; done | sort -u > "${tmp_list}"
while IFS= read -r d; do OUTDIRS+=("$d"); done < "${tmp_list}"
rm -f "${tmp_list}"

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