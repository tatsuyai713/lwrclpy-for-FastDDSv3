#!/usr/bin/env bash
# gen_python_types.sh — Fast DDS v3 + fastddsgen 4.x (macOS/BSD friendly with reproducibility patches)
#  - Normalize long/unsigned long types to fixed width (int64_t/uint64_t)
#  - Unify iterator/__wrap_iter in SWIG *_wrap.cxx and inject <cstdint>/<cstddef>
#  - Disable duplicate SWIG helpers via #if 0 … #endif blocks
#  - Auto-inject missing SWIG helpers (size_t/ptrdiff_t) in plain C
#  - Remove duplicate 'static' from SWIGINTERN / SWIGINTERNINLINE
#  - Build flow: fastddsgen → SWIG-only build → wrap normalization → full build (retry with -j1 on failure)

set -euo pipefail

# ===== Ensure build dependencies =====
echo "[INFO] Ensuring build dependencies..."
python3 -m pip install --upgrade pip setuptools wheel || true

# ===== User-tunable =====
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"
FASTDDSGEN_BIN="${FASTDDSGEN_BIN:-/opt/fast-dds-gen-v3/bin/fastddsgen}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_TYPES_ROOT="${ROS_TYPES_ROOT:-${ROOT_DIR}/third_party/ros-data-types-for-fastdds}"

BUILD_ROOT="${BUILD_ROOT:-${ROOT_DIR}/._types_python_build_v3}"
GEN_SRC_ROOT="${BUILD_ROOT}/src"
INC_STAGE_ROOT="${BUILD_ROOT}/include/src"

PATCH_PY="${PATCH_PY:-${ROOT_DIR}/scripts/patch_fastdds_swig_v3.py}"   # optional (applied if present)

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

LONG_SCALAR_RULES = [
    (r'\bunsigned\s+long\s+long\b', 'uint64_t', 0),
    (r'\bunsigned\s+long\b',        'uint64_t', 0),
    (r'\blong\s+long\b',            'int64_t',  0),
    (r'(?<![A-Za-z_])long(?![A-Za-z_])', 'int64_t', 0),
    (r'\bunsigned\s+int64_t\b',     'uint64_t', 0),
    (r'\bsigned\s+int64_t\b',       'int64_t',  0),
    (r'\bunsigned\s+uint64_t\b',    'uint64_t', 0),
    (r'\bsigned\s+uint64_t\b',      'uint64_t', 0),
]

def replace_long_scalars(buf: str) -> str:
    return sub_many(buf, LONG_SCALAR_RULES)

def ensure_header_includes(buf: str) -> str:
    # Inject <cstdint>/<cstddef> immediately after the first #include
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
    # Avoid duplicate 'static': drop static from SWIGINTERN and reduce SWIGINTERNINLINE to SWIGINLINE
    buf = re.sub(r'(^\s*#\s*define\s+SWIGINTERN\s+)static(\s+SWIGUNUSED\s*$)', r'\1\2', buf, flags=re.M)
    buf = re.sub(r'(^\s*#\s*define\s+SWIGINTERNINLINE\s+)SWIGINTERN\s+SWIGINLINE\s*$', r'\1SWIGINLINE', buf, flags=re.M)
    return buf

def strip_long_alias_macros(buf: str) -> str:
    # SWIG 4.4 (macOS) aliases SWIG_From_int64_t etc. to PyLong_* and clashes with injected helpers.
    # Remove those aliases before appending custom helpers.
    targets = [
        r'SWIG_AsVal_int64_t',
        r'SWIG_From_int64_t',
        r'SWIG_AsVal_uint64_t',
        r'SWIG_From_uint64_t',
    ]
    for name in targets:
        pattern = rf'^\s*#\s*define\s+{name}\b[^\n]*\n'
        buf = re.sub(pattern, '\n', buf, flags=re.M)
    return buf

def fix_uint64_printf(buf: str) -> str:
    # Normalize printf formatting (macOS clang -Wformat prefers %llu for uint64_t)
    replacements = {
        '"attempt to assign sequence of size %lu to extended slice of size %lu"':
            '"attempt to assign sequence of size %llu to extended slice of size %llu"',
        'SWIG_InitializeModule: size %lu':
            'SWIG_InitializeModule: size %llu',
        'SWIG_InitializeModule: type %lu':
            'SWIG_InitializeModule: type %llu',
    }
    for old, new in replacements.items():
        buf = buf.replace(old, new)
    return buf

HELPER_DECL_LINE_RE = re.compile(r'^\s*(?:SWIGINTERN(?:INLINE)?|static(?:\s+inline)?)\b')
HELPER_INLINE_DEF_RE = re.compile(
    r'^\s*(?:SWIGINTERN(?:INLINE)?|static(?:\s+inline)?)\b[^\n]*?\b'
    r'(SWIG_(?:AsVal|From)_(?:u?int64_t))\b'
)
HELPER_FUNC_NAME_RE = re.compile(r'^\s*(SWIG_(?:AsVal|From)_(?:u?int64_t))\s*\(')

def detect_helper_def(prev_line: str, line: str):
    m_inline = HELPER_INLINE_DEF_RE.match(line)
    if m_inline:
        return m_inline.group(1)
    m_name = HELPER_FUNC_NAME_RE.match(line)
    if m_name and HELPER_DECL_LINE_RE.match(prev_line):
        return m_name.group(1)
    return None

def helper_defs_from_lines(block_lines):
    names = set()
    for idx, line in enumerate(block_lines):
        prev = block_lines[idx - 1] if idx > 0 else ''
        name = detect_helper_def(prev, line)
        if name:
            names.add(name)
    return names

def disable_longlong_dups(buf: str) -> str:
    # Disable duplicate helper blocks under SWIG_LONG_LONG_AVAILABLE entirely
    lines = buf.splitlines()
    out = []
    defined = set()
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith('#if') and 'SWIG_LONG_LONG_AVAILABLE' in stripped:
            depth = 1
            block_lines = [line]
            i += 1
            while i < n and depth > 0:
                current = lines[i]
                cur_strip = current.strip()
                block_lines.append(current)
                if cur_strip.startswith('#if'):
                    depth += 1
                if cur_strip.startswith('#endif'):
                    depth -= 1
                i += 1
            block_helpers = helper_defs_from_lines(block_lines)
            if block_helpers & defined:
                continue  # drop duplicate helper block entirely
            defined.update(block_helpers)
            out.extend(block_lines)
            continue
        else:
            out.append(line)
            prev = lines[i - 1] if i > 0 else ''
            name = detect_helper_def(prev, line)
            if name:
                defined.add(name)
            i += 1
    return "\n".join(out)

def normalize_vector_templates(buf: str) -> str:
    target = "std::vector"
    parts = []
    i = 0
    n = len(buf)
    while True:
        idx = buf.find(target, i)
        if idx == -1:
            parts.append(buf[i:])
            break
        parts.append(buf[i:idx])
        j = idx + len(target)
        while j < n and buf[j].isspace():
            j += 1
        if j >= n or buf[j] != '<':
            parts.append(buf[idx:j])
            i = j
            continue
        head = buf[idx:j]  # includes std::vector and whitespace
        depth = 1
        k = j + 1
        while k < n and depth > 0:
            ch = buf[k]
            if ch == '<':
                depth += 1
            elif ch == '>':
                depth -= 1
            k += 1
        if depth != 0:
            parts.append(buf[idx:])
            break
        inner = buf[j + 1:k - 1]
        parts.append(f"{head}<{replace_long_scalars(inner)}>")
        i = k
    return ''.join(parts)

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
    # Append helper if referenced but not defined
    ref = re.search(rf'\b{name}\s*\(', buf) is not None
    defd = re.search(rf'^\s*(?:static\s+inline|static|SWIGINTERNINLINE|SWIGINTERN)\s+[A-Za-z_][A-Za-z_0-9:\s\*]*\b{name}\s*\(', buf, re.M) is not None
    if ref and not defd:
        if code not in buf:
            buf += "\n\n" + code + "\n"
    return buf

def normalize_text(buf: str, is_wrap: bool) -> str:
    # (0) Normalize mangled symbol names up front
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

    # (1) Rewrite STL templates / iterators
    buf = sub_many(buf, [
        (r'std::array<\s*long(?:\s+long)?\s*,',          r'std::array<int64_t,', 0),
        (r'std::array<\s*unsigned\s+long(?:\s+long)?\s*,',r'std::array<uint64_t,',0),
        (r'std::vector<\s*long(?:\s+long)?(\s*,\s*[^>]+)?>',           r'std::vector<int64_t\1>', 0),
        (r'std::vector<\s*unsigned\s+long(?:\s+long)?(\s*,\s*[^>]+)?>',r'std::vector<uint64_t\1>',0),
        (r'std::vector<\s*long(?:\s+long)?(\s*,\s*[^>]+)?>\s*::',      r'std::vector<int64_t\1>::', 0),
        (r'std::vector<\s*unsigned\s+long(?:\s+long)?(\s*,\s*[^>]+)?>\s*::', r'std::vector<uint64_t\1>::',0),
        (r'std::allocator<\s*long(?:\s+long)?\s*>',       r'std::allocator<int64_t>',  0),
        (r'std::allocator<\s*unsigned\s+long(?:\s+long)?\s*>', r'std::allocator<uint64_t>', 0),
        (r'__wrap_iter<\s*long(?:\s+long)?\s*\*>',        r'__wrap_iter<int64_t *>',  0),
        (r'__wrap_iter<\s*unsigned\s+long(?:\s+long)?\s*\*>', r'__wrap_iter<uint64_t *>', 0),
        (r'(?<!\w)reverse_iterator<\s*long(?:\s+long)?\s*\*>',  r'reverse_iterator<int64_t *>', 0),
        (r'(?<!\w)reverse_iterator<\s*unsigned\s+long(?:\s+long)?\s*\*>', r'reverse_iterator<uint64_t *>',0),
        (r'(?<!\w)iterator<\s*long(?:\s+long)?\s*\*>',          r'iterator<int64_t *>',  0),
        (r'(?<!\w)iterator<\s*unsigned\s+long(?:\s+long)?\s*\*>',r'iterator<uint64_t *>', 0),
    ])

    # (2) Convert remaining long forms to fixed-width types
    buf = replace_long_scalars(buf)
    buf = normalize_vector_templates(buf)
    buf = replace_long_scalars(buf)

    # (3) Rewrite SWIG aliases for long/unsigned long to 64-bit equivalents
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

    buf = strip_long_alias_macros(buf)
    buf = fix_uint64_printf(buf)
    buf = disable_longlong_dups(buf)

    if is_wrap:
        buf = fix_swig_macros(buf)      # fix duplicate static qualifiers
        buf = ensure_header_includes(buf)
        # Add missing helpers
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
    echo "[FATAL] Residual long/long long usage found: ${tree}" >&2
    grep -RsnE --include='*.i' --include='*.hpp' --include='*.h' --include='*.hh' --include='*.hxx' --include='*.cxx' --include='*_wrap.cxx' "$pat" "$tree" | sed 's/^/  /' >&2 || true
    exit 1
  fi
}

print_log_excerpt() {
  local log_file="$1"
  local label="$2"
  local mode="${3:-first}"
  local match_expr='error|failed|fatal|undefined reference'
  if [[ ! -s "$log_file" ]]; then
    echo "[LOG] ${label}: log missing (${log_file})"
    return
  fi
  local err_line=""
  if command -v rg >/dev/null 2>&1; then
    if [[ "$mode" == "last" ]]; then
      err_line="$( (rg -n -i "(${match_expr})" "${log_file}" || true) | tail -n1 | cut -d: -f1 )"
    else
      err_line="$( (rg -n -i "(${match_expr})" -m1 "${log_file}" || true) | cut -d: -f1 )"
    fi
  else
    if [[ "$mode" == "last" ]]; then
      err_line="$( (grep -n -i -E "${match_expr}" "${log_file}" || true) | tail -n1 | cut -d: -f1 )"
    else
      err_line="$( (grep -n -i -m1 -E "${match_expr}" "${log_file}" || true) | cut -d: -f1 )"
    fi
  fi
  local start end
  if [[ -n "${err_line}" ]]; then
    start=$(( err_line > 40 ? err_line - 40 : 1 ))
    end=$(( err_line + 60 ))
  else
    local total
    total="$(wc -l < "${log_file}")"
    start=$(( total > 200 ? total - 200 : 1 ))
    end="${total}"
  fi
  echo "[LOG] ${label} (${log_file}#${start}-${end})"
  sed -n "${start},${end}p" "${log_file}" | sed 's/^/  | /'
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
    cat "${outdir}/_gen.log" || true
    echo "[FATAL] fastddsgen command failed, exiting immediately"
    exit 1
  fi
  
  # fastddsgen v4.0.4+ recreates the full source path structure inside the output directory
  # Move generated files from nested structure to the expected flat location
  local nested_outdir="${outdir}/${GEN_SRC_ROOT}/${dir_rel}"
  if [[ -d "${nested_outdir}" ]]; then
    echo "[DEBUG] Flattening nested output from ${nested_outdir}"
    # Move all generated files to the top level
    find "${nested_outdir}" -type f \( -name "*.i" -o -name "*.hpp" -o -name "*.cxx" -o -name "*.h" -o -name "*PubSubTypes.*" -o -name "*TypeObjectSupport.*" \) -exec mv {} "${outdir}/" \;
    # Clean up nested structure
    rm -rf "${outdir}/$(echo ${GEN_SRC_ROOT} | cut -d/ -f1)"
  fi
  
  if [[ ! -f "${outdir}/${base}.i" ]]; then
    echo "[ERR]  ${base}.i not generated (fastddsgen succeeded but no .i file)"
    echo "[ERR]  Log contents:"
    cat "${outdir}/_gen.log" || true
    echo "[FATAL] .i file generation failed, exiting immediately"
    exit 1
  fi

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
    local build_log="${PWD}/_cmake_build.log"
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

    # Build every available SWIG target first, normalize wraps, then do full build
    local -a SWIG_BASES=()
    while IFS= read -r i_path; do
      SWIG_BASES+=("$i_path")
    done < <(find .. -maxdepth 1 -type f -name "*.i" -print | sort)
    if [[ ${#SWIG_BASES[@]} -gt 0 ]]; then
      echo "[INFO] Pre-building SWIG wrappers (${#SWIG_BASES[@]})"
      for i_path in "${SWIG_BASES[@]}"; do
        local base; base="$(basename "${i_path}" .i)"
        [[ -z "$base" ]] && continue
        local tgt="${base}Wrapper_swig_compilation"
        echo "  [SWIG] ${tgt}"
        cmake --build . --target "${tgt}" > "_cmake_swig_${base}.log" 2>&1 || true
      done
      normalize_tree_py "$(pwd)"
      assert_no_long_containers "$(pwd)"
    fi

    if ! cmake --build . -j "${JOBS}" > "${build_log}" 2>&1; then
      normalize_tree_py "$(pwd)"
      assert_no_long_containers "$(pwd)"
      echo "[ERR]  cmake build failed: ${outdir#${BUILD_ROOT}/} (log: ${build_log}) — retrying -j1 after normalize"
      print_log_excerpt "${build_log}" "cmake build failure (-j ${JOBS})" "first"
      cmake --build . -j 1 >> "${build_log}" 2>&1 || {
        print_log_excerpt "${build_log}" "cmake build failure after -j1 retry" "last"
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
