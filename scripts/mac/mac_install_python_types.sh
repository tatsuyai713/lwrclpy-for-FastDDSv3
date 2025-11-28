#!/usr/bin/env bash
# install_python_types.sh (macOS/BSD-friendly)
# Purpose:
#   Install SWIG-generated Fast DDS Python bindings into a ROS-like package layout
#   so users can `from <pkg>.<ns> import <Type>` (e.g., `from std_msgs.msg import String`).

set -euo pipefail

# ========= Settings =========
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
: "${BUILD_ROOT:=}"                               # e.g., /path/to/repo/._types_python_build_v3
: "${INSTALL_ROOT:=/opt/fast-dds-v3-libs/python/src}"  # Where Python pkgs will be installed

log(){ echo "$@" >&2; }
need(){ command -v "$1" >/dev/null 2>&1 || { log "[FATAL] '$1' not found"; exit 1; }; }
need python3

# Use BSD 'install' if present (macOS has /usr/bin/install)
: "${INSTALL_BIN:=$(command -v install || true)}"
[[ -n "${INSTALL_BIN}" ]] || { log "[FATAL] 'install' command not found"; exit 1; }

mkdir -p "${INSTALL_ROOT}"

# ========= BUILD_ROOT auto-detection =========
if [[ -z "${BUILD_ROOT}" ]]; then
  for d in \
    "${ROOT_DIR}/._types_python_build_v3" \
    "$(cd "${ROOT_DIR}/.." && pwd)/._types_python_build_v3" \
  ; do
    [[ -d "$d/src" ]] && BUILD_ROOT="$d" && break
  done
fi
if [[ -z "${BUILD_ROOT}" || ! -d "${BUILD_ROOT}/src" ]]; then
  log "[FATAL] BUILD_ROOT not found. Example:"
  log "  BUILD_ROOT=/path/to/repo/._types_python_build_v3 \\"
  log "    INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \\"
  log "    bash install_python_types.sh"
  exit 1
fi
log "[INFO] BUILD_ROOT=${BUILD_ROOT}"
log "[INFO] INSTALL_ROOT=${INSTALL_ROOT}"

# ========= Helpers =========
ensure_init(){
  local d="$1"
  [[ -d "$d" ]] || mkdir -p "$d"
  [[ -f "$d/__init__.py" ]] || echo '# auto-generated' > "$d/__init__.py"
}

has_file(){ # has_file DIR GLOB
  local dir="$1" pat="$2"
  (shopt -s nullglob; set +e; for _f in "$dir"/$pat; do return 0; done; return 1)
}

# Install a single generated type directory
install_one(){
  # Example: .../_types_python_build_v3/src/std_msgs/msg/String
  local type_dir="$1"
  local rel="${type_dir#${BUILD_ROOT}/src/}"     # std_msgs/msg/String
  local pkg_dir="$(dirname "${rel}")"            # std_msgs/msg
  local name="$(basename "${rel}")"              # String
  local bld="${type_dir}/build"

  # Locate SWIG outputs
  local py_src="" so_wrapper="" so_core=""
  [[ -f "${bld}/${name}.py" ]] && py_src="${bld}/${name}.py"

  # _NameWrapper.* (extension module) — prefer .so, but accept anything
  # macOS でも Python 拡張は通常 .so
  so_wrapper="$(find "${bld}" -maxdepth 1 -type f -name "_${name}Wrapper.*" | sort | head -n1 || true)"

  # The core shared lib produced by CMake (either .so or .dylib)
  if [[ -f "${bld}/lib${name}.so" ]]; then
    so_core="${bld}/lib${name}.so"
  elif [[ -f "${bld}/lib${name}.dylib" ]]; then
    so_core="${bld}/lib${name}.dylib"
  fi

  if [[ -z "${py_src}" || -z "${so_wrapper}" ]]; then
    log "[SKIP] ${rel}: missing SWIG outputs (py='${py_src:-}' wrapper='${so_wrapper:-}')"
    return 0
  fi

  local dst_pkg="${INSTALL_ROOT}/${pkg_dir}"
  ensure_init "${INSTALL_ROOT}/$(dirname "${pkg_dir}")"   # e.g., std_msgs
  ensure_init "${dst_pkg}"                                # e.g., std_msgs/msg

  log "[INST] ${pkg_dir}/${name}"
  "${INSTALL_BIN}" -m 0644 "${py_src}" "${dst_pkg}/${name}.py"
  local wrapper_basename wrapper_target
  wrapper_basename="$(basename "${so_wrapper}")"
  wrapper_target="${wrapper_basename}"
  if [[ "${wrapper_target}" == *.dylib ]]; then
    # Python extension modules on macOS must end with .so; rename the SWIG output.
    wrapper_target="${wrapper_target%.dylib}.so"
  fi
  "${INSTALL_BIN}" -m 0755 "${so_wrapper}" "${dst_pkg}/${wrapper_target}"
  if [[ "${wrapper_target}" != "${wrapper_basename}" && -e "${dst_pkg}/${wrapper_basename}" ]]; then
    rm -f "${dst_pkg}/${wrapper_basename}"
  fi
  [[ -n "${so_core}" ]] && "${INSTALL_BIN}" -m 0755 "${so_core}" "${dst_pkg}/$(basename "${so_core}")"

  # Re-export the class at package level for ROS-like import:
  #   from .String import String as String
  local init="${dst_pkg}/__init__.py"
  grep -q "from .${name} import ${name} as ${name}" "${init}" 2>/dev/null || \
    echo "from .${name} import ${name} as ${name}" >> "${init}"
}

# ========= Scan and install all generated types =========
# depth=3 (<pkg>/<ns>/<Type>) ディレクトリのみ抽出
TYPE_DIRS_FILE="$(mktemp)"
find "${BUILD_ROOT}/src" -mindepth 3 -maxdepth 3 -type d | while IFS= read -r d; do
  rel="${d#${BUILD_ROOT}/src/}"
  [[ "$rel" == "$d" ]] && continue
  IFS='/' read -r pkg ns type extra <<<"${rel}"
  if [[ -n "${pkg}" && -n "${ns}" && -n "${type}" && -z "${extra:-}" ]]; then
    case "${ns}" in
      msg|srv|action)
        if compgen -G "${d}"/*.i >/dev/null || [[ -f "${d}/CMakeLists.txt" ]]; then
          echo "${d}" >> "${TYPE_DIRS_FILE}"
        fi
        ;;
    esac
  fi
done

# 重複排除
TYPE_DIRS_SORTED="$(mktemp)"
sort -u "${TYPE_DIRS_FILE}" > "${TYPE_DIRS_SORTED}"
rm -f "${TYPE_DIRS_FILE}"

if ! [[ -s "${TYPE_DIRS_SORTED}" ]]; then
  log "[FATAL] No generated type dirs found: ${BUILD_ROOT}/src/*/(msg|srv|action)/*"
  exit 1
fi

while IFS= read -r d; do
  install_one "${d}"
done < "${TYPE_DIRS_SORTED}"
rm -f "${TYPE_DIRS_SORTED}"

cat <<'EOF'

[OK] Installation finished.

For macOS shells, add once (DYLD is used on macOS):
  export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH
  export DYLD_LIBRARY_PATH=/opt/fast-dds-v3-libs/python/src:$DYLD_LIBRARY_PATH

(On Linux, use LD_LIBRARY_PATH instead of DYLD_LIBRARY_PATH.)

Sanity check:
  python3 - <<'PY'
from std_msgs.msg import String
s = String()
print('OK:', type(s))
PY
EOF
