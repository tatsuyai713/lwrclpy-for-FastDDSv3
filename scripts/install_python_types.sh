#!/usr/bin/env bash
# install_python_types.sh (v2)
# Purpose:
#   Install SWIG-generated Fast DDS Python bindings into a ROS-like package layout
#   so users can `from <pkg>.<ns> import <Type>` (e.g., `from std_msgs.msg import String`).

set -euo pipefail

# ========= Settings =========
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
: "${BUILD_ROOT:=}"   # e.g., /path/to/repo/._types_python_build_v3
: "${INSTALL_ROOT:=/opt/fast-dds-v3-libs/python/src}"  # Root where Python packages will be installed

log(){ echo "$@" >&2; }
need(){ command -v "$1" >/dev/null 2>&1 || { log "[FATAL] '$1' not found"; exit 1; }; }
need python3
mkdir -p "${INSTALL_ROOT}"

# ========= BUILD_ROOT auto-detection =========
if [[ -z "${BUILD_ROOT}" ]]; then
  # Try a few common locations (repo root, parent of repo root)
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
  # _NameWrapper.* (.so/.pyd)
  mapfile -t _cands < <(find "${bld}" -maxdepth 1 -type f -name "_${name}Wrapper.*" 2>/dev/null | sort)
  [[ ${#_cands[@]} -gt 0 ]] && so_wrapper="${_cands[0]}"
  [[ -f "${bld}/lib${name}.so" ]] && so_core="${bld}/lib${name}.so"

  if [[ -z "${py_src}" || -z "${so_wrapper}" ]]; then
    log "[SKIP] ${rel}: missing SWIG outputs (py='${py_src:-}', wrapper='${so_wrapper:-}')"
    return 0
  fi

  local dst_pkg="${INSTALL_ROOT}/${pkg_dir}"
  ensure_init "${INSTALL_ROOT}/$(dirname "${pkg_dir}")"   # e.g., std_msgs
  ensure_init "${dst_pkg}"                                # e.g., std_msgs/msg

  log "[INST] ${pkg_dir}/${name}"
  install -m 0644 "${py_src}" "${dst_pkg}/${name}.py"
  install -m 0755 "${so_wrapper}" "${dst_pkg}/$(basename "${so_wrapper}")"
  [[ -n "${so_core}" ]] && install -m 0755 "${so_core}" "${dst_pkg}/lib${name}.so"

  # Re-export the class at package level for ROS-like import:
  #   from .String import String as String
  local init="${dst_pkg}/__init__.py"
  grep -q "from .${name} import ${name} as ${name}" "${init}" 2>/dev/null || \
    echo "from .${name} import ${name} as ${name}" >> "${init}"
}

# ========= Scan and install all generated types =========
mapfile -t TYPE_DIRS < <(find "${BUILD_ROOT}/src" -mindepth 3 -maxdepth 3 -type d -regex '.*/\(msg\|srv\|action\)/[^/]+$' | sort)
[[ ${#TYPE_DIRS[@]} -gt 0 ]] || { log "[FATAL] No generated type dirs found: ${BUILD_ROOT}/src/*/(msg|srv|action)/*"; exit 1; }

for d in "${TYPE_DIRS[@]}"; do
  install_one "${d}"
done

cat <<EOF

[OK] Installation finished.

Before running, add these environment variables once:
  export PYTHONPATH=${INSTALL_ROOT}:\$PYTHONPATH
  export LD_LIBRARY_PATH=${INSTALL_ROOT}:\$LD_LIBRARY_PATH

Sanity check:
  python3 - <<'PY'
from std_msgs.msg import String
s = String()   # use as a class directly
print('OK:', type(s))
PY
EOF
