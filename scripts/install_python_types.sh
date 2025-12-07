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
VENDOR_LIB="${INSTALL_ROOT}/_vendor/lib"

log(){ echo "$@" >&2; }
need(){ command -v "$1" >/dev/null 2>&1 || { log "[FATAL] '$1' not found"; exit 1; }; }
need python3
mkdir -p "${INSTALL_ROOT}"

# ========= Stage Fast DDS runtime libs (best-effort) =========
stage_runtime_libs(){
  local dst="${VENDOR_LIB}"
  mkdir -p "${dst}"
  # Always copy directly from fixed Fast DDS install (no search ambiguity)
  for f in /opt/fast-dds-v3/lib/libfastdds.so* /opt/fast-dds-v3/lib/libfastcdr.so* /opt/fast-dds-v3/lib/libfastrtps.so*; do
    [[ -f "$f" ]] || continue
    bn="$(basename "$f")"
    log "[STAGE] runtime lib ${bn} from /opt/fast-dds-v3/lib"
    /bin/cp -pL "$f" "${dst}/${bn}"
  done
}
stage_runtime_libs

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
  
  # Patch Python file to preload lib*.so before importing wrapper
  if [[ -n "${so_core}" ]]; then
    local tmp_py="${dst_pkg}/.${name}.py.tmp"
    {
      echo "# Auto-generated preload for lib${name}.so"
      echo "import os, ctypes"
      echo "_lib_path = os.path.join(os.path.dirname(__file__), 'lib${name}.so')"
      echo "if os.path.exists(_lib_path):"
      echo "    try:"
      echo "        ctypes.CDLL(_lib_path, mode=getattr(ctypes, 'RTLD_GLOBAL', os.RTLD_GLOBAL))"
      echo "    except Exception:"
      echo "        pass"
      echo ""
      cat "${py_src}"
    } > "${tmp_py}"
    install -m 0644 "${tmp_py}" "${dst_pkg}/${name}.py"
    rm -f "${tmp_py}"
  else
    install -m 0644 "${py_src}" "${dst_pkg}/${name}.py"
  fi
  
  install -m 0755 "${so_wrapper}" "${dst_pkg}/$(basename "${so_wrapper}")"
  [[ -n "${so_core}" ]] && install -m 0755 "${so_core}" "${dst_pkg}/lib${name}.so"

  # Re-export the class at package level for ROS-like import:
  #   from .String import String as String
  # BUT: skip this for action types - they will be handled by add_all_action_shims()
  if [[ "${pkg_dir}" != */action ]]; then
    local init="${dst_pkg}/__init__.py"
    grep -q "from .${name} import ${name} as ${name}" "${init}" 2>/dev/null || \
      echo "from .${name} import ${name} as ${name}" >> "${init}"
  fi
}

# ========= Scan and install all generated types =========
mapfile -t TYPE_DIRS < <(find "${BUILD_ROOT}/src" -mindepth 3 -maxdepth 3 -type d -regex '.*/\(msg\|srv\|action\)/[^/]+$' | sort)
[[ ${#TYPE_DIRS[@]} -gt 0 ]] || { log "[FATAL] No generated type dirs found: ${BUILD_ROOT}/src/*/(msg|srv|action)/*"; exit 1; }

for d in "${TYPE_DIRS[@]}"; do
  install_one "${d}"
done

# ========= Add shims for services missing top-level class (Request/Response pairing) =========
add_all_service_shims(){
  # Find all srv dirs and add shims for pairs *_Request.py / *_Response.py if top-level is missing.
  while IFS= read -r srvdir; do
    # build set of request/response stems
    mapfile -t reqs < <(find "${srvdir}" -maxdepth 1 -type f -name "*_Request.py" -printf "%f\n" 2>/dev/null | sed 's/_Request\.py$//')
    for stem in "${reqs[@]}"; do
      local local_base req res init
      local_base="${stem}"
      req="${stem}_Request"
      res="${stem}_Response"
      # skip if either side missing
      [[ -f "${srvdir}/${req}.py" && -f "${srvdir}/${res}.py" ]] || continue
      # skip if already exists
      [[ -f "${srvdir}/${local_base}.py" ]] && continue
      log "[SHIM] ${srvdir}/${local_base}.py"
      cat > "${srvdir}/${local_base}.py" <<PY
from .${req} import ${req} as Request
from .${res} import ${res} as Response

class ${local_base}:
    Request = Request
    Response = Response
PY
      local init="${srvdir}/__init__.py"
      grep -q "from .${local_base} import ${local_base} as ${local_base}" "${init}" 2>/dev/null || \
        echo "from .${local_base} import ${local_base} as ${local_base}" >> "${init}"
    done
  done < <(find "${INSTALL_ROOT}" -type d -path "*/srv")
}

add_all_service_shims

# ========= Add shims for actions (Goal/Result/Feedback/SendGoal/GetResult/FeedbackMessage) =========
add_all_action_shims(){
  # Find all action dirs and create ROS 2-style action class wrappers
  while IFS= read -r actiondir; do
    # List all *_Goal.py files to find action names
    mapfile -t goals < <(find "${actiondir}" -maxdepth 1 -type f -name "*_Goal.py" -printf "%f\n" 2>/dev/null | sed 's/_Goal\.py$//')
    for action_name in "${goals[@]}"; do
      # Check required components exist
      local goal="${action_name}_Goal"
      local result="${action_name}_Result"
      local feedback="${action_name}_Feedback"
      local sendgoal_req="${action_name}_SendGoal_Request"
      local sendgoal_res="${action_name}_SendGoal_Response"
      local getresult_req="${action_name}_GetResult_Request"
      local getresult_res="${action_name}_GetResult_Response"
      local feedback_msg="${action_name}_FeedbackMessage"
      
      # Verify all required files exist
      [[ -f "${actiondir}/${goal}.py" ]] || continue
      [[ -f "${actiondir}/${result}.py" ]] || continue
      [[ -f "${actiondir}/${feedback}.py" ]] || continue
      
      # Skip if wrapper already exists
      [[ -f "${actiondir}/${action_name}.py" ]] && continue
      
      log "[ACTION SHIM] ${actiondir}/${action_name}.py"
      cat > "${actiondir}/${action_name}.py" <<PY
# Auto-generated ROS 2-style action wrapper for ${action_name}
from .${goal} import ${goal} as Goal
from .${result} import ${result} as Result
from .${feedback} import ${feedback} as Feedback

# Import SendGoal/GetResult/FeedbackMessage if they exist
try:
    from .${sendgoal_req} import ${sendgoal_req} as _SendGoal_Request
    from .${sendgoal_res} import ${sendgoal_res} as _SendGoal_Response
    SendGoal = type('SendGoal', (), {'Request': _SendGoal_Request, 'Response': _SendGoal_Response})
except ImportError:
    SendGoal = None

try:
    from .${getresult_req} import ${getresult_req} as _GetResult_Request
    from .${getresult_res} import ${getresult_res} as _GetResult_Response
    GetResult = type('GetResult', (), {'Request': _GetResult_Request, 'Response': _GetResult_Response})
except ImportError:
    GetResult = None

try:
    from .${feedback_msg} import ${feedback_msg} as FeedbackMessage
except ImportError:
    FeedbackMessage = None

class ${action_name}:
    """ROS 2-style action type for ${action_name}."""
    Goal = Goal
    Result = Result
    Feedback = Feedback
    SendGoal = SendGoal
    GetResult = GetResult
    FeedbackMessage = FeedbackMessage
PY
      
      # Add to __init__.py
      local init="${actiondir}/__init__.py"
      grep -q "from .${action_name} import ${action_name} as ${action_name}" "${init}" 2>/dev/null || \
        echo "from .${action_name} import ${action_name} as ${action_name}" >> "${init}"
    done
  done < <(find "${INSTALL_ROOT}" -type d -path "*/action")
}

add_all_action_shims

# ===== Ensure dependent lib*.so are colocated with wrappers =====
# Build index: basename -> full path
LIB_INDEX="$(mktemp)"
find "${INSTALL_ROOT}" -type f -name "lib*.so*" -print | while IFS= read -r f; do
  bn="$(basename "$f")"
  echo "${bn}|${f}"
done | sort -u > "${LIB_INDEX}"
if ! grep -q "libfastdds.so" "${LIB_INDEX}"; then
  log "[WARN] libfastdds.so not found during staging; wrappers may still fail to load"
fi

# ===== Patch RPATH to point at vendor libs (avoid per-msg copies) =====
PATCHED_RPATH="\$ORIGIN:\$ORIGIN/../../lwrclpy/_vendor/lib"
patch_rpath_dir() {
  local dir="$1"
  if ! command -v patchelf >/dev/null 2>&1; then
    log "[WARN] patchelf not found; cannot patch RPATH in ${dir}"
    return
  fi
  while IFS= read -r so; do
    # Skip core vendor libs
    [[ "${so}" == *"/_vendor/lib/"* ]] && continue
    case "$(basename "$so")" in
      libfastdds.so*|libfastcdr.so*|libfastrtps.so*) continue ;;
    esac
    log "[RPATH] ${so}"
    patchelf --force-rpath --set-rpath "${PATCHED_RPATH}" "${so}" 2>/dev/null || \
      log "[WARN] patchelf failed for ${so}"
  done < <(find "${dir}" -maxdepth 1 -type f \( -name "*.so" -o -name "*.so.*" \))
}

while IFS= read -r d; do
  patch_rpath_dir "${d}"
done < <(find "${INSTALL_ROOT}" -type d \( -path "*/msg" -o -path "*/srv" -o -path "*/action" \))

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
