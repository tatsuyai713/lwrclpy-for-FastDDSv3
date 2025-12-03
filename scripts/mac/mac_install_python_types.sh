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

log(){ echo "$@" >&2; }
need(){ command -v "$1" >/dev/null 2>&1 || { log "[FATAL] '$1' not found"; exit 1; }; }
need python3

PY_PURELIB="$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_paths()["platlib"])
PY
)"
: "${INSTALL_ROOT:=${PY_PURELIB}}"  # Default to current Python's site-packages

# Use BSD 'install' if present (macOS has /usr/bin/install)
: "${INSTALL_BIN:=$(command -v install || true)}"
[[ -n "${INSTALL_BIN}" ]] || { log "[FATAL] 'install' command not found"; exit 1; }
: "${INSTALL_NAME_TOOL:=$(command -v install_name_tool || true)}"
: "${OTOOL:=$(command -v otool || true)}"

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
  log "    INSTALL_ROOT=\$(python3 - <<'PY' ; import sysconfig ; print(sysconfig.get_paths()[\"platlib\"]) ; PY) \\"
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

set_rpaths(){
  local target="$1"
  [[ -x "${INSTALL_NAME_TOOL}" && -x "${OTOOL}" ]] || return 0
  local current_rpaths new_rpaths=() rp
  current_rpaths="$(otool -l "${target}" 2>/dev/null | awk '/LC_RPATH/{getline;getline;print $2}')"
  # Always prefer local directory first
  new_rpaths+=("@loader_path")
  while IFS= read -r rp; do
    [[ -z "${rp}" ]] && continue
    # Drop build-tree rpaths
    if [[ "${rp}" == *"${BUILD_ROOT}"* ]]; then
      continue
    fi
    # Avoid duplicates and keep only a small set (fastdds/homebrew if present)
    case "${rp}" in
      @loader_path) continue ;;
      /opt/fast-dds-v3/lib) new_rpaths+=("/opt/fast-dds-v3/lib") ;;
      /opt/homebrew/lib) new_rpaths+=("/opt/homebrew/lib") ;;
    esac
  done <<< "${current_rpaths}"

  # Clear existing and re-add ordered rpaths
  while IFS= read -r rp; do
    [[ -z "${rp}" ]] && continue
    install_name_tool -delete_rpath "${rp}" "${target}" >/dev/null 2>&1 || true
  done <<< "${current_rpaths}"
  for rp in "${new_rpaths[@]}"; do
    install_name_tool -add_rpath "${rp}" "${target}" >/dev/null 2>&1 || true
  done
}

rewrite_dep_paths(){
  local target="$1"
  [[ -x "${INSTALL_NAME_TOOL}" && -x "${OTOOL}" ]] || return 0
  # Replace @rpath/lib*.dylib (except fastdds/fastcdr) with @loader_path/lib*.dylib to keep deps local
  while IFS= read -r line; do
    dep="$(echo "$line" | awk '{print $1}')"
    case "${dep}" in
      @rpath/libfastdds.*|@rpath/libfastcdr.*) continue ;;
      @rpath/lib*.dylib)
        base="$(basename "${dep}")"
        install_name_tool -change "${dep}" "@loader_path/${base}" "${target}" >/dev/null 2>&1 || true
        ;;
    esac
  done < <("${OTOOL}" -L "${target}" 2>/dev/null | tail -n +2)
}

copy_local_deps(){
  local target="$1" dst_dir dep
  dst_dir="$(dirname "${target}")"
  [[ -x "${OTOOL}" ]] || return 0
  while IFS= read -r line; do
    dep="$(echo "$line" | awk '{print $1}')"
    case "${dep}" in
      @loader_path/*) dep="${dst_dir}/${dep#@loader_path/}" ;;
      ${INSTALL_ROOT}/*) ;;
      *) continue ;;
    esac
    local bn="$(basename "${dep}")"
    if [[ ! -e "${dst_dir}/${bn}" ]]; then
      if [[ -f "${dep}" ]]; then
        /bin/cp -p "${dep}" "${dst_dir}/"
      else
        dep="$(find "${INSTALL_ROOT}" -type f -name "${bn}" -print -quit 2>/dev/null || true)"
        if [[ -n "${dep}" ]]; then
          /bin/cp -p "${dep}" "${dst_dir}/"
        fi
      fi
    fi
  done < <("${OTOOL}" -L "${target}" 2>/dev/null | tail -n +2)
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
  # Python extension modules on macOS are typically .so
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
  set_rpaths "${dst_pkg}/${wrapper_target}"
  if [[ "${wrapper_target}" != "${wrapper_basename}" && -e "${dst_pkg}/${wrapper_basename}" ]]; then
    rm -f "${dst_pkg}/${wrapper_basename}"
  fi
  if [[ -n "${so_core}" ]]; then
    "${INSTALL_BIN}" -m 0755 "${so_core}" "${dst_pkg}/$(basename "${so_core}")"
    set_rpaths "${dst_pkg}/$(basename "${so_core}")"
    rewrite_dep_paths "${dst_pkg}/$(basename "${so_core}")"
    copy_local_deps "${dst_pkg}/$(basename "${so_core}")"
  fi

  # Re-export the class at package level for ROS-like import:
  #   from .String import String as String
  local init="${dst_pkg}/__init__.py"
  grep -q "from .${name} import ${name} as ${name}" "${init}" 2>/dev/null || \
    echo "from .${name} import ${name} as ${name}" >> "${init}"
}

# ========= Scan and install all generated types =========
# depth=3 (<pkg>/<ns>/<Type>) directories only
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

# Deduplicate list
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

# Final pass: ensure all installed .so/.dylib resolve to their own directory first
if [[ -x "${INSTALL_NAME_TOOL}" ]]; then
  while IFS= read -r bin; do
      set_rpaths "${bin}"
      rewrite_dep_paths "${bin}"
      copy_local_deps "${bin}"
  done < <(find "${INSTALL_ROOT}" -type f \( -name "*.so" -o -name "*.dylib" \))
fi

# Generic dependency mirroring: for every msg dir, ensure all @loader_path/lib*.dylib deps are present
LIB_INDEX="$(mktemp)"
find "${INSTALL_ROOT}" -type f -name "lib*.dylib" -print | while IFS= read -r f; do
  bn="$(basename "$f")"
  echo "${bn}|${f}"
done | sort -u > "${LIB_INDEX}"

mirror_missing_deps(){
  local dir="$1" dep_path dep_bn src
  [[ -x "${OTOOL}" ]] || return 0
  while IFS= read -r file; do
    while IFS= read -r line; do
      dep_path="$(echo "$line" | awk '{print $1}')"
      case "${dep_path}" in
        @loader_path/lib*.dylib)
          dep_bn="$(basename "${dep_path}")"
          if [[ ! -e "${dir}/${dep_bn}" ]]; then
            src="$(awk -F'|' -v bn="${dep_bn}" '$1==bn{print $2; exit}' "${LIB_INDEX}")"
            if [[ -n "${src}" && -f "${src}" ]]; then
              /bin/cp -p "${src}" "${dir}/"
            fi
          fi
          ;;
      esac
    done < <("${OTOOL}" -L "${file}" 2>/dev/null | tail -n +2)
  done < <(find "${dir}" -maxdepth 1 -type f \( -name "lib*.dylib" -o -name "*Wrapper.so" -o -name "*Wrapper.dylib" \))
}

while IFS= read -r d; do
  mirror_missing_deps "${d}"
done < <(find "${INSTALL_ROOT}" -type d -path "*/msg")

rm -f "${LIB_INDEX}"

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
