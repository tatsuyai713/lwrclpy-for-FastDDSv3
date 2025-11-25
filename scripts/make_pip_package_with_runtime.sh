#!/usr/bin/env bash
# scripts/make_pip_package_with_runtime.sh
#
# Build a self-contained 'lwrclpy' wheel for Ubuntu venvs with NO search:
#  - Prebuilt ROS message bindings are staged from ._types_python_build_v3/src (no rebuild)
#  - lwrclpy pure-Python sources are staged
#  - Native libs libfastcdr.so/libfastdds.so are copied from /opt/fast-dds-v3/lib
#  - The ENTIRE fastdds site-packages directory is vendored:
#       /opt/fast-dds-v3/lib/pythonX.Y/site-packages/fastdds/
#     (or .../dist-packages/fastdds/ as fallback)
#  - Bootstrap adds _vendor to sys.path and imports the vendored 'fastdds'
#
# Usage:
#   python3 -m venv venv && source venv/bin/activate
#   bash scripts/make_pip_package_with_runtime.sh
#   pip install dist/lwrclpy-*.whl
#   python3 examples/talker_string.py
set -euo pipefail

# ----- Fixed inputs (no search) -----
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="${REPO_ROOT}/scripts"
PKG_NAME="lwrclpy"
PKG_VERSION="${PKG_VERSION:-0.1.0}"

BUILD_ROOT="${BUILD_ROOT:-${REPO_ROOT}/._types_python_build_v3}"   # prebuilt DataTypes
PREFIX_V3="${PREFIX_V3:-/opt/fast-dds-v3}"                          # fixed Fast-DDS prefix

# ----- Outputs -----
STAGING_ROOT="${REPO_ROOT}/._pip_pkg_lwrclpy"
DIST_DIR="${REPO_ROOT}/dist"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "[FATAL] '$1' not found" >&2; exit 1; }; }
need python3
need rsync

[[ -d "${REPO_ROOT}/lwrclpy" ]] || { echo "[FATAL] lwrclpy/ folder not found"; exit 1; }
[[ -x "${SCRIPTS_DIR}/install_python_types.sh" ]] || { echo "[FATAL] scripts/install_python_types.sh not found or not executable"; exit 1; }
[[ -d "${BUILD_ROOT}/src" ]] || { echo "[FATAL] BUILD_ROOT/src not found: ${BUILD_ROOT}/src"; exit 1; }
[[ -d "${PREFIX_V3}/lib" ]] || { echo "[FATAL] ${PREFIX_V3}/lib not found"; exit 1; }

rm -rf "${STAGING_ROOT}" "${DIST_DIR}"
mkdir -p "${STAGING_ROOT}" "${DIST_DIR}"

# Determine X.Y of the CURRENT python (for fixed site/dist-packages path)
PYXY="$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')"

# Fixed fastdds site/dist-packages locations (NO search)
FASTDDS_PKG_SITE="${PREFIX_V3}/lib/python${PYXY}/site-packages/fastdds"
FASTDDS_PKG_DIST="${PREFIX_V3}/lib/python${PYXY}/dist-packages/fastdds"

if [[ -d "${FASTDDS_PKG_SITE}" ]]; then
  FASTDDS_PKG_SRC="${FASTDDS_PKG_SITE}"
elif [[ -d "${FASTDDS_PKG_DIST}" ]]; then
  FASTDDS_PKG_SRC="${FASTDDS_PKG_DIST}"
else
  echo "[FATAL] fastdds package directory not found:"
  echo "  ${FASTDDS_PKG_SITE}"
  echo "  ${FASTDDS_PKG_DIST}"
  echo "Ensure: /opt/fast-dds-v3/lib/python${PYXY}/site-packages/fastdds/_fastdds_python.so exists."
  exit 2
fi

# Validate core native libs exist
[[ -f "${PREFIX_V3}/lib/libfastcdr.so" ]] || { echo "[FATAL] ${PREFIX_V3}/lib/libfastcdr.so not found"; exit 2; }
[[ -f "${PREFIX_V3}/lib/libfastdds.so" ]] || { echo "[FATAL] ${PREFIX_V3}/lib/libfastdds.so not found"; exit 2; }
# Validate python extension inside package
[[ -f "${FASTDDS_PKG_SRC}/_fastdds_python.so" ]] || { echo "[FATAL] ${FASTDDS_PKG_SRC}/_fastdds_python.so not found"; exit 2; }

# ========= 1) Stage prebuilt ROS message bindings (no rebuild) =========
echo "[INFO] Staging prebuilt ROS message packages…"
INSTALL_ROOT="${STAGING_ROOT}" \
BUILD_ROOT="${BUILD_ROOT}" \
bash "${SCRIPTS_DIR}/install_python_types.sh"

# ========= 2) Stage lwrclpy pure-Python sources =========
echo "[INFO] Staging 'lwrclpy' sources…"
rsync -a --exclude='__pycache__' --exclude='*.pyc' "${REPO_ROOT}/lwrclpy/" "${STAGING_ROOT}/lwrclpy/"

echo "[INFO] Staging 'rclpy' compatibility shim…"
rsync -a --exclude='__pycache__' --exclude='*.pyc' "${REPO_ROOT}/rclpy/" "${STAGING_ROOT}/rclpy/"

# ========= 3) Vendor native libs (fixed paths) =========
echo "[INFO] Vendoring native libs → lwrclpy/_vendor/lib"
VEN_LIB_DIR="${STAGING_ROOT}/lwrclpy/_vendor/lib"
mkdir -p "${VEN_LIB_DIR}"
install -m 0644 "${PREFIX_V3}/lib/libfastcdr.so"  "${VEN_LIB_DIR}/"
install -m 0644 "${PREFIX_V3}/lib/libfastdds.so"  "${VEN_LIB_DIR}/"

# ========= 4) Vendor the ENTIRE fastdds package (no search) =========
echo "[INFO] Vendoring fastdds package directory:"
echo "      ${FASTDDS_PKG_SRC}"
VEN_FASTDDS_PARENT="${STAGING_ROOT}/lwrclpy/_vendor"
VEN_FASTDDS_DIR="${VEN_FASTDDS_PARENT}/fastdds"
mkdir -p "${VEN_FASTDDS_PARENT}"
rsync -a "${FASTDDS_PKG_SRC}/" "${VEN_FASTDDS_DIR}/"

# ========= 5) Bootstrap: import vendored fastdds package deterministically =========
echo "[INFO] Injecting bootstrap…"
LWRCLPY_INIT="${STAGING_ROOT}/lwrclpy/__init__.py"
[[ -f "${LWRCLPY_INIT}" ]] || echo "# created by packager" > "${LWRCLPY_INIT}"

cat > "${STAGING_ROOT}/lwrclpy/_bootstrap_fastdds.py" <<'PY'
# -*- coding: utf-8 -*-
# Deterministic loader that imports the vendored 'fastdds' package (no search).
# We insert <lwrclpy/_vendor> at sys.path[0] so 'import fastdds' resolves to the bundled copy,
# preload native libs (libfastdds/fastcdr), then import fastdds (which loads _fastdds_python.so).
import os, sys, ctypes

_pkg_dir = os.path.dirname(__file__)
_vendor_parent = os.path.join(_pkg_dir, "_vendor")
_vendor_lib = os.path.join(_vendor_parent, "lib")      # libfastdds.so, libfastcdr.so
_vendor_fastdds = os.path.join(_vendor_parent, "fastdds")

def _preload_libs(d):
    if not os.path.isdir(d):
        return
    for name in sorted(os.listdir(d)):
        if name.endswith(".so") or ".so." in name:
            fp = os.path.join(d, name)
            try:
                ctypes.CDLL(fp, mode=getattr(ctypes, "RTLD_GLOBAL", os.RTLD_GLOBAL))
            except Exception:
                pass

def _preload_ros_msg_libs():
    base = os.path.dirname(_pkg_dir)  # std_msgs/, geometry_msgs/, …
    for dp, _dn, files in os.walk(base):
        for f in files:
            if f.startswith("lib") and f.endswith(".so"):
                fp = os.path.join(dp, f)
                try:
                    ctypes.CDLL(fp, mode=getattr(ctypes, "RTLD_GLOBAL", os.RTLD_GLOBAL))
                except Exception:
                    pass

def ensure_fastdds():
    # 1) preload core native libs
    _preload_libs(_vendor_lib)

    # 2) import vendored 'fastdds' by putting its parent on sys.path front
    if not os.path.isdir(_vendor_fastdds):
        raise ImportError("Vendored fastdds package missing: " + _vendor_fastdds)
    if _vendor_parent not in sys.path:
        sys.path.insert(0, _vendor_parent)

    # 3) now a plain import resolves to our vendored package
    import fastdds  # noqa: F401 (this triggers loading _fastdds_python.so inside the package)

    # 4) preload ROS msg shared libs (SWIG types depend on RTLD_GLOBAL)
    _preload_ros_msg_libs()
PY

# Prepend bootstrap call to __init__.py (idempotent)
if ! grep -q 'ensure_fastdds()' "${LWRCLPY_INIT}"; then
  tmp="${STAGING_ROOT}/.init.tmp"
  printf '%s\n' "from ._bootstrap_fastdds import ensure_fastdds" "ensure_fastdds()" | cat - "${LWRCLPY_INIT}" > "${tmp}"
  mv -f "${tmp}" "${LWRCLPY_INIT}"
fi

# ========= 6) Optional: set RPATH with patchelf (minimal) =========
if command -v patchelf >/dev/null 2>&1; then
  echo "[INFO] Setting RPATH via patchelf…"
  while IFS= read -r so; do
    patchelf --set-rpath '$ORIGIN:$ORIGIN/../lib' "$so" || true
  done < <(find "${STAGING_ROOT}" -type f -name '*.so' | sort)
else
  echo "[WARN] patchelf not found; relying on ctypes preload only."
fi

# ========= 7) Make wheel metadata (platform-specific via bdist_wheel) =========
echo "[INFO] Writing packaging metadata…"

# PEP 517 backend: use legacy mode so setup.py is honored (bdist_wheel etc.)
cat > "${STAGING_ROOT}/pyproject.toml" <<PYPROJECT
[build-system]
requires = ["setuptools>=64", "wheel"]
build-backend = "setuptools.build_meta:__legacy__"
PYPROJECT

# setup.cfg: metadata & package_data
cat > "${STAGING_ROOT}/setup.cfg" <<PYSETUPCFG
[metadata]
name = ${PKG_NAME}
version = ${PKG_VERSION}
description = lwrclpy with vendored Fast-DDS v3 runtime (fixed /opt layout) and ROS message bindings
long_description = Wheel bundles Fast-DDS runtime (fastdds package incl. _fastdds_python.so), native libs, and lwrclpy.
long_description_content_type = text/plain
author = Your Org

[options]
packages = find:
python_requires = >=3.8
include_package_data = True
zip_safe = False

[options.package_data]
# Include ALL .so files, not only lib*.so
* = **/*.py, **/*Wrapper.*, **/*.so
PYSETUPCFG

# setup.py: force has_ext_modules() to True so wheel is NOT treated as pure-python
# → bdist_wheel will treat it as platform-specific and automatically choose
#    the correct platform tag (e.g., linux_aarch64, linux_x86_64) based on
#    the current Python's sysconfig.get_platform().
cat > "${STAGING_ROOT}/setup.py" <<'PYSETUP'
from setuptools import setup
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        # We ship .so as package_data, but tell setuptools this is a binary dist.
        # This makes the wheel platform-specific instead of py3-none-any.
        return True

if __name__ == "__main__":
    setup(distclass=BinaryDistribution)
PYSETUP

echo "[INFO] Ensuring 'build' module…"
python3 - <<'PY'
import sys, subprocess
try:
    import build  # noqa
except Exception:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-U", "build"])
PY

( cd "${STAGING_ROOT}" && python3 -m build --wheel --outdir "${DIST_DIR}" )

echo
echo "========== Bundled summary =========="
echo "[fastdds core libs]"
ls -l "${STAGING_ROOT}/lwrclpy/_vendor/lib"/libfastcdr.so "${STAGING_ROOT}/lwrclpy/_vendor/lib"/libfastdds.so
echo "[vendored fastdds package]"
ls -l "${STAGING_ROOT}/lwrclpy/_vendor/fastdds"/ | sed 's/^/  /'
echo "===================================="

echo
echo "✅ Wheel ready under: ${DIST_DIR}"
ls -1 "${DIST_DIR}"/${PKG_NAME}-*.whl
echo "Install:"
echo "  pip install \$(ls -1 ${DIST_DIR}/${PKG_NAME}-*.whl | tail -n1)"
echo "Run examples:"
echo "  python3 examples/talker_string.py"
echo "  python3 examples/listener_string.py"
