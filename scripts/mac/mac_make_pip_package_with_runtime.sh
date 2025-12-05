#!/usr/bin/env bash
# Build a macOS wheel that bundles:
#   - Generated ROS message/service/action packages from /opt/fast-dds-v3-libs/python/src
#   - lwrclpy + rclpy sources from this repo
#   - fastdds runtime libs (libfastdds.dylib/libfastcdr.dylib) from /opt/fast-dds-v3/lib
#   - Vendored fastdds Python package (including _fastdds_python.so/.dylib)
# Usage:
#   python3 -m venv venv && source venv/bin/activate
#   bash scripts/mac/mac_make_pip_package_with_runtime.sh
#   pip install dist/lwrclpy-*.whl
set -euo pipefail

# ----- Ensure build dependencies -----
echo "[INFO] Ensuring build dependencies..."
python3 -m pip install --upgrade pip setuptools wheel delocate || true

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPTS_DIR="${REPO_ROOT}/scripts"
PKG_NAME="lwrclpy"
PKG_VERSION="${PKG_VERSION:-0.1.1}"

BUILD_ROOT="${BUILD_ROOT:-${REPO_ROOT}/._types_python_build_v3}"           # prebuilt DataTypes tree
PY_INSTALL_ROOT="${PY_INSTALL_ROOT:-}"                                     # optional: already-installed DataTypes
FASTDDS_PREFIX="${FASTDDS_PREFIX:-/opt/fast-dds-v3}"

STAGING_ROOT="${REPO_ROOT}/._pip_pkg_lwrclpy_mac"
DIST_DIR="${REPO_ROOT}/dist"

need(){ command -v "$1" >/dev/null 2>&1 || { echo "[FATAL] '$1' not found" >&2; exit 1; }; }
need python3
need rsync

[[ -d "${REPO_ROOT}/lwrclpy" ]] || { echo "[FATAL] lwrclpy/ folder not found"; exit 1; }
[[ -d "${REPO_ROOT}/rclpy" ]] || { echo "[FATAL] rclpy/ folder not found"; exit 1; }
[[ -d "${FASTDDS_PREFIX}/lib" ]] || { echo "[FATAL] ${FASTDDS_PREFIX}/lib not found"; exit 1; }

rm -rf "${STAGING_ROOT}"
mkdir -p "${STAGING_ROOT}" "${DIST_DIR}"

PYXY="$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')"
FASTDDS_PKG_SITE="${FASTDDS_PREFIX}/lib/python${PYXY}/site-packages/fastdds"
FASTDDS_PKG_DIST="${FASTDDS_PREFIX}/lib/python${PYXY}/dist-packages/fastdds"

if [[ -d "${FASTDDS_PKG_SITE}" ]]; then
  FASTDDS_PKG_SRC="${FASTDDS_PKG_SITE}"
elif [[ -d "${FASTDDS_PKG_DIST}" ]]; then
  FASTDDS_PKG_SRC="${FASTDDS_PKG_DIST}"
else
  echo "[FATAL] fastdds package not found:"
  echo "  ${FASTDDS_PKG_SITE}"
  echo "  ${FASTDDS_PKG_DIST}"
  exit 2
fi

[[ -f "${FASTDDS_PREFIX}/lib/libfastdds.dylib" ]] || { echo "[FATAL] ${FASTDDS_PREFIX}/lib/libfastdds.dylib missing"; exit 2; }
[[ -f "${FASTDDS_PREFIX}/lib/libfastcdr.dylib" ]] || { echo "[FATAL] ${FASTDDS_PREFIX}/lib/libfastcdr.dylib missing"; exit 2; }
[[ -f "${FASTDDS_PKG_SRC}/_fastdds_python.so" || -f "${FASTDDS_PKG_SRC}/_fastdds_python.dylib" ]] || {
  echo "[FATAL] fastdds Python extension not found under ${FASTDDS_PKG_SRC}"; exit 2; }

echo "[INFO] Staging generated ROS packages into ${STAGING_ROOT}"
if [[ -n "${PY_INSTALL_ROOT}" ]]; then
  [[ -d "${PY_INSTALL_ROOT}" ]] || { echo "[FATAL] PY_INSTALL_ROOT not found: ${PY_INSTALL_ROOT}"; exit 1; }
  rsync -a "${PY_INSTALL_ROOT}/" "${STAGING_ROOT}/"
else
  [[ -d "${BUILD_ROOT}/src" ]] || {
    echo "[FATAL] BUILD_ROOT/src not found: ${BUILD_ROOT}/src"
    echo "        Provide PY_INSTALL_ROOT to use an existing install tree."
    exit 1
  }
  INSTALL_ROOT="${STAGING_ROOT}" BUILD_ROOT="${BUILD_ROOT}" \
    bash "${REPO_ROOT}/scripts/mac/mac_install_python_types.sh"
fi

echo "[INFO] Patching action types to add ROS 2-style wrapper classes"
if [[ -f "${SCRIPTS_DIR}/patch_action_types.py" ]]; then
  python3 "${SCRIPTS_DIR}/patch_action_types.py" "${STAGING_ROOT}"
else
  echo "[WARN] patch_action_types.py not found, skipping action type patching"
fi

echo "[INFO] Patching service types to add ROS 2-style wrapper classes"
if [[ -f "${SCRIPTS_DIR}/patch_service_types.py" ]]; then
  python3 "${SCRIPTS_DIR}/patch_service_types.py" "${STAGING_ROOT}"
else
  echo "[WARN] patch_service_types.py not found, skipping service type patching"
fi

echo "[INFO] Staging lwrclpy sources"
rsync -a --exclude='__pycache__' --exclude='*.pyc' "${REPO_ROOT}/lwrclpy/" "${STAGING_ROOT}/lwrclpy/"

echo "[INFO] Staging rclpy compatibility shim"
rsync -a --exclude='__pycache__' --exclude='*.pyc' "${REPO_ROOT}/rclpy/" "${STAGING_ROOT}/rclpy/"

SENSORMSGS_PY_DIR="${REPO_ROOT}/third_party/common_interfaces/sensor_msgs_py/sensor_msgs_py"
if [[ -d "${SENSORMSGS_PY_DIR}" ]]; then
  echo "[INFO] Staging sensor_msgs_py utilities"
  rsync -a --exclude='__pycache__' --exclude='*.pyc' "${SENSORMSGS_PY_DIR}/" "${STAGING_ROOT}/sensor_msgs_py/"
else
  echo "[FATAL] sensor_msgs_py not found at ${SENSORMSGS_PY_DIR}"
  echo "       Run: git submodule update --init --recursive"
  echo "       Or set SKIP_SENSORMSGS_PY=1 to build without pointcloud utilities."
  [[ "${SKIP_SENSORMSGS_PY:-0}" == "1" ]] || exit 2
fi

echo "[INFO] Vendoring Fast DDS native libs"
VEN_LIB_DIR="${STAGING_ROOT}/lwrclpy/_vendor/lib"
mkdir -p "${VEN_LIB_DIR}"
install -m 0644 "${FASTDDS_PREFIX}/lib/libfastdds.dylib" "${VEN_LIB_DIR}/"
install -m 0644 "${FASTDDS_PREFIX}/lib/libfastcdr.dylib" "${VEN_LIB_DIR}/"

echo "[INFO] Vendoring fastdds Python package"
VEN_FASTDDS_PARENT="${STAGING_ROOT}/lwrclpy/_vendor"
VEN_FASTDDS_DIR="${VEN_FASTDDS_PARENT}/fastdds"
mkdir -p "${VEN_FASTDDS_PARENT}"
rsync -a "${FASTDDS_PKG_SRC}/" "${VEN_FASTDDS_DIR}/"

echo "[INFO] Writing bootstrap loader"
LWRCLPY_INIT="${STAGING_ROOT}/lwrclpy/__init__.py"
[[ -f "${LWRCLPY_INIT}" ]] || echo "# auto-generated" > "${LWRCLPY_INIT}"
cat > "${STAGING_ROOT}/lwrclpy/_bootstrap_fastdds.py" <<'PY'
import os, sys, ctypes

_pkg_dir = os.path.dirname(__file__)
_vendor_parent = os.path.join(_pkg_dir, "_vendor")
_vendor_lib = os.path.join(_vendor_parent, "lib")

def _preload_libs(root):
    if not os.path.isdir(root):
        return
    for name in sorted(os.listdir(root)):
        if name.endswith(".so") or name.endswith(".dylib") or ".so." in name:
            fp = os.path.join(root, name)
            try:
                ctypes.CDLL(fp, mode=getattr(ctypes, "RTLD_GLOBAL", os.RTLD_GLOBAL))
            except Exception:
                pass

def _preload_ros_msg_libs():
    base = os.path.dirname(_pkg_dir)
    seen = set()
    for dp, _dn, files in sorted(os.walk(base), key=lambda item: item[0]):
        for f in sorted(files):
            if not f.startswith("lib"):
                continue
            if not (f.endswith(".so") or f.endswith(".dylib") or ".so." in f):
                continue
            if f in seen:
                continue
            seen.add(f)
            fp = os.path.join(dp, f)
            try:
                ctypes.CDLL(fp, mode=getattr(ctypes, "RTLD_GLOBAL", os.RTLD_GLOBAL))
            except Exception:
                pass

def ensure_fastdds():
    _preload_libs(_vendor_lib)
    vendor_fastdds = os.path.join(_vendor_parent, "fastdds")
    if not os.path.isdir(vendor_fastdds):
        raise ImportError("Vendored fastdds missing: " + vendor_fastdds)
    if _vendor_parent not in sys.path:
        sys.path.insert(0, _vendor_parent)
    import fastdds  # noqa: F401
    _preload_ros_msg_libs()
PY

if ! grep -q 'ensure_fastdds()' "${LWRCLPY_INIT}"; then
  tmp="${STAGING_ROOT}/.init.tmp"
  {
    echo "from ._bootstrap_fastdds import ensure_fastdds"
    echo "ensure_fastdds()"
    cat "${LWRCLPY_INIT}"
  } > "${tmp}"
  mv -f "${tmp}" "${LWRCLPY_INIT}"
fi

cat > "${STAGING_ROOT}/pyproject.toml" <<'PYPROJECT'
[build-system]
requires = ["setuptools>=64", "wheel"]
build-backend = "setuptools.build_meta:__legacy__"
PYPROJECT

cat > "${STAGING_ROOT}/setup.cfg" <<PYSETUPCFG
[metadata]
name = ${PKG_NAME}
version = ${PKG_VERSION}
description = lwrclpy bundle (macOS) with Fast DDS runtime and generated ROS DataTypes
long_description = Vendored Fast DDS runtime (fastdds package, libfastdds/libfastcdr) and generated ROS message packages.
long_description_content_type = text/plain
author = Your Org

[options]
packages = find:
python_requires = >=3.8
include_package_data = True
zip_safe = False

[options.package_data]
* = **/*.py, **/*.so, **/*.dylib, **/*Wrapper.*
PYSETUPCFG

cat > "${STAGING_ROOT}/setup.py" <<'PYSETUP'
from setuptools import setup
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        return True

if __name__ == "__main__":
    setup(distclass=BinaryDistribution)
PYSETUP

python3 - <<'PY'
import sys, subprocess
try:
    import build  # noqa
except Exception:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-U", "build"])
PY

( cd "${STAGING_ROOT}" && python3 -m build --wheel --outdir "${DIST_DIR}" )

echo
echo "✅ macOS wheel ready under: ${DIST_DIR}"
ls -1 "${DIST_DIR}/${PKG_NAME}-"*.whl
