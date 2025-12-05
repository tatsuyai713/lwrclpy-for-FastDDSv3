#!/usr/bin/env bash
# Purpose:
#   1) Ensure SWIG 4.1 is available and selected via update-alternatives.
#   2) Generate & build Fast DDS Python message bindings (delegates to gen_python_types.sh).
#   3) Install generated Python packages into /opt/fast-dds-v3-libs/python/src (delegates to install_python_types.sh).
#   4) Collect all generated lib*.so into a single directory (/opt/fast-dds-v3-libs/lib) and update runtime paths.
#   5) Export environment variables for the current shell only; persistence must be handled manually.
set -e

# --- Install SWIG 4.1 and register it as the default 'swig' ---
sudo apt update
sudo apt install -y swig4.1
ls -l /usr/bin/swig4.1
sudo update-alternatives --install /usr/bin/swig swig /usr/bin/swig4.1 50

# Verify SWIG is the expected version (4.1.x)
hash -r
which swig
swig -version

# --- Step 1: generate & build all message bindings ---
bash gen_python_types.sh

# --- Step 2: install generated packages into the target prefix ---
current_dir=$(pwd)
echo "Current dir: $current_dir"
BUILD_ROOT="${current_dir}/../._types_python_build_v3"
echo "BUILD_ROOT: $BUILD_ROOT"
sudo INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \
     BUILD_ROOT="${BUILD_ROOT}" \
     bash install_python_types.sh

# --- Step 2.1: add ROS 2-style Service aliases to generated Python modules ---
PY_ALIAS_SCRIPT="${current_dir}/third_party/ros-data-types-for-fastdds/scripts/add_service_aliases.py"
if [[ -f "${PY_ALIAS_SCRIPT}" ]]; then
  echo "[INFO] Injecting Request/Response service aliases into generated Python packages..."
  python3 "${PY_ALIAS_SCRIPT}" /opt/fast-dds-v3-libs/python/src || echo "[WARN] Failed to install service aliases"
fi

# --- Step 2.1.1: add ROS 2-style Service wrapper classes (patch_service_types.py) ---
PY_SERVICE_PATCH="${current_dir}/patch_service_types.py"
if [[ -f "${PY_SERVICE_PATCH}" ]]; then
  echo "[INFO] Injecting ROS 2-style Service wrapper classes into generated Python packages..."
  sudo python3 "${PY_SERVICE_PATCH}" /opt/fast-dds-v3-libs/python/src || echo "[WARN] Failed to patch service types"
fi

# --- Step 2.2: add ROS 2-style Action wrapper classes to generated Python modules ---
PY_ACTION_PATCH="${current_dir}/patch_action_types.py"
if [[ -f "${PY_ACTION_PATCH}" ]]; then
  echo "[INFO] Injecting ROS 2-style Action wrapper classes into generated Python packages..."
  sudo python3 "${PY_ACTION_PATCH}" /opt/fast-dds-v3-libs/python/src || echo "[WARN] Failed to patch action types"
fi

# --- Step 2.5: collect all lib*.so into a single location (idempotent) ---
echo "[INFO] Collecting generated lib*.so to /opt/fast-dds-v3-libs/lib ..."
sudo mkdir -p /opt/fast-dds-v3-libs/lib
# Clean up broken symlinks if any
sudo find /opt/fast-dds-v3-libs/lib -xtype l -delete || true
# Create symlinks for each generated shared library
while IFS= read -r sofile; do
  bn=$(basename "$sofile")
  if [ ! -e "/opt/fast-dds-v3-libs/lib/$bn" ]; then
    sudo ln -s "$sofile" "/opt/fast-dds-v3-libs/lib/$bn"
  fi
done < <(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' | sort -u)

# --- Step 3: export runtime paths for this shell session ---
# Add the central lib dir + all directories that contain lib*.so (helps dlopen from _TypeWrapper)
ADD_LD_DIRS="$(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' -printf '%h\n' | sort -u | paste -sd: -)"
export LD_LIBRARY_PATH="/opt/fast-dds-v3-libs/lib:${ADD_LD_DIRS}:${LD_LIBRARY_PATH:-}"

# Add Python packages (generated message packages)
export PYTHONPATH="/opt/fast-dds-v3-libs/python/src:${PYTHONPATH:-}"

# Also add Fast-DDS Python site-packages if present
PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' :)"
if [ -n "$PY_SITE_PACK" ]; then
  export PYTHONPATH="${PY_SITE_PACK}:${PYTHONPATH}"
fi

# --- Step 4: sanity check import path resolution ---
python3 - <<'PY'
import sys, os
print("[DBG] PYTHONPATH=", os.environ.get("PYTHONPATH",""))
print("[DBG] LD_LIBRARY_PATH=", os.environ.get("LD_LIBRARY_PATH",""))
from std_msgs.msg import String
s = String()
print('OK:', type(s))
PY

cat <<'NOTE'
[NOTE]
- Environment variables are exported only for the current shell session.
- Rerun this script in each shell that needs the generated ROS DataTypes or manually add the PYTHONPATH/LD_LIBRARY_PATH commands printed above.
NOTE
