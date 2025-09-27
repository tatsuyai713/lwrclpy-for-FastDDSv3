# lwrclpy — Zero-to-Run Guide (Fast DDS v3)

This README shows **two ways** to run the examples:

1) **Use the prebuilt wheel (`.whl`)** — easiest. Works in any Ubuntu venv after `pip install` only.  
2) Build from sources using the scripts under `scripts/` (for developers).

> Tested on Ubuntu 24.04 with Python 3.12.  
> Submodules are expected to be cloned with `--recursive` if you use the source build path.

---

## 1) Easiest: Use the prebuilt wheel

If you already have a wheel in `dist/` (e.g. `dist/lwrclpy-0.1.0-py3-none-any.whl`), you can run the examples **without any environment variables**.

```bash
# 1) Create & activate a venv (recommended)
python3 -m venv venv
source venv/bin/activate

# 2) Install the wheel (replace the filename if different)
pip install dist/lwrclpy-*.whl

# 3) Run the examples (two terminals)
# Terminal A (listener)
python3 examples/listener_string.py

# Terminal B (talker)
python3 examples/talker_string.py
```

What the wheel includes:

- `lwrclpy` Python package
- **Vendored Fast-DDS runtime** (native libs: `libfastdds.so`, `libfastcdr.so`)
- **Vendored Fast-DDS Python package** (`fastdds/` directory including `_fastdds_python.so`)
- **Prebuilt ROS message bindings** (so you can `from std_msgs.msg import String` immediately)

No extra `PYTHONPATH` / `LD_LIBRARY_PATH` tweaks are needed in venvs; the wheel’s bootstrap loads everything from inside the package.

**Troubleshooting for wheel path:**

- If `pip install` succeeds but a sample fails, run:
  ```bash
  python3 - <<'PY'
import pkgutil, lwrclpy, os
print("lwrclpy at:", os.path.dirname(lwrclpy.__file__))
print("has vendored fastdds?",
      os.path.exists(os.path.join(os.path.dirname(lwrclpy.__file__), "_vendor", "fastdds", "_fastdds_python.so")))
PY
  ```
  It should print `True` for the vendored fastdds. If not, rebuild the wheel (see Section 2) and reinstall.

---

## 2) Developer path: Build everything with the provided scripts

This path installs Fast-DDS v3 and generates all ROS DataTypes into system prefixes under `/opt`. Use when you want to **(re)build** or update components.

### 2.0) Pull submodules (once)

```bash
git submodule update --init --recursive
```

### 2.1) Install Fast-DDS v3 core, fastddsgen, and Python bindings

Installs the v3 stack (colcon merge-install) into **`/opt/fast-dds-v3`** and fastddsgen into **`/opt/fast-dds-gen-v3`**.

```bash
bash scripts/install_fastdds_v3_colcon.sh
```

Quick checks:

```bash
which fastddsgen && fastddsgen -version
python3 -c "import fastdds; print('fastdds OK')"
```

### 2.2) Install all ROS DataTypes (single command)

This one script completes **everything for message types**: generation, SWIG patching, building, staging as ROS-like packages, collecting `lib*.so`, and exporting env vars.

```bash
bash scripts/install_ros_data_types.sh
```

After this, a new shell will already have the required env vars via `~/.bashrc`. You can also export manually (Section 2.4).

### 2.3) Run the examples (source build)

Open two terminals:

**Terminal A (listener):**
```bash
python3 examples/listener_string.py
```

**Terminal B (talker):**
```bash
python3 examples/talker_string.py
```

You should see lines like:

```
[recv] hello 0
[recv] hello 1
...
```

### 2.4) (Optional) Manually re-export env (one-liner)

```bash
export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH; ADD_LD_DIRS="$(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' -printf '%h
' | sort -u | paste -sd: -)"; export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:${ADD_LD_DIRS}:$LD_LIBRARY_PATH; PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' :)"; [ -n "$PY_SITE_PACK" ] && export PYTHONPATH=${PY_SITE_PACK}:$PYTHONPATH
```

---

## 3) Rebuilding the wheel yourself (optional)

If you need to produce the wheel that bundles everything (used in Section 1), run:

```bash
# venv recommended
python3 -m venv venv
source venv/bin/activate

# Build the self-contained wheel from the fixed /opt layout
bash scripts/make_pip_package_with_runtime.sh

# Install & test
pip install dist/lwrclpy-*.whl
python3 examples/listener_string.py
python3 examples/talker_string.py
```

This script **does not** rebuild DataTypes; it takes prebuilt bindings from `._types_python_build_v3/src` and packages them with the vendored Fast-DDS runtime from `/opt/fast-dds-v3`.

---

## 4) Troubleshooting (quick)

- **`ModuleNotFoundError: No module named 'std_msgs'`**  
  Use Section 1 (wheel) or run `bash scripts/install_ros_data_types.sh` (Section 2.2).

- **`ImportError: libXxx.so: cannot open shared object file`**  
  Section 1 (wheel) requires nothing else. For Section 2 (source build), rerun `install_ros_data_types.sh` to ensure `LD_LIBRARY_PATH` includes `/opt/fast-dds-v3-libs/lib` and all generated `lib*.so` are symlinked there.

- **QoS enum differences**  
  Fast-DDS v3 changes some names (`HistoryQosPolicyKind` → members on `fastdds`). The provided `lwrclpy/qos.py` already handles v3 safely.

- **`ReturnCode_t` usage**  
  In v3, return codes are exposed as module constants (`fastdds.RETCODE_OK` etc.). Our implementation uses the portable style.

- **Network discovery / domain separation**  
  If you need isolation from other DDS participants, set:  
  `export LWRCL_DOMAIN_ID=<your-id>` (both talker and listener).

---

## 5) License

- This repository: Apache-2.0  
- Generated code contains eProsima Fast-DDS templates; please follow their licenses as applicable.