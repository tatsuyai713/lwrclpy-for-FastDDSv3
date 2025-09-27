# lwrclpy — Zero-to-Run Guide (Fast DDS v3)

This guide uses only the scripts you have under `scripts/` and assumes the submodule `third_party/ros-data-types-for-fastdds` has been cloned with `--recursive`.

> Tested on Ubuntu 24.04, Python 3.12.

---

## 0) Pull submodules (once)

```bash
git submodule update --init --recursive
```

---

## 1) Install Fast-DDS v3 core, fastddsgen, and the Python bindings

This installs the v3 stack (colcon merge-install) into **`/opt/fast-dds-v3`** and fastddsgen into **`/opt/fast-dds-gen-v3`**.

```bash
cd <this-repo>
bash scripts/install_fastdds_v3_colcon.sh
```

Quick sanity checks:

```bash
which fastddsgen && fastddsgen -version
python3 -c "import fastdds; print('fastdds OK')"
```

---

## 2) Install all ROS DataTypes (single command)

Run **one** script and everything about message types is taken care of: generation, SWIG patching, building, staging to a ROS-like layout, collecting `lib*.so`, and setting up environment variables.

```bash
bash scripts/install_ros_data_types.sh
```

What it does (summary):
- Generates Python bindings for all IDL files under `third_party/ros-data-types-for-fastdds/src` using **`fastddsgen -python`** (with the included SWIG patcher).
- Builds the generated bindings.
- Installs them under **`/opt/fast-dds-v3-libs/python/src/<pkg>/<ns>/...`** so you can do `from std_msgs.msg import String`.
- Collects all generated **`lib*.so`** into **`/opt/fast-dds-v3-libs/lib`** (symlinks) so `_TypeWrapper` can `dlopen` them reliably.
- Exports the required **`PYTHONPATH`** and **`LD_LIBRARY_PATH`** for the current shell and (idempotently) appends them to `~/.bashrc` for persistence.

> If you open a **new** terminal after running this script, your environment should already be ready thanks to the `~/.bashrc` updates.

---

## 3) Run the examples

Open two terminals. **Start the listener first**, then the talker.

**Terminal A (listener):**
```bash
cd <this-repo>
python3 examples/listener_string.py
```

**Terminal B (talker):**
```bash
cd <this-repo>
python3 examples/talker_string.py
```

You should see output like:

```
[recv] hello 0
[recv] hello 1
...
```

---

## 4) Troubleshooting (quick)

- **`ModuleNotFoundError: No module named 'std_msgs'`**  
  Re-run `bash scripts/install_ros_data_types.sh` (it ensures `PYTHONPATH` includes `/opt/fast-dds-v3-libs/python/src`).

- **`ImportError: libBool.so: cannot open shared object file`**  
  Re-run `bash scripts/install_ros_data_types.sh` (it collects symlinks into `/opt/fast-dds-v3-libs/lib` and exports `LD_LIBRARY_PATH`).

- **`AttributeError: module 'fastdds' has no attribute 'ReturnCode_t'`**  
  Fast-DDS v3 uses module-level return code constants (e.g., `fastdds.RETCODE_OK`). The bundled `lwrclpy/subscription.py` already handles this.

- **`AttributeError: '...PubSubType' object has no attribute 'getName'`**  
  Fast-DDS v3 uses `get_name/set_name`. The bundled `lwrclpy/typesupport.py` is compatible with both styles.

---

## 5) One-liner to (re)export env manually (if ever needed)

```bash
export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH; ADD_LD_DIRS="$(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' -printf '%h\n' | sort -u | paste -sd: -)"; export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:${ADD_LD_DIRS}:$LD_LIBRARY_PATH; PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' :)"; [ -n "$PY_SITE_PACK" ] && export PYTHONPATH=${PY_SITE_PACK}:$PYTHONPATH
```

---

## 6) License

- This repository: Apache-2.0  
- Generated code includes portions derived from eProsima templates; please follow their licenses accordingly.