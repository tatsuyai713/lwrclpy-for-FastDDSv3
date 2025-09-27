# lwrclpy — Zero-to-Run Guide (Fast DDS v3)

**lwrclpy** is an rclpy-compatible Python library built **directly on Fast DDS v3**.  
It is designed to remove the friction of using ROS 2 alongside modern Python ML/AI stacks by avoiding ROS 2's distro/ABI constraints while keeping familiar ROS-like APIs:

- ROS-style imports: `from std_msgs.msg import String`  
- `Node / QoS / publisher / subscription / spin`  
- Works in **any Ubuntu venv** with a single `pip install` when using the runtime‑bundled wheel.

> Tested on Ubuntu 24.04 with Python 3.12.

---

## Contents

1. [Easiest: Use the prebuilt wheel](#1-easiest-use-the-prebuilt-wheel)  
2. [Developer path: Build with scripts](#2-developer-path-build-everything-with-the-provided-scripts)  
3. [ML Example: PyTorch + lwrclpy in a venv](#3-ml-example-pytorch--lwrclpy-in-a-venv)  
4. [Rebuild the wheel (optional)](#4-rebuilding-the-wheel-yourself-optional)  
5. [Troubleshooting](#5-troubleshooting-quick)  
6. [License](#6-license)

---

## 1) Easiest: Use the prebuilt wheel

If you have a wheel in `dist/` (e.g., `dist/lwrclpy-0.1.0-py3-none-any.whl`), you can run the examples **without any environment variables**.

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

- `lwrclpy` Python package (rclpy-like API on Fast DDS v3)
- **Vendored Fast DDS runtime** (`libfastdds.so`, `libfastcdr.so`)
- **Vendored Fast-DDS Python package** (`fastdds/_fastdds_python.so`)
- **Prebuilt ROS message bindings** (so you can immediately `from std_msgs.msg import String`)

No extra `PYTHONPATH` / `LD_LIBRARY_PATH` tweaks are required in venvs; the wheel’s bootstrap loads everything from inside the package.

**Quick check if a sample fails after install:**

```bash
python3 - <<'PY'
import os, lwrclpy
root = os.path.dirname(lwrclpy.__file__)
print("lwrclpy root:", root)
print("vendored fastdds:",
      os.path.exists(os.path.join(root, "_vendor", "fastdds", "_fastdds_python.so")))
PY
```

It should print `True` for “vendored fastdds”.

---

## 2) Developer path: Build everything with the provided scripts

This path installs Fast DDS v3 and generates all ROS DataTypes into system prefixes under `/opt`. Use when you want to **(re)build** or update components.

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

After this, a new shell will already have the required env vars via `~/.bashrc`.  
You can also export manually:

```bash
export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH
ADD_LD_DIRS="$(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' -printf '%h\n' | sort -u | paste -sd: -)"
export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:${ADD_LD_DIRS}:$LD_LIBRARY_PATH
PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' :)"
[ -n "$PY_SITE_PACK" ] && export PYTHONPATH=${PY_SITE_PACK}:$PYTHONPATH
```

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

Expected output on the listener:

```
[recv] hello 0
[recv] hello 1
...
```

---

## 3) ML Example: PyTorch + lwrclpy in a venv

This example demonstrates why lwrclpy helps: you can use any Python version inside a venv and keep modern ML frameworks without ROS 2 distro/ABI constraints—because **lwrclpy talks to Fast DDS directly**.

### 3.1) Create and activate a venv

```bash
python3 -m venv venv
source venv/bin/activate
python -V
# e.g., Python 3.12.x — choose any version your ML stack supports
```

### 3.2) Install your ML stack freely

```bash
pip install --upgrade pip wheel
# Example: CPU-only PyTorch (use your preferred CUDA/ROCm build if needed)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
# Optionally add numpy / transformers / onnxruntime / opencv, etc.
pip install numpy
```

### 3.3) Install lwrclpy (runtime-bundled wheel)

```bash
pip install dist/lwrclpy-*.whl
```

### 3.4) Minimal demo — publish ML inference results over Fast DDS

Create **`examples/listener_ml.py`**:

```python
#!/usr/bin/env python3
import lwrclpy as rclpy
from std_msgs.msg import String

def on_msg(msg: String.String):
    print("[recv]", msg.data())

def main():
    rclpy.init()
    node = rclpy.Node("ml_listener")
    _ = node.create_subscription(String, "ml/topic", on_msg, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Create **`examples/talker_ml.py`**:

```python
#!/usr/bin/env python3
import time
import torch

import lwrclpy as rclpy
from lwrclpy import QoSProfile
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.Node("ml_talker")
    pub = node.create_publisher(String, "ml/topic", QoSProfile(depth=10))

    # Dummy PyTorch "inference" — pick any model/device/mode without ROS 2 constraints
    x = torch.randn(4, 4)
    w = torch.randn(4, 4)
    for i in range(10):
        y = (x @ w).relu().mean().item()
        msg = String.String()
        msg.data(f"[step {i}] score={y:.4f}")
        pub.publish(msg)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### 3.5) Run

**Terminal A (listener first):**
```bash
source venv/bin/activate
python3 examples/listener_ml.py
```

**Terminal B (talker):**
```bash
source venv/bin/activate
python3 examples/talker_ml.py
```

Listener output:
```
[recv] [step 0] score=0.5432
[recv] [step 1] score=0.6120
...
```

### 3.6) Why this fixes the ROS 2 + ML pain

- **No ROS 2 workspace/distro lock-in** — Any Python version in a venv; ML stacks stay compatible and up to date.
- **Binary/ABI isolation** — The wheel ships the Fast DDS runtime and Python extension; no system ROS 2 or DDS required.
- **ROS-like API, zero friction** — `Node/QoS/publisher/subscription/spin` and ROS-style imports without the ROS 2 stack.
- **Fully flexible ML dependencies** — PyTorch/TensorFlow/JAX/ONNXRuntime/NumPy/CUDA/TensorRT, any mix you require.

---

## 4) Rebuilding the wheel yourself (optional)

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

> This script **does not** rebuild DataTypes; it takes prebuilt bindings from `._types_python_build_v3/src` and packages them with the vendored Fast DDS runtime from `/opt/fast-dds-v3`.

---

## 5) Troubleshooting (quick)

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

## 6) License

- This repository: Apache-2.0  
- Generated code contains eProsima Fast-DDS templates; please follow their licenses as applicable.