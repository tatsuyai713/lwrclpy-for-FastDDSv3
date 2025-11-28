# lwrclpy — Zero-to-Run Guide (Fast DDS v3)

**lwrclpy** is an rclpy-compatible Python library built directly on Fast DDS v3. It ships ROS-like APIs (`Node`, `QoS`, `publisher`, `subscription`, `spin`) without ROS 2 distro/ABI constraints. Message fields can be set with ROS 2–style setters (`msg.data("hi")`) or plain attributes (`msg.data = "hi"`); lwrclpy clones messages on publish/receive so both styles work.

> ✅ **Now tested end-to-end on macOS Sonoma (Apple Silicon)** — full Fast DDS v3 toolchain, ROS DataTypes generation, and packaged wheels are available via the scripts under `scripts/mac/`. Linux (Ubuntu) instructions remain unchanged.

> Tested on Ubuntu 24.04 with Python 3.12.

---

## Contents

1. [Quickstart (prebuilt wheel)](#quickstart-prebuilt-wheel)
2. [macOS setup (Homebrew + scripts)](#macos-setup-homebrew--scripts)
3. [Full build & package from this repo](#full-build--package-from-this-repo)
4. [Examples](#examples)
5. [ROS 2 interoperability](#ros-2-interoperability)
6. [Troubleshooting](#troubleshooting)
7. [License](#license)

---

## Quickstart (prebuilt wheel)

If you already have a wheel in `dist/` (e.g., `dist/lwrclpy-0.1.0-*.whl`), you can run the samples with no extra environment variables.

```bash
# 1) Create & activate a venv (recommended)
python3 -m venv venv
source venv/bin/activate

# 2) Install the wheel
pip install dist/lwrclpy-*.whl

# 3) Run the basic pub/sub example (two terminals)
# Terminal A (listener)
python3 examples/pubsub/string/listener.py
# Terminal B (talker)
python3 examples/pubsub/string/talker.py
```

Quick sanity check after install:

```bash
python3 - <<'PY'
import os, lwrclpy
root = os.path.dirname(lwrclpy.__file__)
print("lwrclpy root:", root)
print("vendored fastdds present:",
      os.path.exists(os.path.join(root, "_vendor", "fastdds", "_fastdds_python.so")))
PY
```

---

## macOS setup (Homebrew + scripts)

The repository now ships dedicated scripts under `scripts/mac/` so Apple Silicon (Sonoma) can run Fast DDS v3 + lwrclpy natively.

1. **Install Homebrew (if needed)**  
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```
   Then add Homebrew to your shell (`eval "$(/opt/homebrew/bin/brew shellenv)"` for Apple Silicon).

2. **Install Xcode Command Line Tools + core deps**  
   ```bash
   xcode-select --install   # once
   brew install cmake ninja git pkg-config tinyxml2 wget curl swig gradle openssl@3 python@3.11
   ```

3. **Install Fast DDS v3 stack (colcon + fastdds_python)**  
   ```bash
   bash scripts/mac/mac_install_fastdds_v3_colcon.sh
   ```
   This builds and installs everything under `/opt/fast-dds-v3` and `/opt/fast-dds-gen-v3`, pinning standalone Asio 1.12.x so legacy APIs still work.

4. **Generate & install ROS DataTypes**  
   ```bash
   bash scripts/mac/mac_install_ros_data_types.sh
   ```
   This mirrors the IDL tree, runs Fast DDS Gen, builds every package, installs them into `/opt/fast-dds-v3-libs/python/src`, and exports `PYTHONPATH` / `DYLD_LIBRARY_PATH`. Service Request/Response aliases are also injected automatically.

5. **(Optional) Build a macOS wheel with bundled runtime**  
   ```bash
   python3 -m venv venv && source venv/bin/activate
   bash scripts/mac/mac_make_pip_package_with_runtime.sh
   pip install dist/lwrclpy-*-macosx*.whl
   ```
   The wheel contains the generated ROS packages, vendored `fastdds` module, and Fast DDS native libraries, so you can run the examples without touching `/opt`.

6. **Sanity check**
   ```bash
   python3 - <<'PY'
   import lwrclpy, os
   print("lwrclpy:", lwrclpy.__file__)
   from std_msgs.msg import String
   print("std_msgs OK:", String)
   PY
   ```

Examples and troubleshooting sections below apply equally to Linux and macOS. If you ever need to re-check your environment, confirm that `~/.zshrc` contains the exported `PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH` (and related `DYLD_LIBRARY_PATH`) entries.

---

## Full build & package from this repo

This path installs Fast DDS v3, generates ROS DataTypes, and builds a self-contained wheel using the provided scripts. Result: a wheel like `dist/lwrclpy-<version>-*.whl` that you can pip-install in any venv.

### 0) Fetch submodules
```bash
git submodule update --init --recursive
```

### 1) Install Fast DDS v3 toolchain
Installs Fast DDS v3, fastddsgen, and Python bindings under `/opt/fast-dds-v3` and `/opt/fast-dds-gen-v3`.
```bash
bash scripts/install_fastdds_v3_colcon.sh
```
Sanity checks:
```bash
which fastddsgen && fastddsgen -version
python3 -c "import fastdds; print('fastdds OK')"
```

### 2) Generate and install ROS DataTypes
Generates all message/service/action bindings, applies SWIG patches, builds, stages as ROS-like packages, and exports env vars.
```bash
bash scripts/install_ros_data_types.sh
```
Notes:
- A new shell will have the required env vars via `~/.bashrc`.  
- If you need manual export, see the script output for `PYTHONPATH` / `LD_LIBRARY_PATH` snippets.
- The script also injects ROS 2-style service aliases (e.g., `SetBool.Request`) into the generated Python packages so rclpy code runs unchanged.

### 3) Build the self-contained wheel (inside a venv)
This packages lwrclpy + vendored Fast DDS runtime + generated Python types from `._types_python_build_v3/src`.
```bash
python3 -m venv venv
source venv/bin/activate
bash scripts/make_pip_package_with_runtime.sh
```
Artifacts:
- `dist/lwrclpy-<version>-*.whl` (runtime-bundled wheel)

### 4) Install and test the new wheel
```bash
pip install dist/lwrclpy-*.whl
python3 examples/pubsub/string/listener.py   # Terminal A
python3 examples/pubsub/string/talker.py     # Terminal B
```

The repository now ships a drop-in `rclpy` shim, so you can launch the official ROS 2 samples without edits. From this repo’s root:

```bash
# Minimal publisher from the official ros2/examples repo (already cloned under third_party/)
python3 third_party/ros2_examples/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
# In another terminal, start the matching subscriber
python3 third_party/ros2_examples/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

---

## Examples

- Basic pub/sub (string): `examples/pubsub/string/{listener.py,talker.py}`
- Executors: `examples/executor/{single_node_spin.py,multithreaded_spin.py}`
- Timers/parameters/services: see `examples/timers/`, `examples/parameters/`, `examples/services/`.
- Actions: `examples/actions/{fibonacci_action_server.py,fibonacci_action_client.py}` demonstrates the bundled action server/client.
- Guard conditions: `examples/guard_condition/trigger_guard_condition.py` demonstrates `Node.create_guard_condition`.
- ML demo with PyTorch: `examples/pubsub/ml/` (requires your ML deps).

For parity testing you can also run the unmodified ROS 2 rclpy samples mirrored in `third_party/ros2_examples/rclpy/…` thanks to the bundled compatibility shim. Service aliases (e.g., `SetBool.Request`) are auto-generated when you run `scripts/install_ros_data_types.sh`, so plain `from std_msgs.srv import SetBool` works exactly like upstream ROS 2.

Example action round-trip with the bundled demos (two shells):

```bash
# Terminal A
python3 examples/actions/fibonacci_action_server.py

# Terminal B
python3 examples/actions/fibonacci_action_client.py
```

---

## ROS 2 interoperability

lwrclpy nodes participate in the same DDS/RTPS network as ROS 2 nodes.

- **Domain**: match IDs. ROS 2 uses `ROS_DOMAIN_ID`; lwrclpy uses `LWRCL_DOMAIN_ID`.
  ```bash
  export ROS_DOMAIN_ID=0
  export LWRCL_DOMAIN_ID=0
  ```
- **Topic names/types**: must match (e.g., `std_msgs/String` on `chatter`).
- **QoS**: align history depth, reliability, durability (defaults mirror common ROS 2 settings).
- **Discovery**: ensure UDP/RTPS ports (7400+ range) aren’t blocked at startup.

Example (ROS 2 listener ↔ lwrclpy talker):
- Terminal A (ROS 2): `ros2 run demo_nodes_cpp listener`
- Terminal B (lwrclpy): `python3 examples/pubsub/string/talker.py`

Reverse direction works the same (ROS 2 talker → lwrclpy listener).

---

## Troubleshooting

- `ModuleNotFoundError: std_msgs`  
  Use the runtime-bundled wheel or run `bash scripts/install_ros_data_types.sh`.

- `ImportError: libXxx.so` (source builds)  
  Ensure `/opt/fast-dds-v3-libs/lib` and collected `lib*.so` paths from the script are on `LD_LIBRARY_PATH`.

- QoS enum differences / ReturnCode_t  
  Fast DDS v3 exposes return codes as module constants; `lwrclpy/qos.py` and code paths handle v3 safely.

- Discovery/domain isolation  
  Set `export LWRCL_DOMAIN_ID=<id>` on all lwrclpy processes (and `ROS_DOMAIN_ID` on ROS 2 if interop).

- Zero-copy (SHM) check  
  After discovery, data may use SHM. Inspect `/dev/shm/fastdds_*` and `lsof -p <PID> | grep /dev/shm`.

---

## License

- This repository: Apache-2.0  
- Generated code includes eProsima Fast-DDS templates; follow their licenses where applicable.

---

# lwrclpy — ゼロから動かすためのガイド（Fast DDS v3）

**lwrclpy** は Fast DDS v3 上に直接構築した rclpy 互換の Python ライブラリです。ROS 2 のディストリ/ABI 制約を避けながら、`Node` / `QoS` / `publisher` / `subscription` / `spin` といった ROS 風 API を提供します。メッセージフィールドは ROS 2 風のセッター（`msg.data("hi")`）でも属性代入（`msg.data = "hi"`）でも設定でき、publish/receive 時にクローンされるためどちらのスタイルも使えます。

> ✅ **macOS Sonoma (Apple Silicon) 対応** — `scripts/mac/` 以下のスクリプトで Fast DDS v3 ツールチェーン、ROS DataTypes 生成、ランタイム同梱ホイールまで一通り動作確認済みです。Linux (Ubuntu) 向け手順はこれまで通りです。

> Ubuntu 24.04 / Python 3.12 で動作確認済み。

---

## 目次

1. [クイックスタート（事前ビルド済みホイール）](#クイックスタート事前ビルド済みホイール)
2. [macOS セットアップ（Homebrew + スクリプト）](#macos-セットアップhomebrew--スクリプト)
3. [このリポジトリからビルドしてパッケージ化](#このリポジトリからビルドしてパッケージ化)
4. [サンプル](#サンプル)
5. [ROS 2 との相互運用](#ros-2-との相互運用)
6. [トラブルシューティング](#トラブルシューティング)
7. [ライセンス](#ライセンス)

---

## クイックスタート（事前ビルド済みホイール）

`dist/` にホイール（例: `dist/lwrclpy-0.1.0-*.whl`）がある場合、追加の環境変数なしですぐ動かせます。

```bash
# 1) venv を作成して有効化（推奨）
python3 -m venv venv
source venv/bin/activate

# 2) ホイールをインストール
pip install dist/lwrclpy-*.whl

# 3) 基本的な pub/sub サンプルを実行（2 端末）
# 端末 A（リスナー）
python3 examples/pubsub/string/listener.py
# 端末 B（トーカー）
python3 examples/pubsub/string/talker.py
```

インストール後の簡易チェック:

```bash
python3 - <<'PY'
import os, lwrclpy
root = os.path.dirname(lwrclpy.__file__)
print("lwrclpy root:", root)
print("vendored fastdds present:",
      os.path.exists(os.path.join(root, "_vendor", "fastdds", "_fastdds_python.so")))
PY
```

---

## macOS セットアップ（Homebrew + スクリプト）

`scripts/mac/` 配下には Apple Silicon (Sonoma) で Fast DDS v3 + lwrclpy を動かすための専用スクリプトが揃っています。

1. **Homebrew をインストール（未導入なら）**  
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```
   Apple Silicon の場合は `eval "$(/opt/homebrew/bin/brew shellenv)"` でパスを通してください。

2. **Xcode Command Line Tools + 依存パッケージ**  
   ```bash
   xcode-select --install   # 初回のみ
   brew install cmake ninja git pkg-config tinyxml2 wget curl swig gradle openssl@3 python@3.11
   ```

3. **Fast DDS v3 スタック（colcon + fastdds_python）をインストール**  
   ```bash
   bash scripts/mac/mac_install_fastdds_v3_colcon.sh
   ```
   `/opt/fast-dds-v3` / `/opt/fast-dds-gen-v3` にツールチェーン一式が展開され、旧 Asio 1.12.x を優先するように構成されます。

4. **ROS DataTypes の生成・インストール**  
   ```bash
   bash scripts/mac/mac_install_ros_data_types.sh
   ```
   IDL ミラーリング → fastddsgen → SWIG ビルド → `/opt/fast-dds-v3-libs/python/src` へのインストールまで自動化され、`PYTHONPATH` / `DYLD_LIBRARY_PATH` もエクスポートされます（`~/.zshrc` に追記されるため次回以降のシェルでも有効）。

5. **（任意）ランタイム同梱ホイールを macOS 向けに作成**  
   ```bash
   python3 -m venv venv && source venv/bin/activate
   bash scripts/mac/mac_make_pip_package_with_runtime.sh
   pip install dist/lwrclpy-*-macosx*.whl
   ```
   生成済み ROS パッケージや `fastdds` モジュール、`libfastdds.dylib` などをすべて同梱したホイールが `dist/` に出力されます。

6. **動作確認**
   ```bash
   python3 - <<'PY'
   import lwrclpy, os
   print("lwrclpy:", lwrclpy.__file__)
   from std_msgs.msg import String
   print("std_msgs OK:", String)
   PY
   ```

以降のサンプルやトラブルシュート手順は Linux と共通です。必要に応じて `PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH` などがシェルに設定されているか確認してください。

---

## このリポジトリからビルドしてパッケージ化

Fast DDS v3 をインストールし、ROS DataTypes を生成した上で、ランタイム同梱ホイールを作ります。結果として `dist/lwrclpy-<version>-*.whl` が得られ、どの venv でも `pip install` できます。

### 0) サブモジュール取得
```bash
git submodule update --init --recursive
```

### 1) Fast DDS v3 ツールチェーンをインストール
Fast DDS v3 / fastddsgen / Python バインディングを `/opt/fast-dds-v3` と `/opt/fast-dds-gen-v3` に配置します。
```bash
bash scripts/install_fastdds_v3_colcon.sh
```
動作確認:
```bash
which fastddsgen && fastddsgen -version
python3 -c "import fastdds; print('fastdds OK')"
```

### 2) ROS DataTypes を生成・インストール
全メッセージ/サービス/アクションを生成し、SWIG パッチを適用してビルド・配置、環境変数を設定します。
```bash
bash scripts/install_ros_data_types.sh
```
備考:
- 新しいシェルでは `~/.bashrc` 経由で環境変数が適用されます。手動設定が必要ならスクリプト出力を参照してください。
- 実行後には生成済み Python パッケージに ROS 2 互換のサービスラッパー（`SetBool.Request` など）が自動で追加されるため、オリジナルの rclpy コードをそのまま使用できます。

### 3) ランタイム同梱ホイールをビルド（venv 内）
`._types_python_build_v3/src` の生成物と Fast DDS ランタイムを同梱したホイールを作ります。
```bash
python3 -m venv venv
source venv/bin/activate
bash scripts/make_pip_package_with_runtime.sh
```
生成物:
- `dist/lwrclpy-<version>-*.whl`

### 4) 新しいホイールをインストールしてテスト
```bash
pip install dist/lwrclpy-*.whl
python3 examples/pubsub/string/listener.py   # 端末 A
python3 examples/pubsub/string/talker.py     # 端末 B
```

---

## サンプル

- 文字列 pub/sub: `examples/pubsub/string/{listener.py,talker.py}`
- Executor: `examples/executor/{single_node_spin.py,multithreaded_spin.py}`
- Timers / Parameters / Services: `examples/timers/`, `examples/parameters/`, `examples/services/`
- PyTorch を用いた ML デモ: `examples/pubsub/ml/`（必要に応じて ML 依存を導入）

`third_party/ros2_examples/` に ROS 2 Jazzy ブランチの rclpy サンプルをそのまま配置しているため、`python3 third_party/ros2_examples/rclpy/...` のように実行すればオリジナルの rclpy コードをそのまま動かせます。`scripts/install_ros_data_types.sh` 実行時には Request/Response を束ねたサービスラッパー（`SetBool.Request` 等）が自動追加されるので、ROS 2 と同じ import で利用できます。

---

## ROS 2 との相互運用

同じ DDS/RTPS ネットワーク上で動作し、ROS 2 と pub/sub できます。

- **ドメイン**: ID を一致させる。ROS 2 は `ROS_DOMAIN_ID`, lwrclpy は `LWRCL_DOMAIN_ID`。
  ```bash
  export ROS_DOMAIN_ID=0
  export LWRCL_DOMAIN_ID=0
  ```
- **トピック名 / 型**: 一致させる（例: `std_msgs/String` の `chatter`）。
- **QoS**: 履歴深さ、信頼性、永続性を合わせる（デフォルトは一般的な ROS 2 設定に近い）。
- **ディスカバリ**: 起動時に UDP/RTPS ポート（7400 以降）をブロックしない。

例（ROS 2 リスナー ↔ lwrclpy トーカー）:
- 端末 A (ROS 2): `ros2 run demo_nodes_cpp listener`
- 端末 B (lwrclpy): `python3 examples/pubsub/string/talker.py`

逆方向（ROS 2 トーカー → lwrclpy リスナー）も同様です。

---

## トラブルシューティング

- `ModuleNotFoundError: std_msgs`  
  ランタイム同梱ホイールを使うか、`bash scripts/install_ros_data_types.sh` を実行してください。

- `ImportError: libXxx.so`（ソースビルド時）  
  `/opt/fast-dds-v3-libs/lib` とスクリプトで収集した `lib*.so` のパスが `LD_LIBRARY_PATH` に入っているか確認してください。

- QoS enum / ReturnCode_t の差分  
  Fast DDS v3 は return code をモジュール定数で提供します。`lwrclpy/qos.py` などで v3 に対応済みです。

- ディスカバリ / ドメイン分離  
  lwrclpy 側で `export LWRCL_DOMAIN_ID=<id>` を設定（ROS 2 と連携する場合は `ROS_DOMAIN_ID` も合わせる）。

- ゼロコピー（SHM）確認  
  ディスカバリ後に SHM を使う場合があります。`/dev/shm/fastdds_*` や `lsof -p <PID> | grep /dev/shm` を確認してください。

---

## ライセンス

- 本リポジトリ: Apache-2.0  
- 生成コードには eProsima Fast-DDS のテンプレートが含まれるため、該当箇所のライセンスにも従ってください。
