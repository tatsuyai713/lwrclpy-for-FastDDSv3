# lwrclpy — Zero-to-Run Guide (Fast DDS v3)

**lwrclpy** is an rclpy-compatible Python library built directly on Fast DDS v3. It ships ROS-like APIs (`Node`, `QoS`, `publisher`, `subscription`, `spin`) without ROS 2 distro/ABI constraints. Message fields can be set with ROS 2–style setters (`msg.data("hi")`) or plain attributes (`msg.data = "hi"`); lwrclpy clones messages on publish/receive so both styles work.

> Tested on Ubuntu 24.04 with Python 3.12.

---

## Contents

1. [Quickstart (prebuilt wheel)](#quickstart-prebuilt-wheel)
2. [Full build & package from this repo](#full-build--package-from-this-repo)
3. [Examples](#examples)
4. [ROS 2 interoperability](#ros-2-interoperability)
5. [Troubleshooting](#troubleshooting)
6. [License](#license)

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

---

## Examples

- Basic pub/sub (string): `examples/pubsub/string/{listener.py,talker.py}`
- Executors: `examples/executor/{single_node_spin.py,multithreaded_spin.py}`
- Timers/parameters/services: see `examples/timers/`, `examples/parameters/`, `examples/services/`.
- ML demo with PyTorch: `examples/pubsub/ml/` (requires your ML deps).

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

> Ubuntu 24.04 / Python 3.12 で動作確認済み。

---

## 目次

1. [クイックスタート（事前ビルド済みホイール）](#クイックスタート事前ビルド済みホイール)
2. [このリポジトリからビルドしてパッケージ化](#このリポジトリからビルドしてパッケージ化)
3. [サンプル](#サンプル)
4. [ROS 2 との相互運用](#ros-2-との相互運用)
5. [トラブルシューティング](#トラブルシューティング)
6. [ライセンス](#ライセンス)

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
