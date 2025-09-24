# lwrclpy — Zero-to-Run Guide (README)

この README は、**ゼロから** `lwrclpy` を動かすための最短手順です。`from std_msgs.msg import String` を使う talker/listener まで一気に到達します。

---

## 0. 前提

- OS: Ubuntu 22.04 / 24.04（他 OS は適宜読み替え）
- Python: 3.9+（3.10/3.11 推奨）
- 必須ツール: Git / CMake / SWIG（4.2 未満を推奨）/ Java 11+（fastddsgen のビルドに必要）

> メモ: DDS の他実装と干渉させない場合は `export LWRCL_DOMAIN_ID=<任意>` を利用してください（デフォルト 0）。

---

## 1. 取得（Submodule もまとめて取得）

```bash
git clone --recursive <YOUR_FORK_OR_THIS_REPO_URL> lwrclpy_ws
cd lwrclpy_ws
```

> `--recursive` で Submodule（ROS 由来の IDL 群）も自動取得されます。追加の説明は不要です。

---
