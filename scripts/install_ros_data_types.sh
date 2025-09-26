#!/bin/bash
set -e

sudo apt update
sudo apt install -y \
    swig4.1
ls -l /usr/bin/swig4.1

# swig の代替に登録（優先度 50 は任意）
sudo update-alternatives --install /usr/bin/swig swig /usr/bin/swig4.1 50

# 反映確認
hash -r
which swig
swig -version   # ← 4.1.x が出ればOK

# 1) 既存スクリプトで生成 & ビルド
bash gen_python_types.sh

# 2) 正しい BUILD_ROOT を使ってインストール
current_dir=$(pwd)
echo "Current dir: $current_dir"
BUILD_ROOT=${current_dir}/../._types_python_build_v3
echo "BUILD_ROOT: $BUILD_ROOT"
sudo INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \
     BUILD_ROOT=${BUILD_ROOT} \
     bash install_python_types.sh

# 3) パスを通す（シェルの RC にも追記推奨）
export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH
export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/python/src:$LD_LIBRARY_PATH

# 4) 確認
python3 - <<'PY'
from std_msgs.msg import String
s = String()
print('OK:', type(s))
PY

BASHRC="$HOME/.bashrc"

add_line_once() {
  local line="$1"
  local file="$2"
  grep -qxF "$line" "$file" || echo "$line" >> "$file"
}

# ① lwrclpy 生成物
add_line_once 'export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH' "$BASHRC"
add_line_once 'export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/python/src:$LD_LIBRARY_PATH' "$BASHRC"

# ② fastdds 本体 Python バインディング
add_line_once 'export PYTHONPATH="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr '\'' '\'' :):$PYTHONPATH"' "$BASHRC"

echo "追加完了: $BASHRC を再読み込みしてください → source ~/.bashrc"
