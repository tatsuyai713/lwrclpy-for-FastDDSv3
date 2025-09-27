#!/bin/bash
set -e

sudo apt update
sudo apt install -y swig4.1
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

# 2.5) 生成された lib*.so を 1 箇所に集約（冪等）
echo "[INFO] Collecting generated lib*.so to /opt/fast-dds-v3-libs/lib ..."
sudo mkdir -p /opt/fast-dds-v3-libs/lib
# 既存の壊れたリンクは掃除（冪等）
sudo find /opt/fast-dds-v3-libs/lib -xtype l -delete || true
# シンボリックリンクを一括作成
while IFS= read -r sofile; do
  bn=$(basename "$sofile")
  # 既に同名リンク/ファイルがあればスキップ
  if [ ! -e "/opt/fast-dds-v3-libs/lib/$bn" ]; then
    sudo ln -s "$sofile" "/opt/fast-dds-v3-libs/lib/$bn"
  fi
done < <(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' | sort -u)

# 3) パスを通す（このシェル / RC にも追記）
# 3.1: ライブラリ探索（集約先 + 念のため実体ディレクトリも追加）
ADD_LD_DIRS="$(find /opt/fast-dds-v3-libs/python/src -type f -name 'lib*.so' -printf '%h\n' | sort -u | paste -sd: -)"
export LD_LIBRARY_PATH="/opt/fast-dds-v3-libs/lib:${ADD_LD_DIRS}:${LD_LIBRARY_PATH:-}"

# 3.2: Python パス（生成パッケージ）
export PYTHONPATH="/opt/fast-dds-v3-libs/python/src:${PYTHONPATH:-}"

# 3.3: Fast-DDS Python の site-packages も（存在すれば）追加
PY_SITE_PACK="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr ' ' :)"
if [ -n "$PY_SITE_PACK" ]; then
  export PYTHONPATH="${PY_SITE_PACK}:${PYTHONPATH}"
fi

# 4) 確認
python3 - <<'PY'
import sys, os
print("[DBG] PYTHONPATH=", os.environ.get("PYTHONPATH",""))
print("[DBG] LD_LIBRARY_PATH=", os.environ.get("LD_LIBRARY_PATH",""))
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

# ① 生成パッケージ
add_line_once 'export PYTHONPATH=/opt/fast-dds-v3-libs/python/src:$PYTHONPATH' "$BASHRC"

# ② Fast-DDS Python (site-packages)
add_line_once 'export PYTHONPATH="$(echo /opt/fast-dds-v3/lib/python*/site-packages /opt/fast-dds-v3/lib/python*/dist-packages 2>/dev/null | tr '\'' '\'' :):$PYTHONPATH"' "$BASHRC"

# ③ 生成ライブラリ（集約先）+ 念のため実体ディレクトリも
add_line_once 'export LD_LIBRARY_PATH=/opt/fast-dds-v3-libs/lib:$LD_LIBRARY_PATH' "$BASHRC"
add_line_once 'export LD_LIBRARY_PATH="$(find /opt/fast-dds-v3-libs/python/src -type f -name '\''lib*.so'\'' -printf '\''%h\n'\'' | sort -u | paste -sd: -):$LD_LIBRARY_PATH"' "$BASHRC"

echo "追加完了: $BASHRC を再読み込みしてください → source ~/.bashrc"
