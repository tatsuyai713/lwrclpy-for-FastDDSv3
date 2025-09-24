#!/usr/bin/env bash
# install_python_types.sh (v2)
# SWIG 生成物を ROS 風パッケージ階層に配置し、
# `from <pkg>.<ns> import <Type>` で使えるようにする。

set -euo pipefail

# ========= 設定 =========
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
: "${BUILD_ROOT:=}"   # 例: /media/psf/Home/repos/lwrclpy/._types_python_build_v3
: "${INSTALL_ROOT:=/opt/fast-dds-v3-libs/python/src}"  # Python パッケージのルート

log(){ echo "$@" >&2; }
need(){ command -v "$1" >/dev/null 2>&1 || { log "[FATAL] '$1' not found"; exit 1; }; }
need python3
mkdir -p "${INSTALL_ROOT}"

# ========= BUILD_ROOT 自動検出 =========
if [[ -z "${BUILD_ROOT}" ]]; then
  # 候補: リポ直下・一つ上など
  for d in \
    "${ROOT_DIR}/._types_python_build_v3" \
    "$(cd "${ROOT_DIR}/.." && pwd)/._types_python_build_v3" \
    ; do
    [[ -d "$d/src" ]] && BUILD_ROOT="$d" && break
  done
fi
if [[ -z "${BUILD_ROOT}" || ! -d "${BUILD_ROOT}/src" ]]; then
  log "[FATAL] BUILD_ROOT not found. 指定例:"
  log "  BUILD_ROOT=/media/psf/Home/repos/lwrclpy/._types_python_build_v3 \\"
  log "    INSTALL_ROOT=/opt/fast-dds-v3-libs/python/src \\"
  log "    bash scripts/install_python_types.sh"
  exit 1
fi
log "[INFO] BUILD_ROOT=${BUILD_ROOT}"
log "[INFO] INSTALL_ROOT=${INSTALL_ROOT}"

# ========= ヘルパ =========
ensure_init(){
  local d="$1"
  [[ -d "$d" ]] || mkdir -p "$d"
  [[ -f "$d/__init__.py" ]] || echo '# auto-generated' > "$d/__init__.py"
}

# 1 Type をインストール
install_one(){
  local type_dir="$1"   # ex) .../_types_python_build_v3/src/std_msgs/msg/String
  local rel="${type_dir#${BUILD_ROOT}/src/}"   # std_msgs/msg/String
  local pkg_dir="$(dirname "${rel}")"          # std_msgs/msg
  local name="$(basename "${rel}")"            # String
  local bld="${type_dir}/build"

  # SWIG 出力の探索
  local py_src="" so_wrapper="" so_core=""
  [[ -f "${bld}/${name}.py" ]] && py_src="${bld}/${name}.py"
  # _NameWrapper.*（.so/.pyd）
  mapfile -t _cands < <(find "${bld}" -maxdepth 1 -type f -name "_${name}Wrapper.*" 2>/dev/null | sort)
  [[ ${#_cands[@]} -gt 0 ]] && so_wrapper="${_cands[0]}"
  [[ -f "${bld}/lib${name}.so" ]] && so_core="${bld}/lib${name}.so"

  if [[ -z "${py_src}" || -z "${so_wrapper}" ]]; then
    log "[SKIP] ${rel}: SWIG 出力不足 (py='${py_src:-}', wrapper='${so_wrapper:-}')"
    return 0
  fi

  local dst_pkg="${INSTALL_ROOT}/${pkg_dir}"
  ensure_init "${INSTALL_ROOT}/$(dirname "${pkg_dir}")"   # 例: std_msgs
  ensure_init "${dst_pkg}"                                # 例: std_msgs/msg

  log "[INST] ${pkg_dir}/${name}"
  install -m 0644 "${py_src}" "${dst_pkg}/${name}.py"
  install -m 0755 "${so_wrapper}" "${dst_pkg}/$(basename "${so_wrapper}")"
  [[ -n "${so_core}" ]] && install -m 0755 "${so_core}" "${dst_pkg}/lib${name}.so"

  # __init__.py にクラスを直接エクスポート
  # String.py 内の class String を公開: from .String import String as String
  # 既に同行があれば追加しない
  local init="${dst_pkg}/__init__.py"
  grep -q "from .${name} import ${name} as ${name}" "${init}" 2>/dev/null || \
    echo "from .${name} import ${name} as ${name}" >> "${init}"
}

# ========= 走査してインストール =========
mapfile -t TYPE_DIRS < <(find "${BUILD_ROOT}/src" -mindepth 3 -maxdepth 3 -type d -regex '.*/\(msg\|srv\|action\)/[^/]+$' | sort)
[[ ${#TYPE_DIRS[@]} -gt 0 ]] || { log "[FATAL] 生成ディレクトリが見つかりません: ${BUILD_ROOT}/src/*/(msg|srv|action)/*"; exit 1; }

for d in "${TYPE_DIRS[@]}"; do
  install_one "${d}"
done

cat <<EOF

[OK] インストール完了

実行前に一度だけ環境変数を通してください:
  export PYTHONPATH=${INSTALL_ROOT}:\$PYTHONPATH
  export LD_LIBRARY_PATH=${INSTALL_ROOT}:\$LD_LIBRARY_PATH

動作確認:
  python3 - <<'PY'
from std_msgs.msg import String
s = String()   # クラスとして直接使えます
print('OK:', type(s))
PY
EOF