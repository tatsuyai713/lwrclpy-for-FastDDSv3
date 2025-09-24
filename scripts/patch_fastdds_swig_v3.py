#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Fast DDS v3 の SWIG .i に、必要最小限の追記を順序固定・重複回避で入れる
# ポイント:
#  - SWIG には SerializedPayload_t の「前方宣言」だけ与える（%inline）
#  - C++ 実体は %{ %} に本物のヘッダを include（再定義しない）
#  - 必要マクロを %{ %} 側で #ifndef ガード付きで定義
#  - 追記は helper includes（std_*.i/typemaps.i）直後に限定

import sys, io, re

if len(sys.argv) != 2:
    print("Usage: patch_fastdds_swig_v3.py <path/to/Generated.i>")
    sys.exit(1)

path = sys.argv[1]
txt = io.open(path, "r", encoding="utf-8").read()
orig = txt

def add_after_anchor(lines):
    global txt
    # helper includes の最後をアンカーにする
    it = list(re.finditer(r'(?m)^\s*%include\s+"(?:std_[a-z]+\.i|typemaps\.i|stdint\.i)"\s*$', txt))
    if it:
        ins = it[-1].end()
    else:
        m = re.search(r'(?m)^\s*%module[^\n]*\n', txt)
        ins = m.end() if m else 0
    block = ""
    for ln in lines:
        if ln and ln not in txt:
            block += ln + "\n"
    if block:
        txt = txt[:ins] + "\n" + block + txt[ins:]

# 1) C++ 挿入ブロック %{ %} を確保
if '%{' not in txt:
    m = re.search(r'(?m)^\s*%module[^\n]*\n', txt)
    pos = m.end() if m else 0
    txt = txt[:pos] + "%{\n%}\n" + txt[pos:]

# 2) %{ %} 内にマクロ定義と本物のヘッダを一度だけ入れる
cpp_snippet = r"""/* __FASTDDS_V3_CPP_BLOCK__ */
#ifndef FASTDDS_EXPORTED_API
#define FASTDDS_EXPORTED_API
#endif
#ifndef eProsima_user_DllExport
#define eProsima_user_DllExport
#endif
#include <fastdds/rtps/common/Types.hpp>
#include <fastdds/rtps/history/IPayloadPool.hpp>
#include <fastdds/rtps/common/SerializedPayload.hpp>
using eprosima::fastdds::rtps::octet;
"""
if "/* __FASTDDS_V3_CPP_BLOCK__ */" not in txt:
    txt = re.sub(r'%\}', cpp_snippet + r'\n%}', txt, count=1)

# 3) SWIG 側（パーサ用）には「前方宣言」だけを与える（再定義を避ける）
#    ※ %inline は SWIG に型名を知らせつつ、C++ 的には前方宣言なので多重定義にならない
fwd_decl = r"""%inline %{
namespace eprosima { namespace fastdds { namespace rtps {
    struct SerializedPayload_t;
}}}
%}"""
if "struct SerializedPayload_t;" not in txt:
    add_after_anchor([fwd_decl])

# 4) SWIG helper の不足分（fastcdr/config など）を helper 群の直後にだけ追加
add_after_anchor([
    '%include <fastcdr/config.h>',
    '%import(module="fastdds") "fastdds/dds/core/LoanableCollection.hpp"',
    '%import(module="fastdds") "fastdds/dds/core/LoanableTypedCollection.hpp"',
    '%import(module="fastdds") "fastdds/dds/core/LoanableSequence.hpp"',
    '%import(module="fastdds") "fastdds/rtps/common/Types.hpp"',
    '%import(module="fastdds") "fastdds/rtps/history/IPayloadPool.hpp"',
    # ここで C++ ヘッダの %include はしない（SWIG 再定義を避ける）
    '%include "stdint.i"',
    '%apply unsigned int { eprosima::fastdds::dds::DataRepresentationId_t };',
])

# 5) %extend（未挿入なら追加）。SWIG は前方宣言で型を認識、実体は C++ 側ヘッダで解決
if not re.search(r'%extend\s+eprosima::fastdds::rtps::SerializedPayload_t', txt):
    extend = r"""
%extend eprosima::fastdds::rtps::SerializedPayload_t
{
    void bind(uintptr_t addr, uint32_t len)
    {
        $self->data = reinterpret_cast<eprosima::fastdds::rtps::octet*>(addr);
        $self->length = len;
        $self->max_size = len;
        $self->pos = 0;
        $self->encapsulation = 0;
    }
    uintptr_t data_addr() const
    {
        return reinterpret_cast<uintptr_t>($self->data);
    }
}
"""
    # 可能なら最初の %include "*.hpp" の直前に、なければ末尾へ
    m = re.search(r'(?m)^\s*%include\s+".+?\.hpp"\s*$', txt)
    if m:
        txt = txt[:m.start()] + extend + "\n" + txt[m.start():]
    else:
        txt = txt.rstrip() + "\n" + extend + "\n"

# 6) 既に混入している誤った再定義パターンを掃除（安全な範囲で）
#    SWIG の出力を壊すような「struct SerializedPayload_t { … }」を .i に置いてしまっている場合は除去
txt = re.sub(
    r'(?s)/\* *SWIG-generated SerializedPayload_t start *\*/.*?/\* *SWIG-generated SerializedPayload_t end *\*/',
    '', txt
)

if txt != orig:
    io.open(path, "w", encoding="utf-8", newline="\n").write(txt)
    print(f"[SWIG] patched: {path}")
else:
    print(f"[SWIG] no-change: {path}")