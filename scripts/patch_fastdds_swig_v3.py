#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Patch helper for Fast DDS v3 SWIG interface (.i) files.
# Goals (minimal, ordered, idempotent):
#   - Provide only a *forward declaration* of SerializedPayload_t to SWIG via %inline.
#   - Include the real C++ headers inside the %{ %} block (no redefinitions on the SWIG side).
#   - Define required macros in the %{ %} block with #ifndef guards.
#   - Insert additions right after the helper includes (std_*.i / typemaps.i), keeping order stable.

import sys, io, re

if len(sys.argv) != 2:
    print("Usage: patch_fastdds_swig_v3.py <path/to/Generated.i>")
    sys.exit(1)

path = sys.argv[1]
txt = io.open(path, "r", encoding="utf-8").read()
orig = txt

def add_after_anchor(lines):
    """Insert given lines once (skip if already present) right after the last helper %include."""
    global txt
    # Anchor: last occurrence among helper includes (std_*.i / typemaps.i / stdint.i).
    it = list(re.finditer(r'(?m)^\s*%include\s+"(?:std_[a-z]+\.i|typemaps\.i|stdint\.i)"\s*$', txt))
    if it:
        ins = it[-1].end()
    else:
        # Fallback: right after %module if helpers are not found.
        m = re.search(r'(?m)^\s*%module[^\n]*\n', txt)
        ins = m.end() if m else 0
    block = ""
    for ln in lines:
        if ln and ln not in txt:
            block += ln + "\n"
    if block:
        txt = txt[:ins] + "\n" + block + txt[ins:]

# 1) Ensure we have a C++ insertion block %{ %}.
if '%{' not in txt:
    m = re.search(r'(?m)^\s*%module[^\n]*\n', txt)
    pos = m.end() if m else 0
    txt = txt[:pos] + "%{\n%}\n" + txt[pos:]

# 2) In the %{ %} block, add macro guards and real C++ headers (once).
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

# 3) For SWIG parsing, expose only a *forward declaration* (avoid redefinition in the .i).
#    %inline informs SWIG of the type name; the actual definition comes from the real headers above.
fwd_decl = r"""%inline %{
namespace eprosima { namespace fastdds { namespace rtps {
    struct SerializedPayload_t;
}}}
%}"""
if "struct SerializedPayload_t;" not in txt:
    add_after_anchor([fwd_decl])

# 4) Add missing SWIG helper imports (right after helper includes only).
add_after_anchor([
    '%include <fastcdr/config.h>',
    '%import(module="fastdds") "fastdds/dds/core/LoanableCollection.hpp"',
    '%import(module="fastdds") "fastdds/dds/core/LoanableTypedCollection.hpp"',
    '%import(module="fastdds") "fastdds/dds/core/LoanableSequence.hpp"',
    '%import(module="fastdds") "fastdds/rtps/common/Types.hpp"',
    '%import(module="fastdds") "fastdds/rtps/history/IPayloadPool.hpp"',
    # Do NOT %include the C++ headers here to avoid SWIG-side redefinitions.
    '%include "stdint.i"',
    '%apply unsigned int { eprosima::fastdds::dds::DataRepresentationId_t };',
])

# 5) Add %extend for SerializedPayload_t if not already present.
#    SWIG sees the type via the forward declaration; the definition is provided by the real headers.
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
    # Prefer inserting before the first %include "*.hpp" if present, otherwise append.
    m = re.search(r'(?m)^\s*%include\s+".+?\.hpp"\s*$', txt)
    if m:
        txt = txt[:m.start()] + extend + "\n" + txt[m.start():]
    else:
        txt = txt.rstrip() + "\n" + extend + "\n"

# 6) Clean up any known bad redefinition blocks (safe pattern).
#    E.g., if someone injected a hand-written "struct SerializedPayload_t { … }" block into the .i, remove it.
txt = re.sub(
    r'(?s)/\* *SWIG-generated SerializedPayload_t start *\*/.*?/\* *SWIG-generated SerializedPayload_t end *\*/',
    '', txt
)

if txt != orig:
    io.open(path, "w", encoding="utf-8", newline="\n").write(txt)
    print(f"[SWIG] patched: {path}")
else:
    print(f"[SWIG] no-change: {path}")
